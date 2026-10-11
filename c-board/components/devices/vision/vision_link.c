/**
 * @file vision_link.c
 * @brief 视觉链路实现
 * @date 2026-09-11
 *
 * 并发模型（与仓库其余部分一致，不使用 RTOS IPC）：
 * - 目标帧写入发生在 5 ms 电机任务（USARTDispatchPending -> 回调）；
 * - 遥测帧发送发生在 20 ms 控制任务；
 * - 诊断任务只读。
 * 共享状态一律用 RmCritical_Enter/Exit 做快照，无锁无阻塞。
 */

#include "vision_link.h"

#include <string.h>

#include "bsp_usart.h"
#include "daemon.h"
#include "rm_critical.h"
#include "vision_link_state.h"

/*============================================================================
 * 模块状态（固定静态存储，无堆）
 *============================================================================*/
static USARTInstance *vision_usart;
static DaemonInstance *vision_daemon;

static RmVisionRxStream rx_stream;

/** 最近一帧目标与遥测快照，跨任务读取须经临界区。 */
static RmVisionTarget latest_target;
static RmVisionTelemetry latest_telemetry;
static bool telemetry_valid;
static RmVisionLinkState link_state;

/** 发送侧状态（仅 20 ms 控制任务访问）。 */
static uint16_t tx_seq;
static bool tracker_reset_pulse;
static uint8_t enemy_color;
static uint8_t bullet_speed;

static RmVisionLinkStats link_stats;

/** 目标帧缓冲必须常驻：DMA 发送期间源缓冲不可失效。 */
static uint8_t tx_frame[RM_VISION_TELEMETRY_FRAME_LEN];

/*============================================================================
 * 接收：由 5 ms 电机任务的 USARTDispatchPending() 驱动
 *============================================================================*/
static void Vision_RxCallback(void)
{
    RmVisionTarget decoded;
    RmCriticalState state;
    bool got;
    bool accepted = false;
    bool armed;
    bool finite;
    bool tracking;

    /* A failed daemon registration may leave an endpoint owned when DMA
     * cannot be aborted.  Keep that endpoint quiet instead of treating a
     * half-initialized link as a live telemetry channel. */
    if (vision_usart == NULL || vision_daemon == NULL ||
        vision_usart->recv_len == 0U) {
        return;
    }

    /* 回调期间 recv_buff 不会被 DMA 复用，可原地读取。 */
    got = RmVisionRx_Push(&rx_stream,
                          vision_usart->recv_buff,
                          vision_usart->recv_len,
                          &decoded);

    /* 统计无条件同步：即使本批无有效帧也要反映错误计数。 */
    state = RmCritical_Enter();
    link_stats.crc_errors = rx_stream.crc_errors;
    link_stats.format_errors = rx_stream.format_errors;
    link_stats.resync_bytes = rx_stream.resync_bytes;
    link_stats.rx_frames = rx_stream.accepted_frames;
    RmCritical_Exit(state);

    if (!got) {
        return;
    }

    /*
     * 任何 CRC/长度有效的目标帧都喂守护：链路在线与自瞄开关、TRACKING
     * 分离。这样手动模式或暂时没有目标时也能在诊断里看到视觉在线。
     */
    DaemonReload(vision_daemon);

    finite = RmVisionTarget_HasFiniteState(&decoded);
    tracking = (decoded.flags & RM_VISION_TARGET_FLAG_TRACKING) != 0U;

    /*
     * 自瞄开关与目标写入必须在同一个临界区内判定，否则会出现竞态：
     * 控制任务刚关闭自瞄、本回调却已通过检查并写入可用目标，
     * 结果自瞄关闭后云台仍被视觉接管。
     *
     * 安全关键：自瞄未开启时一律不接受目标（§7.2 硬约束）。
     */
    state = RmCritical_Enter();
    armed = link_state.auto_aim_enabled;
    RmVisionLinkState_ConsumeFrame(&link_state, finite, tracking);
    if (!armed) {
        link_stats.rejected_not_armed++;
    } else if (!finite) {
        /* 失效关闭：绝不把 NaN/Inf 传到云台。 */
        link_stats.rejected_nonfinite++;
    } else {
        latest_target = decoded;
        accepted = true;
    }

    if (accepted && RmVisionLinkState_HasTarget(&link_state)) {
        link_stats.target_frames++;
        link_stats.last_target_id = RmVisionTarget_GetTargetId(&decoded);
        link_stats.last_armors_num = RmVisionTarget_GetArmorsNum(&decoded);
    }
    RmCritical_Exit(state);
}

/**
 * @brief 守护离线回调：只清目标可用性，不停机。
 *
 * 与裁判/遥控的离线回调不同，此处**不**重启串口接收——视觉掉线是降级条件，
 * 云台退回手动即可，底盘与发射完全不受影响。
 */
static void Vision_LostCallback(void *owner)
{
    RmCriticalState state;

    (void)owner;

    state = RmCritical_Enter();
    RmVisionLinkState_OnPeerLost(&link_state);
    RmCritical_Exit(state);
}

/*============================================================================
 * 发送：由 20 ms 控制任务调用
 *============================================================================*/
void RmVisionLink_Update(uint32_t now_ms,
                         float yaw_deg,
                         float pitch_deg,
                         float roll_deg)
{
    RmVisionTelemetry telemetry;
    RmCriticalState state;
    bool auto_aim;

    (void)now_ms;

    if (vision_usart == NULL || vision_daemon == NULL) {
        return;
    }

    state = RmCritical_Enter();
    auto_aim = link_state.auto_aim_enabled;
    RmCritical_Exit(state);

    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.seq = tx_seq;
    telemetry.bullet_speed = bullet_speed;
    telemetry.yaw = yaw_deg;
    /* 内部 IMU 符号（抬头为负）转 ROS REP-103（抬头为正）。 */
    telemetry.pitch = RmVision_PitchImuToWire(pitch_deg);
    telemetry.roll = roll_deg;

    if ((enemy_color & 0x1U) != 0U) {
        telemetry.flags |= RM_VISION_TELEM_FLAG_ENEMY_COLOR;
    }
    if (auto_aim) {
        telemetry.flags |= RM_VISION_TELEM_FLAG_AUTO_AIM_ENABLE;
    }
    if (tracker_reset_pulse) {
        telemetry.flags |= RM_VISION_TELEM_FLAG_RESET_TRACKER;
    }

    /* INS_Read() reports snapshot availability, not numerical validity.  Do
     * not put a NaN/Inf attitude on the wire if the estimator has diverged;
     * dropping this telemetry frame is safer than publishing an invalid
     * pose to the upper computer. */
    if (!RmVisionTelemetry_HasFiniteAngles(&telemetry)) {
        state = RmCritical_Enter();
        link_stats.tx_dropped++;
        RmCritical_Exit(state);
        return;
    }

    if (RmVision_EncodeTelemetry(&telemetry, tx_frame, sizeof(tx_frame)) == 0U) {
        return;
    }

    /* 串口忙时丢弃本帧，绝不阻塞 20 ms 控制周期。 */
    if (USARTIsReady(vision_usart) == 0U ||
        USARTSend(vision_usart, tx_frame, RM_VISION_TELEMETRY_FRAME_LEN,
                  USART_TRANSFER_DMA) == 0U) {
        state = RmCritical_Enter();
        link_stats.tx_dropped++;
        RmCritical_Exit(state);
        return;
    }

    /* 只有真正入队的帧才消费一次性脉冲；串口忙或姿态非法时保留请求，
     * 避免上位机永远收不到 RESET_TRACKER。 */
    tracker_reset_pulse = false;
    tx_seq++;
    state = RmCritical_Enter();
    link_stats.tx_frames++;
    latest_telemetry = telemetry;
    telemetry_valid = true;
    RmCritical_Exit(state);
}

/*============================================================================
 * 初始化
 *============================================================================*/
bool RmVisionLink_Init(const RmVisionLinkConfig *config)
{
    USART_Init_Config_s usart_conf = {0};
    DaemonConfig daemon_conf = {0};

    if (config == NULL || config->uart == NULL) {
        return false;
    }
    if (vision_usart != NULL || vision_daemon != NULL) {
        return false;
    }

    /* CRC 表必须在任何收发之前构建，避免首次调用落在任务/中断上下文。 */
    RmVision_Init();
    RmVisionRx_Reset(&rx_stream);

    usart_conf.module_callback = Vision_RxCallback;
    usart_conf.usart_handle = config->uart;
    usart_conf.recv_buff_size = RM_VISION_TARGET_FRAME_LEN;

    vision_usart = USARTRegister(&usart_conf);
    if (vision_usart == NULL) {
        return false;
    }

    daemon_conf.timeout_ms = (config->timeout_ms != 0U)
                                 ? config->timeout_ms
                                 : RM_VISION_LINK_DEFAULT_TIMEOUT_MS;
    daemon_conf.initial_grace_ms = (config->initial_grace_ms != 0U)
                                       ? config->initial_grace_ms
                                       : RM_VISION_LINK_DEFAULT_GRACE_MS;
    daemon_conf.callback = Vision_LostCallback;
    daemon_conf.owner = NULL;

    vision_daemon = DaemonRegister(&daemon_conf);
    if (vision_daemon == NULL) {
        /* USARTRegister() starts DMA immediately; do not leak that endpoint
         * when the health registry is already full or rejects the config. */
        if (USARTUnregister(vision_usart)) {
            vision_usart = NULL;
        }
        return false;
    }

    memset(&link_stats, 0, sizeof(link_stats));
    memset(&latest_target, 0, sizeof(latest_target));
    memset(&latest_telemetry, 0, sizeof(latest_telemetry));
    RmVisionLinkState_Reset(&link_state);
    telemetry_valid = false;
    tx_seq = 0U;
    tracker_reset_pulse = false;
    enemy_color = 0U;
    bullet_speed = 0U;

    return true;
}

/*============================================================================
 * 设置接口
 *============================================================================*/
void RmVisionLink_SetAutoAim(bool enabled)
{
    RmCriticalState state;

    state = RmCritical_Enter();
    RmVisionLinkState_SetAutoAim(&link_state, enabled);
    RmCritical_Exit(state);
}

void RmVisionLink_SetEnemyColor(uint8_t color)
{
    enemy_color = (uint8_t)(color & 0x1U);
}

void RmVisionLink_SetBulletSpeed(uint8_t speed_mps)
{
    bullet_speed = speed_mps;
}

void RmVisionLink_RequestTrackerReset(void)
{
    tracker_reset_pulse = true;
}

/*============================================================================
 * 状态查询
 *============================================================================*/
bool RmVisionLink_IsOnline(void)
{
    RmCriticalState state;
    bool peer_online;

    if (vision_daemon == NULL) {
        return false;
    }

    state = RmCritical_Enter();
    peer_online = RmVisionLinkState_IsOnline(&link_state);
    RmCritical_Exit(state);

    /* 在线只回答“合法帧是否仍在到达”，不要求自瞄开启或 TRACKING。 */
    return peer_online && DaemonIsOnline(vision_daemon);
}

bool RmVisionLink_GetTelemetry(RmVisionTelemetry *out)
{
    RmCriticalState state;
    bool valid;

    if (out == NULL) {
        return false;
    }

    state = RmCritical_Enter();
    valid = telemetry_valid;
    if (valid) {
        *out = latest_telemetry;
    }
    RmCritical_Exit(state);

    return valid;
}

bool RmVisionLink_HasTarget(void)
{
    RmCriticalState state;
    bool usable;

    if (vision_daemon == NULL) {
        return false;
    }

    state = RmCritical_Enter();
    usable = RmVisionLinkState_HasTarget(&link_state);
    RmCritical_Exit(state);

    return usable && DaemonIsOnline(vision_daemon);
}

bool RmVisionLink_GetTarget(RmVisionTarget *out)
{
    RmCriticalState state;
    bool valid;

    if (out == NULL) {
        return false;
    }

    state = RmCritical_Enter();
    valid = RmVisionLinkState_HasValidTarget(&link_state);
    if (valid) {
        *out = latest_target;
    }
    RmCritical_Exit(state);

    return valid;
}

bool RmVisionLink_GetStats(RmVisionLinkStats *out)
{
    RmCriticalState state;

    if (out == NULL) {
        return false;
    }

    state = RmCritical_Enter();
    *out = link_stats;
    RmCritical_Exit(state);
    return true;
}
