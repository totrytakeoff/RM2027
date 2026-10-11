/**
 * @file minimal_debug.c
 * @brief 最小框架统一调试输出层实现
 */

#include "debug.h"

#include <stdarg.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "usart.h"
#include "robot_app.h"
#include "robot_types.h"
#include "input.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "ins_task.h"
#include "rm_time.h"
#include "utils.h"

static uint32_t last_text_tick = 0U;
static uint32_t last_vofa_tick = 0U;
static uint32_t vofa_tx_fail_cnt = 0U;
static uint32_t text_tx_fail_cnt = 0U;

static uint32_t MinimalDebug_GetTxTimeoutMs(uint16_t len)
{
    uint32_t baud = (MINIMAL_DEBUG_UART_BAUDRATE > 0U) ? MINIMAL_DEBUG_UART_BAUDRATE : 115200U;
    /* UART 8N1约10bit/byte，留一点冗余 */
    uint32_t need_ms = ((uint32_t)len * 10U * 1000U + baud - 1U) / baud + 2U;
    if (need_ms < MINIMAL_DEBUG_UART_TIMEOUT_MS) {
        need_ms = MINIMAL_DEBUG_UART_TIMEOUT_MS;
    }
    return need_ms;
}

static void __attribute__((unused)) MinimalDebug_TxRaw(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0U) {
        return;
    }

    /*
     * 文本诊断同时镜像到 RTT 和板载调试串口：RTT 供在线调试定位，
     * UART 供脱离调试器后的上机观察。RTT 上行采用非阻塞写，不能拖慢控制环。
     */
    (void)SEGGER_RTT_Write(0U, buf, (unsigned)len);
    if (HAL_UART_Transmit(&MINIMAL_DEBUG_UART_HANDLE, (uint8_t *)buf, len, MinimalDebug_GetTxTimeoutMs(len)) != HAL_OK) {
        text_tx_fail_cnt++;
    }
}

static void MinimalDebug_LogEventByTag(const char *tag, const char *fmt, va_list args)
{
#if MINIMAL_DEBUG_TEXT_STREAM_ACTIVE
    char msg_buf[224];
    char line_buf[256];
    int msg_n;
    int line_n;
    size_t msg_len;

    msg_n = RmFormat_Vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
    if (msg_n < 0) {
        return;
    }
    msg_len = strnlen(msg_buf, sizeof(msg_buf));
    while (msg_len > 0U && (msg_buf[msg_len - 1U] == '\r' || msg_buf[msg_len - 1U] == '\n')) {
        msg_buf[msg_len - 1U] = '\0';
        msg_len--;
    }
    line_n = RmFormat_Snprintf(line_buf,
                              sizeof(line_buf),
                              "[DBG][%s] %s\r\n",
                              tag,
                              msg_buf);
    if (line_n > 0) {
        uint16_t tx_len;
        if (line_n < (int)sizeof(line_buf)) {
            tx_len = (uint16_t)line_n;
        } else {
            /* 行被截断时强制补齐 CRLF，避免日志黏连 */
            size_t cap = sizeof(line_buf);
            line_buf[cap - 3U] = '\r';
            line_buf[cap - 2U] = '\n';
            line_buf[cap - 1U] = '\0';
            tx_len = (uint16_t)(cap - 1U);
        }
        MinimalDebug_TxRaw((const uint8_t *)line_buf, tx_len);
    }
#else
    (void)tag;
    (void)fmt;
    (void)args;
#endif
}

static void __attribute__((unused)) MinimalDebug_TextPeriodic(void)
{
#if MINIMAL_DEBUG_TEXT_STREAM_ACTIVE
#if MINIMAL_DEBUG_MOD_SYSTEM || MINIMAL_DEBUG_MOD_INPUT
    RemoteControlState remote = {0};

    (void)Input_GetRemoteState(&remote);
#endif

#if MINIMAL_DEBUG_MOD_SYSTEM
    MinimalDebug_LogEventSystem(
        "safety=%u reason=0x%08lx init=%u health(c/g/s/ins)=%u/%u/%u/%u remote=%u online=%u gate=%u safe=%u edge=%u",
        (unsigned)RobotApp_GetSafetyState(),
        (unsigned long)RobotApp_GetSafetyReasons(),
        (unsigned)g_robot.initialized,
        Chassis_IsHealthy() ? 1U : 0U,
        Gimbal_IsHealthy() ? 1U : 0U,
        Shoot_IsHealthy() ? 1U : 0U,
        INS_IsReady() ? 1U : 0U,
        (unsigned)remote.type,
        (unsigned)g_robot.input.online,
        (unsigned)g_robot.input.operator_enable_request,
        (unsigned)g_robot.input.operator_safe_position,
        (unsigned)g_robot.input.operator_arm_event);
#endif

#if MINIMAL_DEBUG_MOD_INPUT
    MinimalDebug_LogEventInput(
        "remote=%u link=%u valid=%u fs=%u seq=%lu sm=0x%02lx mode=%u fire=%u gate=%u safe=%u edge=%u estop=%u sw=(%u,%u,%u,%u) axis=(%d,%d,%d,%d) intent(x=%ld y=%ld r=%ld yaw=%ld pitch=%ld)",
        (unsigned)remote.type,
        (unsigned)remote.link_online,
        (unsigned)remote.data_valid,
        (unsigned)remote.failsafe,
        (unsigned long)remote.sample_sequence,
        (unsigned long)remote.switch_valid_mask,
        (unsigned)g_robot.input.control_mode,
        (unsigned)g_robot.input.fire_mode,
        (unsigned)g_robot.input.operator_enable_request,
        (unsigned)g_robot.input.operator_safe_position,
        (unsigned)g_robot.input.operator_arm_event,
        (unsigned)g_robot.input.emergency_stop,
        (unsigned)remote.switches[0],
        (unsigned)remote.switches[1],
        (unsigned)remote.switches[2],
        (unsigned)remote.switches[3],
        (int)remote.axis_raw[REMOTE_AXIS_LEFT_X],
        (int)remote.axis_raw[REMOTE_AXIS_LEFT_Y],
        (int)remote.axis_raw[REMOTE_AXIS_RIGHT_X],
        (int)remote.axis_raw[REMOTE_AXIS_RIGHT_Y],
        (long)(g_robot.input.chassis_x_intent * 1000.0f),
        (long)(g_robot.input.chassis_y_intent * 1000.0f),
        (long)(g_robot.input.chassis_rotate_intent * 1000.0f),
        (long)(g_robot.input.gimbal_yaw_intent * 1000.0f),
        (long)(g_robot.input.gimbal_pitch_intent * 1000.0f));
#endif

#if MINIMAL_DEBUG_MOD_CHASSIS
    MinimalDebug_LogEventChassis("vx_mms=%ld vy_mms=%ld wz_mrads=%ld pwr_x1e3=%ld fr_ref_mrad_s=%ld fr_fdb_mrad_s=%ld",
                                 (long)(g_robot.chassis.vx * 1000.0f),
                                 (long)(g_robot.chassis.vy * 1000.0f),
                                 (long)(g_robot.chassis.wz * 1000.0f),
                                 (long)(Chassis_GetPowerScale() * 1000.0f),
                                 (long)(Chassis_GetFRMotorSpeedRefRadS() * 1000.0f),
                                 (long)(Chassis_GetFRMotorSpeedFdbRadS() * 1000.0f));
#endif

#if MINIMAL_DEBUG_MOD_GIMBAL
    MinimalDebug_LogEventGimbal("mode=%u pitch_loop=%u yaw_ref=%ld yaw_fdb=%ld yaw_off=%ld pitch_ref=%ld pitch_fdb=%ld pitch_tgt=%ld enc(y=%ld p=%ld) imu(y=%ld p=%ld) ff=%ld",
                                (unsigned)Gimbal_GetMode(),
                                (unsigned)Gimbal_GetPitchCtrlMode(),
                                (long)(Gimbal_GetYawSpeedRef()),
                                (long)(Gimbal_GetYawSpeedFdb()),
                                (long)(Gimbal_GetYawOffsetLogicDeg()),
                                (long)(Gimbal_GetPitchSpeedRef()),
                                (long)(Gimbal_GetPitchSpeedFdb()),
                                (long)(Gimbal_GetPitchTargetAngle()),
                                (long)(Gimbal_GetYawEncoderAngle()),
                                (long)(Gimbal_GetPitchEncoderAngle()),
                                (long)(Gimbal_GetYawIMUAngle()),
                                (long)(Gimbal_GetPitchIMUAngle()),
                                (long)(Gimbal_GetPitchGravityFeedforward()));
#endif

#if MINIMAL_DEBUG_MOD_SHOOT
    MinimalDebug_LogEventShoot("state=%u loader_ref=%ld loader_fdb=%ld pend=%u act=%u",
                               (unsigned)Shoot_GetState(),
                               (long)(Shoot_GetLoaderRef()),
                               (long)(Shoot_GetLoaderFeedback()),
                               (unsigned)Shoot_GetPendingShots(),
                               (unsigned)Shoot_IsSingleActive());
#endif
#endif
}

void MinimalDebug_Init(void)
{
#if MINIMAL_DEBUG_ENABLE
#if (MINIMAL_DEBUG_UART_PORT == 1U) || (MINIMAL_DEBUG_UART_PORT == 6U)
    MINIMAL_DEBUG_UART_HANDLE.Init.BaudRate = MINIMAL_DEBUG_UART_BAUDRATE;
    MINIMAL_DEBUG_UART_HANDLE.Init.WordLength = UART_WORDLENGTH_8B;
    MINIMAL_DEBUG_UART_HANDLE.Init.StopBits = UART_STOPBITS_1;
    MINIMAL_DEBUG_UART_HANDLE.Init.Parity = UART_PARITY_NONE;
    MINIMAL_DEBUG_UART_HANDLE.Init.Mode = UART_MODE_TX_RX;
    MINIMAL_DEBUG_UART_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    MINIMAL_DEBUG_UART_HANDLE.Init.OverSampling = UART_OVERSAMPLING_16;
    (void)HAL_UART_Init(&MINIMAL_DEBUG_UART_HANDLE);
#endif

#if MINIMAL_DEBUG_TEXT_STREAM_ACTIVE && MINIMAL_DEBUG_MOD_SYSTEM
    MinimalDebug_LogEventSystem("debug mode=0x%02x uart=%u baud=%lu",
                                MINIMAL_DEBUG_MODE,
                                MINIMAL_DEBUG_UART_PORT,
                                (unsigned long)MINIMAL_DEBUG_UART_BAUDRATE);
#endif
#endif
}

void MinimalDebug_UpdatePeriodic(uint32_t now_ms)
{
#if MINIMAL_DEBUG_ENABLE
#if MINIMAL_DEBUG_TEXT_STREAM_ACTIVE
    if ((now_ms - last_text_tick) >= MINIMAL_DEBUG_TEXT_PERIOD_MS) {
        last_text_tick = now_ms;
        MinimalDebug_TextPeriodic();
    }
#endif

#if MINIMAL_DEBUG_VOFA_STREAM_ACTIVE
    if ((now_ms - last_vofa_tick) >= MINIMAL_DEBUG_VOFA_PERIOD_MS) {
        last_vofa_tick = now_ms;
        MinimalDebug_PublishVofaFrame();
    }
#endif
#else
    (void)now_ms;
#endif
}

void MinimalDebug_LogEventSystem(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    MinimalDebug_LogEventByTag("SYS", fmt, args);
    va_end(args);
}

void MinimalDebug_LogEventInput(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    MinimalDebug_LogEventByTag("IN", fmt, args);
    va_end(args);
}

void MinimalDebug_LogEventChassis(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    MinimalDebug_LogEventByTag("CHS", fmt, args);
    va_end(args);
}

void MinimalDebug_LogEventGimbal(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    MinimalDebug_LogEventByTag("GMB", fmt, args);
    va_end(args);
}

void MinimalDebug_LogEventShoot(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    MinimalDebug_LogEventByTag("SHT", fmt, args);
    va_end(args);
}

void MinimalDebug_PublishVofaFrame(void)
{
#if MINIMAL_DEBUG_VOFA_STREAM_ACTIVE
    float ch[16];
    uint8_t txbuf[(sizeof(ch)) + 4U];
    static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    HAL_StatusTypeDef ret;

    ch[0] = (float)RmTime_NowMs();
    ch[1] = (float)Input_GetRemoteType();
    ch[2] = (float)g_robot.input.online;
    ch[3] = g_robot.chassis.vx;
    ch[4] = g_robot.chassis.vy;
    ch[5] = g_robot.chassis.wz;
    ch[6] = Chassis_GetFRMotorSpeedRefRadS();
    ch[7] = Chassis_GetFRMotorSpeedFdbRadS();
    ch[8] = Gimbal_GetYawSpeedRef();
    ch[9] = Gimbal_GetYawSpeedFdb();
    ch[10] = Gimbal_GetPitchSpeedRef();
    ch[11] = Gimbal_GetPitchSpeedFdb();
    ch[12] = Gimbal_GetYawOffsetLogicDeg();
    ch[13] = Gimbal_GetPitchTargetAngle();
    ch[14] = Shoot_GetLoaderRef();
    ch[15] = Shoot_GetLoaderFeedback();

    memcpy(txbuf, ch, sizeof(ch));
    memcpy(txbuf + sizeof(ch), tail, sizeof(tail));

    ret = HAL_UART_Transmit(&MINIMAL_DEBUG_UART_HANDLE,
                            txbuf,
                            (uint16_t)sizeof(txbuf),
                            MinimalDebug_GetTxTimeoutMs((uint16_t)sizeof(txbuf)));
    if (ret != HAL_OK) {
        vofa_tx_fail_cnt++;
#if MINIMAL_DEBUG_TEXT_STREAM_ACTIVE && MINIMAL_DEBUG_MOD_SYSTEM
        if ((vofa_tx_fail_cnt % 50U) == 1U) {
            MinimalDebug_LogEventSystem("vofa_tx_fail=%lu text_tx_fail=%lu",
                                        (unsigned long)vofa_tx_fail_cnt,
                                        (unsigned long)text_tx_fail_cnt);
        }
#endif
    }
#endif
}
