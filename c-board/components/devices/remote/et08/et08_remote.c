#include "et08_remote.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "rm_critical.h"
#include "rm_time.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"

#define ET08_SBUS_FRAME_SIZE 25u
#define ET08_SBUS_START_BYTE 0x0F

static ET08_Ctrl_t et08_ctrl;
static uint8_t et08_init_flag = 0;

static USARTInstance *et08_usart_instance;
static DaemonInstance *et08_daemon;

static uint32_t et08_frame_count = 0u;
static uint32_t et08_bad_count = 0u;
static uint32_t et08_last_frame_ms = 0u;
static uint32_t et08_max_frame_gap_ms = 0u;
static uint32_t et08_lost_count = 0u;
static uint32_t et08_frame_lost_count = 0u;
static uint32_t et08_failsafe_count = 0u;

static void ET08_PublishCtrl(const ET08_Ctrl_t *ctrl)
{
    RmCriticalState state;

    if (ctrl == NULL) {
        return;
    }

    state = RmCritical_Enter();
    memcpy(&et08_ctrl, ctrl, sizeof(et08_ctrl));
    RmCritical_Exit(state);
}

static void ET08_ParseSbusChannels(const uint8_t *buf, uint16_t *ch)
{
    ch[0] = (uint16_t)((buf[1] | (buf[2] << 8)) & 0x07FF);
    ch[1] = (uint16_t)(((buf[2] >> 3) | (buf[3] << 5)) & 0x07FF);
    ch[2] = (uint16_t)(((buf[3] >> 6) | (buf[4] << 2) | (buf[5] << 10)) & 0x07FF);
    ch[3] = (uint16_t)(((buf[5] >> 1) | (buf[6] << 7)) & 0x07FF);
    ch[4] = (uint16_t)(((buf[6] >> 4) | (buf[7] << 4)) & 0x07FF);
    ch[5] = (uint16_t)(((buf[7] >> 7) | (buf[8] << 1) | (buf[9] << 9)) & 0x07FF);
    ch[6] = (uint16_t)(((buf[9] >> 2) | (buf[10] << 6)) & 0x07FF);
    ch[7] = (uint16_t)(((buf[10] >> 5) | (buf[11] << 3)) & 0x07FF);
    ch[8] = (uint16_t)(((buf[12] | (buf[13] << 8))) & 0x07FF);
    ch[9] = (uint16_t)(((buf[13] >> 3) | (buf[14] << 5)) & 0x07FF);
    ch[10] = (uint16_t)(((buf[14] >> 6) | (buf[15] << 2) | (buf[16] << 10)) & 0x07FF);
    ch[11] = (uint16_t)(((buf[16] >> 1) | (buf[17] << 7)) & 0x07FF);
    ch[12] = (uint16_t)(((buf[17] >> 4) | (buf[18] << 4)) & 0x07FF);
    ch[13] = (uint16_t)(((buf[18] >> 7) | (buf[19] << 1) | (buf[20] << 9)) & 0x07FF);
    ch[14] = (uint16_t)(((buf[20] >> 2) | (buf[21] << 6)) & 0x07FF);
    ch[15] = (uint16_t)(((buf[21] >> 5) | (buf[22] << 3)) & 0x07FF);
}

uint8_t ET08_MapSwitchState(uint16_t raw_value)
{
    const uint16_t levels[ET08_SWITCH_LEVEL_COUNT] = {
        ET08_SWITCH_LEVEL_0,
        ET08_SWITCH_LEVEL_1,
        ET08_SWITCH_LEVEL_2,
        ET08_SWITCH_LEVEL_3,
        ET08_SWITCH_LEVEL_4,
        ET08_SWITCH_LEVEL_5,
    };

    uint8_t best_index = 0xFF;
    uint16_t best_diff = 0xFFFF;

    for (uint8_t i = 0; i < ET08_SWITCH_LEVEL_COUNT; ++i)
    {
        uint16_t diff = (raw_value > levels[i]) ? (raw_value - levels[i]) : (levels[i] - raw_value);
        if (diff < best_diff)
        {
            best_diff = diff;
            best_index = i;
        }
    }

    if (best_diff <= ET08_SWITCH_TOLERANCE)
        return best_index;
    return 0xFF;
}

static void ET08_FillCtrl(const uint16_t *ch, uint8_t flags, ET08_Ctrl_t *ctrl)
{
    memset(ctrl, 0, sizeof(*ctrl));

    for (uint8_t i = 0; i < ET08_CHANNEL_COUNT_FULL; ++i) {
        ctrl->raw_full[i] = ch[i];
        ctrl->centered_full[i] = (int16_t)ch[i] - ET08_CHANNEL_CENTER;
        if (i < ET08_CHANNEL_COUNT) {
            ctrl->raw[i] = ch[i];
            ctrl->centered[i] = (int16_t)ch[i] - ET08_CHANNEL_CENTER;
        }
    }

    ctrl->right.x = ctrl->centered[ET08_MAP_RIGHT_X_CH];
    ctrl->right.y = ctrl->centered[ET08_MAP_RIGHT_Y_CH];
    ctrl->left.y = ctrl->centered[ET08_MAP_LEFT_Y_CH];
    ctrl->left.x = ctrl->centered[ET08_MAP_LEFT_X_CH];

    ctrl->switch_sa_sb_raw = ctrl->raw[ET08_MAP_SA_SB_CH];
    ctrl->switch_sa_sb_centered = ctrl->centered[ET08_MAP_SA_SB_CH];
    ctrl->switch_sa_sb_state = ET08_MapSwitchState(ctrl->switch_sa_sb_raw);

    ctrl->switch_sd_sc_raw = ctrl->raw[ET08_MAP_SD_SC_CH];
    ctrl->switch_sd_sc_centered = ctrl->centered[ET08_MAP_SD_SC_CH];
    ctrl->switch_sd_sc_state = ET08_MapSwitchState(ctrl->switch_sd_sc_raw);

    ctrl->knob_left = ctrl->centered[ET08_MAP_KNOB_LEFT_CH];
    ctrl->knob_right = ctrl->centered[ET08_MAP_KNOB_RIGHT_CH];

    ctrl->frame_lost = (flags & 0x04u) ? 1u : 0u;
    ctrl->failsafe = (flags & 0x08u) ? 1u : 0u;
}

static void ET08_RxCallback(void)
{
    uint32_t now_ms;

    if (et08_usart_instance == NULL)
        return;

    if (et08_usart_instance->recv_len != ET08_SBUS_FRAME_SIZE)
    {
        et08_bad_count++;
        return;
    }

    const uint8_t *buf = et08_usart_instance->recv_buff;
    if (buf[0] != ET08_SBUS_START_BYTE)
    {
        et08_bad_count++;
        return;
    }

    uint16_t ch[16] = {0};
    ET08_ParseSbusChannels(buf, ch);

    uint8_t flags = buf[23];
    ET08_Ctrl_t next;
    ET08_FillCtrl(ch, flags, &next);
    et08_frame_count++;

    /*
     * SBUS frame_lost 表示接收机丢失了单个射频帧，不等价于链路掉线。
     * 该帧的通道值不可采用，但也不能让安全状态在相邻好帧之间反复急停。
     * 保留上一份有效快照，并且只用有效帧续期守护；连续没有有效帧时仍会
     * 在 ET08_ONLINE_TIMEOUT_MS 内进入掉线保护。
     */
    if (next.failsafe != 0U) {
        et08_failsafe_count++;
        ET08_PublishCtrl(&next);
        return;
    }
    if (next.frame_lost != 0U) {
        et08_frame_lost_count++;
        return;
    }

    now_ms = RmTime_NowMs();
    if (et08_last_frame_ms != 0U) {
        uint32_t gap_ms = RmTime_ElapsedMs(now_ms, et08_last_frame_ms);
        if (gap_ms > et08_max_frame_gap_ms) {
            et08_max_frame_gap_ms = gap_ms;
        }
    }
    et08_last_frame_ms = now_ms;
    ET08_PublishCtrl(&next);
    DaemonReload(et08_daemon);
}

static void ET08_LostCallback(void *id)
{
    const ET08_Ctrl_t offline = {0};
    const uint32_t now_ms = RmTime_NowMs();
    const uint32_t age_ms = (et08_frame_count != 0U)
                                ? RmTime_ElapsedMs(now_ms,
                                                   et08_last_frame_ms)
                                : 0U;

    (void)id;
    ET08_PublishCtrl(&offline);
    et08_lost_count++;
    LOGWARNING("[et08] remote lost count=%lu age=%lums max_good_gap=%lums frames=%lu frame_lost=%lu failsafe=%lu bad=%lu",
               (unsigned long)et08_lost_count,
               (unsigned long)age_ms,
               (unsigned long)et08_max_frame_gap_ms,
               (unsigned long)et08_frame_count,
               (unsigned long)et08_frame_lost_count,
               (unsigned long)et08_failsafe_count,
               (unsigned long)et08_bad_count);

    if (et08_usart_instance)
        USARTServiceInit(et08_usart_instance);
}

static bool ET08_ReinitUartForSbus(UART_HandleTypeDef *uart_handle)
{
    if (uart_handle == NULL)
        return false;

    (void)HAL_UART_DeInit(uart_handle);
    uart_handle->Init.BaudRate = 100000;
    uart_handle->Init.WordLength = UART_WORDLENGTH_9B;
    uart_handle->Init.StopBits = UART_STOPBITS_2;
    uart_handle->Init.Parity = UART_PARITY_EVEN;
    uart_handle->Init.Mode = UART_MODE_TX_RX;
    uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_handle->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(uart_handle) != HAL_OK)
    {
        LOGWARNING("[et08] sbus uart reinit failed");
        return false;
    }
    return true;
}

ET08_Ctrl_t *ET08_Init(UART_HandleTypeDef *uart_handle)
{
    return ET08_InitWithTimeout(uart_handle, ET08_DEFAULT_TIMEOUT_MS);
}

ET08_Ctrl_t *ET08_InitWithTimeout(UART_HandleTypeDef *uart_handle,
                                  uint32_t timeout_ms)
{
    const ET08_Ctrl_t empty = {0};

    ET08_PublishCtrl(&empty);
    et08_init_flag = 0U;

    if (!ET08_ReinitUartForSbus(uart_handle)) {
        return NULL;
    }

    USART_Init_Config_s conf = {0};
    conf.module_callback = ET08_RxCallback;
    conf.usart_handle = uart_handle;
    conf.recv_buff_size = ET08_SBUS_FRAME_SIZE;
    et08_usart_instance = USARTRegister(&conf);
    if (et08_usart_instance == NULL) {
        return NULL;
    }

    DaemonConfig daemon_conf = {
        .timeout_ms = (timeout_ms != 0U) ? timeout_ms
                                         : ET08_DEFAULT_TIMEOUT_MS,
        .callback = ET08_LostCallback,
        .owner = NULL,
    };
    et08_daemon = DaemonRegister(&daemon_conf);
    if (et08_daemon == NULL) {
        return NULL;
    }

    et08_init_flag = 1;
    et08_frame_count = 0u;
    et08_bad_count = 0u;
    et08_last_frame_ms = 0u;
    et08_max_frame_gap_ms = 0u;
    et08_lost_count = 0u;
    et08_frame_lost_count = 0u;
    et08_failsafe_count = 0u;

    return &et08_ctrl;
}

uint8_t ET08_IsOnline(void)
{
    if (!et08_init_flag)
        return 0u;
    return DaemonIsOnline(et08_daemon);
}

bool ET08_Read(ET08_Ctrl_t *snapshot)
{
    RmCriticalState state;

    if (snapshot == NULL) {
        return false;
    }
    if (!ET08_IsOnline()) {
        memset(snapshot, 0, sizeof(*snapshot));
        return false;
    }

    state = RmCritical_Enter();
    memcpy(snapshot, &et08_ctrl, sizeof(*snapshot));
    RmCritical_Exit(state);

    if (!ET08_IsOnline()) {
        memset(snapshot, 0, sizeof(*snapshot));
        return false;
    }
    return true;
}

ET08_Ctrl_t *ET08_GetCtrl(void)
{
    return &et08_ctrl;
}
