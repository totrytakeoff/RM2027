#include "dt7_remote.h"

#include <stdlib.h>
#include <string.h>

#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "rm_critical.h"

#define REMOTE_CONTROL_FRAME_SIZE 18U

static RC_ctrl_t rc_ctrl[2];
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon;
static uint8_t rc_initialized;

static void RemoteControlPublish(const RC_ctrl_t state[2])
{
    RmCriticalState critical_state;

    critical_state = RmCritical_Enter();
    memcpy(rc_ctrl, state, sizeof(rc_ctrl));
    RmCritical_Exit(critical_state);
}

static bool RemoteControlSwitchValid(uint8_t value)
{
    return value == RC_SW_UP || value == RC_SW_MID || value == RC_SW_DOWN;
}

static bool RemoteControlParse(const uint8_t *buffer, RC_ctrl_t state[2])
{
    RC_ctrl_t *current = &state[TEMP];
    const RC_ctrl_t *previous = &state[LAST];
    uint16_t key_now;
    uint16_t key_last;
    uint16_t key_with_ctrl;
    uint16_t key_with_shift;
    uint16_t key_last_with_ctrl;
    uint16_t key_last_with_shift;

    current->rc.rocker_r_ =
        (int16_t)(((buffer[0] | (buffer[1] << 8)) & 0x07FFU) -
                  RC_CH_VALUE_OFFSET);
    current->rc.rocker_r1 =
        (int16_t)((((buffer[1] >> 3) | (buffer[2] << 5)) & 0x07FFU) -
                  RC_CH_VALUE_OFFSET);
    current->rc.rocker_l_ =
        (int16_t)((((buffer[2] >> 6) | (buffer[3] << 2) |
                    (buffer[4] << 10)) &
                   0x07FFU) -
                  RC_CH_VALUE_OFFSET);
    current->rc.rocker_l1 =
        (int16_t)((((buffer[4] >> 1) | (buffer[5] << 7)) & 0x07FFU) -
                  RC_CH_VALUE_OFFSET);
    current->rc.dial =
        (int16_t)(((buffer[16] | (buffer[17] << 8)) & 0x07FFU) -
                  RC_CH_VALUE_OFFSET);
    current->rc.switch_right = (uint8_t)((buffer[5] >> 4) & 0x03U);
    current->rc.switch_left = (uint8_t)(((buffer[5] >> 4) & 0x0CU) >> 2);

    if (abs(current->rc.rocker_l_) > 660 ||
        abs(current->rc.rocker_l1) > 660 ||
        abs(current->rc.rocker_r_) > 660 ||
        abs(current->rc.rocker_r1) > 660 ||
        abs(current->rc.dial) > 660 ||
        !RemoteControlSwitchValid(current->rc.switch_left) ||
        !RemoteControlSwitchValid(current->rc.switch_right)) {
        return false;
    }

    current->mouse.x = (int16_t)(buffer[6] | (buffer[7] << 8));
    current->mouse.y = (int16_t)(buffer[8] | (buffer[9] << 8));
    current->mouse.z = (int16_t)(buffer[10] | (buffer[11] << 8));
    current->mouse.press_l = buffer[12] != 0U ? 1U : 0U;
    current->mouse.press_r = buffer[13] != 0U ? 1U : 0U;
    current->key[KEY_PRESS].keys =
        (uint16_t)(buffer[14] | (buffer[15] << 8));

    current->key[KEY_PRESS_WITH_CTRL].keys =
        current->key[KEY_PRESS].ctrl ? current->key[KEY_PRESS].keys : 0U;
    current->key[KEY_PRESS_WITH_SHIFT].keys =
        current->key[KEY_PRESS].shift ? current->key[KEY_PRESS].keys : 0U;

    key_now = current->key[KEY_PRESS].keys;
    key_last = previous->key[KEY_PRESS].keys;
    key_with_ctrl = current->key[KEY_PRESS_WITH_CTRL].keys;
    key_with_shift = current->key[KEY_PRESS_WITH_SHIFT].keys;
    key_last_with_ctrl = previous->key[KEY_PRESS_WITH_CTRL].keys;
    key_last_with_shift = previous->key[KEY_PRESS_WITH_SHIFT].keys;

    for (uint16_t index = 0U, bit = 1U; index < 16U;
         ++index, bit <<= 1U) {
        if (index == Key_Shift || index == Key_Ctrl) {
            continue;
        }
        if ((key_now & bit) != 0U && (key_last & bit) == 0U &&
            (key_with_ctrl & bit) == 0U &&
            (key_with_shift & bit) == 0U) {
            current->key_count[KEY_PRESS][index]++;
        }
        if ((key_with_ctrl & bit) != 0U &&
            (key_last_with_ctrl & bit) == 0U) {
            current->key_count[KEY_PRESS_WITH_CTRL][index]++;
        }
        if ((key_with_shift & bit) != 0U &&
            (key_last_with_shift & bit) == 0U) {
            current->key_count[KEY_PRESS_WITH_SHIFT][index]++;
        }
    }

    state[LAST] = state[TEMP];
    return true;
}

static bool RemoteControlConfigureUart(UART_HandleTypeDef *uart_handle)
{
    if (uart_handle == NULL) {
        return false;
    }

    (void)HAL_UART_DeInit(uart_handle);
    uart_handle->Init.BaudRate = 100000U;
    uart_handle->Init.WordLength = UART_WORDLENGTH_9B;
    uart_handle->Init.StopBits = UART_STOPBITS_1;
    uart_handle->Init.Parity = UART_PARITY_EVEN;
    uart_handle->Init.Mode = UART_MODE_TX_RX;
    uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_handle->Init.OverSampling = UART_OVERSAMPLING_16;
    return HAL_UART_Init(uart_handle) == HAL_OK;
}

static void RemoteControlRxCallback(void)
{
    RC_ctrl_t next[2];
    RmCriticalState critical_state;

    if (rc_usart_instance == NULL ||
        rc_usart_instance->recv_len != REMOTE_CONTROL_FRAME_SIZE) {
        return;
    }

    critical_state = RmCritical_Enter();
    memcpy(next, rc_ctrl, sizeof(next));
    RmCritical_Exit(critical_state);

    if (!RemoteControlParse(rc_usart_instance->recv_buff, next)) {
        return;
    }

    RemoteControlPublish(next);
    DaemonReload(rc_daemon);
}

static void RemoteControlLostCallback(void *owner)
{
    const RC_ctrl_t offline[2] = {0};

    (void)owner;
    RemoteControlPublish(offline);
    if (rc_usart_instance != NULL) {
        USARTServiceInit(rc_usart_instance);
    }
    LOGWARNING("[dt7] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    return RemoteControlInitWithTimeout(rc_usart_handle,
                                        DT7_DEFAULT_TIMEOUT_MS);
}

RC_ctrl_t *RemoteControlInitWithTimeout(UART_HandleTypeDef *rc_usart_handle,
                                       uint32_t timeout_ms)
{
    const RC_ctrl_t empty[2] = {0};
    USART_Init_Config_s uart_config = {0};
    DaemonConfig daemon_config = {0};

    rc_initialized = 0U;
    RemoteControlPublish(empty);
    if (!RemoteControlConfigureUart(rc_usart_handle)) {
        return NULL;
    }

    uart_config.module_callback = RemoteControlRxCallback;
    uart_config.usart_handle = rc_usart_handle;
    uart_config.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&uart_config);
    if (rc_usart_instance == NULL) {
        return NULL;
    }

    daemon_config.timeout_ms =
        timeout_ms != 0U ? timeout_ms : DT7_DEFAULT_TIMEOUT_MS;
    daemon_config.callback = RemoteControlLostCallback;
    rc_daemon = DaemonRegister(&daemon_config);
    if (rc_daemon == NULL) {
        return NULL;
    }

    rc_initialized = 1U;
    return rc_ctrl;
}

bool RemoteControlRead(RC_ctrl_t snapshot[2])
{
    RmCriticalState critical_state;

    if (snapshot == NULL || !RemoteControlIsOnline()) {
        if (snapshot != NULL) {
            memset(snapshot, 0, sizeof(rc_ctrl));
        }
        return false;
    }

    critical_state = RmCritical_Enter();
    memcpy(snapshot, rc_ctrl, sizeof(rc_ctrl));
    RmCritical_Exit(critical_state);

    if (!RemoteControlIsOnline()) {
        memset(snapshot, 0, sizeof(rc_ctrl));
        return false;
    }
    return true;
}

uint8_t RemoteControlIsOnline(void)
{
    return rc_initialized != 0U && DaemonIsOnline(rc_daemon) ? 1U : 0U;
}

uint8_t RemoteControlIsDt7Online(void)
{
    return RemoteControlIsOnline();
}
