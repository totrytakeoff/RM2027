/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Remote control telemetry demo
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025.
 * All rights reserved.
 *
 * 本测试固件依赖 RM2026 框架的 remote_control 模块。程序会初始化 USART3
 * 作为 DT7/DR16 的 D-BUS 输入，并把实时解析结果通过 USART6 以文本形式转发，
 * 用于抓取/调试遥控数据。
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "cmsis_os.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bsp_init.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dt7_remote.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TELEMETRY_TX_INTERVAL_MS 50U
#define TELEMETRY_USART_RX_DUMMY 32U
#define HEARTBEAT_INTERVAL_MS 1000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static RC_ctrl_t *rc_data = NULL;
static USARTInstance *telemetry_usart = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Debug_DisableWatchdogs(void);
static void InitTelemetryUsart(void);
static void DumpRemoteState(void);
static void SendHeartbeat(uint32_t seq);
static void TelemetrySendBuffer(const uint8_t *buffer, uint16_t len);
static void TelemetrySendString(const char *str);
static void AppendFormat(char *buffer, size_t buffer_size, size_t *offset, const char *fmt, ...);
static void AppendKeyList(const Key_t *key_state, char *buffer, size_t buffer_size, size_t *offset);
static void AppendKeyCounts(const char *label, const uint8_t counts[16], char *buffer, size_t buffer_size, size_t *offset);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const char *kKeyNames[16] = {
    "W",   "S",   "D",   "A",
    "Shift", "Ctrl", "Q",  "E",
    "R",   "F",   "G",   "Z",
    "X",   "C",   "V",   "B"};

/**
 * @brief 在调试模式下冻结 IWDG/WWDG，避免单步调试时复位
 */
static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

/**
 * @brief 注册 USART6 作为调试转发口，仅使用发送功能
 */
static void InitTelemetryUsart(void)
{
    USART_Init_Config_s config = {
        .module_callback = NULL,
        .recv_buff_size = TELEMETRY_USART_RX_DUMMY,
        .usart_handle = &huart6,
    };
    telemetry_usart = USARTRegister(&config);
    TelemetrySendString("[rc_demo] USART6 telemetry ready\r\n");
}

static void TelemetrySendBuffer(const uint8_t *buffer, uint16_t len)
{
    if (telemetry_usart == NULL || buffer == NULL || len == 0U)
        return;

    USARTSend(telemetry_usart, (uint8_t *)buffer, len, USART_TRANSFER_BLOCKING);
}

static void TelemetrySendString(const char *str)
{
    if (str == NULL)
        return;
    TelemetrySendBuffer((const uint8_t *)str, (uint16_t)strlen(str));
}

/**
 * @brief 追加格式化字符串到 buffer，内部负责更新 offset
 */
static void AppendFormat(char *buffer, size_t buffer_size, size_t *offset, const char *fmt, ...)
{
    if (*offset >= buffer_size)
        return;

    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(buffer + *offset, buffer_size - *offset, fmt, args);
    va_end(args);

    if (written < 0)
        return;

    size_t written_sz = (size_t)written;
    if (written_sz >= buffer_size - *offset)
    {
        *offset = buffer_size - 1U;
        buffer[*offset] = '\0';
    }
    else
    {
        *offset += written_sz;
    }
}

/**
 * @brief 将当前按键状态转换成人类可读的名称列表
 */
static void AppendKeyList(const Key_t *key_state, char *buffer, size_t buffer_size, size_t *offset)
{
    AppendFormat(buffer, buffer_size, offset, "KeysDown: ");

    bool any_pressed = false;
    for (uint32_t i = 0; i < 16U; ++i)
    {
        if (key_state->keys & (1U << i))
        {
            AppendFormat(buffer, buffer_size, offset, "%s ", kKeyNames[i]);
            any_pressed = true;
        }
    }

    if (!any_pressed)
    {
        AppendFormat(buffer, buffer_size, offset, "None");
    }
    AppendFormat(buffer, buffer_size, offset, "\r\n");
}

/**
 * @brief 打印 key_count 的 16 个计数值
 */
static void AppendKeyCounts(const char *label, const uint8_t counts[16], char *buffer, size_t buffer_size, size_t *offset)
{
    AppendFormat(buffer, buffer_size, offset, "%s: [", label);
    for (uint32_t i = 0; i < 16U; ++i)
    {
        AppendFormat(buffer, buffer_size, offset, "%u%s", counts[i], (i < 15U) ? ", " : "");
    }
    AppendFormat(buffer, buffer_size, offset, "]\r\n");
}

/**
 * @brief 将 dt7_remote.c 中解析出的所有字段通过 USART6 打印出来
 */
static void DumpRemoteState(void)
{
    if (telemetry_usart == NULL || rc_data == NULL)
        return;

    const RC_ctrl_t *cur = &rc_data[TEMP];

    char buffer[512];
    size_t offset = 0U;

    AppendFormat(buffer, sizeof(buffer), &offset,
                 "[rc_demo] RC R(%4d,%4d) L(%4d,%4d) Dial:%4d S1:%u S2:%u\r\n",
                 cur->rc.rocker_r_, cur->rc.rocker_r1,
                 cur->rc.rocker_l_, cur->rc.rocker_l1,
                 cur->rc.dial, cur->rc.switch_left, cur->rc.switch_right);

    AppendFormat(buffer, sizeof(buffer), &offset,
                 "Mouse X:%d Y:%d L:%u R:%u\r\n",
                 cur->mouse.x, cur->mouse.y, cur->mouse.press_l, cur->mouse.press_r);

    AppendFormat(buffer, sizeof(buffer), &offset,
                 "Keys Raw:0x%04X Ctrl:0x%04X Shift:0x%04X\r\n",
                 cur->key[KEY_PRESS].keys,
                 cur->key[KEY_PRESS_WITH_CTRL].keys,
                 cur->key[KEY_PRESS_WITH_SHIFT].keys);

    AppendKeyList(&cur->key[KEY_PRESS], buffer, sizeof(buffer), &offset);
    AppendKeyCounts("PressCount", cur->key_count[KEY_PRESS], buffer, sizeof(buffer), &offset);
    AppendKeyCounts("CtrlPressCount", cur->key_count[KEY_PRESS_WITH_CTRL], buffer, sizeof(buffer), &offset);
    AppendKeyCounts("ShiftPressCount", cur->key_count[KEY_PRESS_WITH_SHIFT], buffer, sizeof(buffer), &offset);

    TelemetrySendBuffer((uint8_t *)buffer, (uint16_t)offset);
}

static void SendHeartbeat(uint32_t seq)
{
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "[rc_demo] heartbeat #%lu (online=%u)\r\n",
                       (unsigned long)seq, (unsigned int)RemoteControlIsOnline());
    if (len > 0)
        TelemetrySendBuffer((uint8_t *)msg, (uint16_t)len);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    Debug_DisableWatchdogs();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_SPI1_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_USART3_UART_Init();
    MX_RNG_Init();
    MX_RTC_Init();
    MX_TIM1_Init();
    MX_TIM10_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_TIM8_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_SPI2_Init();
    MX_CRC_Init();
    MX_DAC_Init();

    BSPInit();
    InitTelemetryUsart();
    rc_data = RemoteControlInit(&huart3);

    LOGINFO("[rc_demo] ready, streaming DT7/DR16 data via USART6");

    uint32_t last_tx_tick = 0U;
    uint32_t last_hb_tick = 0U;
    uint32_t heartbeat_seq = 0U;
    bool is_online = false;

    while (1)
    {
        DaemonTask();

        uint32_t now = HAL_GetTick();
        if (now - last_tx_tick >= TELEMETRY_TX_INTERVAL_MS)
        {
            last_tx_tick = now;

            bool rc_online = RemoteControlIsOnline();
            if (rc_online)
            {
                if (!is_online)
                {
                    const char *online_msg = "[rc_demo] remote control online\r\n";
                    TelemetrySendString(online_msg);
                }
                DumpRemoteState();
            }
            else if (telemetry_usart != NULL)
            {
                const char *offline_msg = "[rc_demo] waiting for remote control...\r\n";
                TelemetrySendString(offline_msg);
            }
            is_online = rc_online;
        }

        if (now - last_hb_tick >= HEARTBEAT_INTERVAL_MS)
        {
            last_hb_tick = now;
            SendHeartbeat(heartbeat_seq++);
        }

        HAL_Delay(5);
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/**
 * @brief TIM14 1ms 中断回调，用于累加 HAL 的系统节拍
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14)
    {
        HAL_IncTick();
    }
}
/* USER CODE END 4 */
