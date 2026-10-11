/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : ET08 SBUS telemetry demo
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026.
 * All rights reserved.
 *
 * 本测试固件用于解析标准 SBUS 帧并通过 USART6 输出各通道原始值，方便测量
 * ET08(RF206S) 的通道映射和数值范围。
 *
 * - SBUS 输入: USART3 (100k 8E2)
 * - 串口输出: USART6 (文本)
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint16_t ch[16];
    uint8_t ch17;
    uint8_t ch18;
    uint8_t frame_lost;
    uint8_t failsafe;
} SbusFrame_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TELEMETRY_TX_INTERVAL_MS 50U
#define TELEMETRY_USART_RX_DUMMY 32U
#define HEARTBEAT_INTERVAL_MS 1000U
#define SBUS_FRAME_SIZE 25U
#define SBUS_START_BYTE 0x0F
#define SBUS_BAUDRATE 100000U
#define SBUS_ONLINE_TIMEOUT_MS 120U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static USARTInstance *sbus_usart = NULL;
static USARTInstance *telemetry_usart = NULL;
static volatile SbusFrame_t sbus_frame;
static volatile uint32_t sbus_last_tick = 0U;
static volatile uint32_t sbus_frame_count = 0U;
static volatile uint32_t sbus_bad_count = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Debug_DisableWatchdogs(void);
static void InitTelemetryUsart(void);
static void TelemetrySendBuffer(const uint8_t *buffer, uint16_t len);
static void TelemetrySendString(const char *str);
static void AppendFormat(char *buffer, size_t buffer_size, size_t *offset, const char *fmt, ...);
static void DumpSbusState(void);
static void SendHeartbeat(uint32_t seq);
static void ReinitUartForSbus(UART_HandleTypeDef *uart_handle);
static void SbusDecode(const uint8_t *buf, SbusFrame_t *out);
static void SbusRxCallback(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

static void InitTelemetryUsart(void)
{
    USART_Init_Config_s config = {
        .module_callback = NULL,
        .recv_buff_size = TELEMETRY_USART_RX_DUMMY,
        .usart_handle = &huart6,
    };
    telemetry_usart = USARTRegister(&config);
    TelemetrySendString("[et08] USART6 telemetry ready\r\n");
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

static void ReinitUartForSbus(UART_HandleTypeDef *uart_handle)
{
    if (uart_handle == NULL)
        return;

    (void)HAL_UART_DeInit(uart_handle);
    uart_handle->Init.BaudRate = SBUS_BAUDRATE;
    uart_handle->Init.WordLength = UART_WORDLENGTH_9B;
    uart_handle->Init.StopBits = UART_STOPBITS_2;
    uart_handle->Init.Parity = UART_PARITY_EVEN;
    uart_handle->Init.Mode = UART_MODE_TX_RX;
    uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_handle->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(uart_handle) != HAL_OK)
    {
        LOGWARNING("[et08] sbus uart reinit failed");
    }
}

static void SbusDecode(const uint8_t *buf, SbusFrame_t *out)
{
    out->ch[0] = (uint16_t)((buf[1] | (buf[2] << 8)) & 0x07FF);
    out->ch[1] = (uint16_t)(((buf[2] >> 3) | (buf[3] << 5)) & 0x07FF);
    out->ch[2] = (uint16_t)(((buf[3] >> 6) | (buf[4] << 2) | (buf[5] << 10)) & 0x07FF);
    out->ch[3] = (uint16_t)(((buf[5] >> 1) | (buf[6] << 7)) & 0x07FF);
    out->ch[4] = (uint16_t)(((buf[6] >> 4) | (buf[7] << 4)) & 0x07FF);
    out->ch[5] = (uint16_t)(((buf[7] >> 7) | (buf[8] << 1) | (buf[9] << 9)) & 0x07FF);
    out->ch[6] = (uint16_t)(((buf[9] >> 2) | (buf[10] << 6)) & 0x07FF);
    out->ch[7] = (uint16_t)(((buf[10] >> 5) | (buf[11] << 3)) & 0x07FF);
    out->ch[8] = (uint16_t)((buf[12] | (buf[13] << 8)) & 0x07FF);
    out->ch[9] = (uint16_t)(((buf[13] >> 3) | (buf[14] << 5)) & 0x07FF);
    out->ch[10] = (uint16_t)(((buf[14] >> 6) | (buf[15] << 2) | (buf[16] << 10)) & 0x07FF);
    out->ch[11] = (uint16_t)(((buf[16] >> 1) | (buf[17] << 7)) & 0x07FF);
    out->ch[12] = (uint16_t)(((buf[17] >> 4) | (buf[18] << 4)) & 0x07FF);
    out->ch[13] = (uint16_t)(((buf[18] >> 7) | (buf[19] << 1) | (buf[20] << 9)) & 0x07FF);
    out->ch[14] = (uint16_t)(((buf[20] >> 2) | (buf[21] << 6)) & 0x07FF);
    out->ch[15] = (uint16_t)(((buf[21] >> 5) | (buf[22] << 3)) & 0x07FF);

    uint8_t flags = buf[23];
    out->ch17 = (flags & 0x01U) ? 1U : 0U;
    out->ch18 = (flags & 0x02U) ? 1U : 0U;
    out->frame_lost = (flags & 0x04U) ? 1U : 0U;
    out->failsafe = (flags & 0x08U) ? 1U : 0U;
}

static void SbusRxCallback(void)
{
    if (sbus_usart == NULL)
        return;

    const uint8_t *buf = sbus_usart->recv_buff;
    if (buf[0] != SBUS_START_BYTE)
    {
        sbus_bad_count++;
        return;
    }

    SbusFrame_t frame;
    SbusDecode(buf, &frame);
    sbus_frame = frame;
    sbus_last_tick = HAL_GetTick();
    sbus_frame_count++;
}

static void DumpSbusState(void)
{
    if (telemetry_usart == NULL)
        return;

    SbusFrame_t snapshot = sbus_frame;

    char buffer[512];
    size_t offset = 0U;

    AppendFormat(buffer, sizeof(buffer), &offset,
                 "[et08] frames:%lu bad:%lu flags(17:%u 18:%u lost:%u fs:%u)\r\n",
                 (unsigned long)sbus_frame_count,
                 (unsigned long)sbus_bad_count,
                 (unsigned int)snapshot.ch17,
                 (unsigned int)snapshot.ch18,
                 (unsigned int)snapshot.frame_lost,
                 (unsigned int)snapshot.failsafe);

    AppendFormat(buffer, sizeof(buffer), &offset,
                 "CH00-07: %4u %4u %4u %4u %4u %4u %4u %4u\r\n",
                 snapshot.ch[0], snapshot.ch[1], snapshot.ch[2], snapshot.ch[3],
                 snapshot.ch[4], snapshot.ch[5], snapshot.ch[6], snapshot.ch[7]);
    AppendFormat(buffer, sizeof(buffer), &offset,
                 "CH08-15: %4u %4u %4u %4u %4u %4u %4u %4u\r\n",
                 snapshot.ch[8], snapshot.ch[9], snapshot.ch[10], snapshot.ch[11],
                 snapshot.ch[12], snapshot.ch[13], snapshot.ch[14], snapshot.ch[15]);

    AppendFormat(buffer, sizeof(buffer), &offset,
                 "Center:  %4d %4d %4d %4d %4d %4d %4d %4d\r\n",
                 (int)snapshot.ch[0] - 1024, (int)snapshot.ch[1] - 1024,
                 (int)snapshot.ch[2] - 1024, (int)snapshot.ch[3] - 1024,
                 (int)snapshot.ch[4] - 1024, (int)snapshot.ch[5] - 1024,
                 (int)snapshot.ch[6] - 1024, (int)snapshot.ch[7] - 1024);
    AppendFormat(buffer, sizeof(buffer), &offset,
                 "Center:  %4d %4d %4d %4d %4d %4d %4d %4d\r\n",
                 (int)snapshot.ch[8] - 1024, (int)snapshot.ch[9] - 1024,
                 (int)snapshot.ch[10] - 1024, (int)snapshot.ch[11] - 1024,
                 (int)snapshot.ch[12] - 1024, (int)snapshot.ch[13] - 1024,
                 (int)snapshot.ch[14] - 1024, (int)snapshot.ch[15] - 1024);

    TelemetrySendBuffer((uint8_t *)buffer, (uint16_t)offset);
}

static void SendHeartbeat(uint32_t seq)
{
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "[et08] heartbeat #%lu\r\n", (unsigned long)seq);
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

    ReinitUartForSbus(&huart3);

    USART_Init_Config_s sbus_config = {
        .module_callback = SbusRxCallback,
        .recv_buff_size = SBUS_FRAME_SIZE,
        .usart_handle = &huart3,
    };
    sbus_usart = USARTRegister(&sbus_config);

    LOGINFO("[et08] ready, streaming SBUS data via USART6");

    uint32_t last_tx_tick = 0U;
    uint32_t last_hb_tick = 0U;
    uint32_t heartbeat_seq = 0U;
    bool was_online = false;

    while (1)
    {
        DaemonTask();

        uint32_t now = HAL_GetTick();
        bool is_online = (now - sbus_last_tick) < SBUS_ONLINE_TIMEOUT_MS;

        if (now - last_tx_tick >= TELEMETRY_TX_INTERVAL_MS)
        {
            last_tx_tick = now;

            if (is_online)
            {
                if (!was_online)
                    TelemetrySendString("[et08] sbus online\r\n");
                DumpSbusState();
            }
            else
            {
                TelemetrySendString("[et08] waiting for sbus...\r\n");
            }
            was_online = is_online;
        }

        if (now - last_hb_tick >= HEARTBEAT_INTERVAL_MS)
        {
            last_hb_tick = now;
            SendHeartbeat(heartbeat_seq++);
        }

        HAL_Delay(500);
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14)
    {
        HAL_IncTick();
    }
}
/* USER CODE END 4 */
