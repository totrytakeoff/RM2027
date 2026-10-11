/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Wheel-leg IMU test (DM-IMU CAN2 request mode)
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"

#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include <string.h>

#include "bsp_init.h"
#include "bsp_usart.h"
#include "dm_imu.h"
#include "utils.h"

#define IMU_REQUEST_INTERVAL_MS 20U
#define IMU_PRINT_INTERVAL_MS 50U
#define IMU_HEARTBEAT_INTERVAL_MS 1000U
#define IMU_SCAN_PRINT_INTERVAL_MS 1000U

#define IMU_SCAN_ENABLE 0
/* CAN2 filter banks allow 14 IDs; adjust if you want to scan other ranges. */
#define IMU_SCAN_ID_START 0x00
#define IMU_SCAN_ID_COUNT 14U

#define IMU_ID 0x01
#define IMU_MASTER_ID 0x00
#define IMU_RX_ID (IMU_MASTER_ID)

static USARTInstance *usart6_ = NULL;
static dm_imu_t dm_imu;
#if IMU_SCAN_ENABLE
static CANInstance *imu_scan_can[IMU_SCAN_ID_COUNT] = {0};
static CANInstance *imu_tx_can = NULL;
static uint32_t imu_scan_count[IMU_SCAN_ID_COUNT] = {0};
static uint8_t imu_scan_last_type[IMU_SCAN_ID_COUNT] = {0};
static uint32_t imu_scan_last_ms[IMU_SCAN_ID_COUNT] = {0};
#endif

void SystemClock_Config(void);
void Error_Handler(void);

static void Usart6Init(void)
{
    USART_Init_Config_s config = {
        .module_callback = NULL,
        .recv_buff_size = 1,
        .usart_handle = &huart6,
    };
    usart6_ = USARTRegister(&config);
}

static void TelemetrySend(const char *msg)
{
    if (!msg)
        return;

    uint16_t len = (uint16_t)strlen(msg);
    if (usart6_)
    {
        USARTSend(usart6_, (uint8_t *)msg, len, USART_TRANSFER_BLOCKING);
    }
    else
    {
        HAL_UART_Transmit(&huart6, (uint8_t *)msg, len, 1000);
    }
}

#if !IMU_SCAN_ENABLE
static void ImuInit(void)
{
    dm_imu_can_config_t can_cfg = {
        .can_handle = &hcan2,
        .can_id = IMU_ID,
        .mst_id = IMU_RX_ID,
    };
    dm_imu_init_can(&dm_imu, &can_cfg);
    dm_imu_can_set_active(&dm_imu, false);
}

static void ImuRequestOnce(void)
{
    dm_imu_can_request_accel(&dm_imu);
    dm_imu_can_request_gyro(&dm_imu);
    dm_imu_can_request_euler(&dm_imu);
    dm_imu_can_request_quat(&dm_imu);
}
#endif

#if IMU_SCAN_ENABLE
static void ImuScanRxCallback(CANInstance *instance)
{
    if (!instance || instance->rx_len < 1)
        return;
    uint16_t rx_id = (uint16_t)instance->rx_id;
    if (rx_id < IMU_SCAN_ID_START || rx_id >= (IMU_SCAN_ID_START + IMU_SCAN_ID_COUNT))
        return;
    uint8_t idx = (uint8_t)(rx_id - IMU_SCAN_ID_START);
    imu_scan_count[idx]++;
    imu_scan_last_type[idx] = instance->rx_buff[0];
    imu_scan_last_ms[idx] = HAL_GetTick();
}

static void ImuScanInit(void)
{
    for (uint8_t i = 0; i < IMU_SCAN_ID_COUNT; ++i)
    {
        uint16_t rx_id = (uint16_t)(IMU_SCAN_ID_START + i);
        CAN_Init_Config_s can_cfg = {
            .can_handle = &hcan2,
            .tx_id = IMU_ID,
            .rx_id = rx_id,
            .can_module_callback = ImuScanRxCallback,
            .id = NULL,
        };
        imu_scan_can[i] = CANRegister(&can_cfg);
    }
    imu_tx_can = imu_scan_can[0];
}

static void ImuScanRequest(uint8_t rid)
{
    if (!imu_tx_can)
        return;
    imu_tx_can->tx_buff[0] = 0xCC;
    imu_tx_can->tx_buff[1] = rid;
    imu_tx_can->tx_buff[2] = 0x00;
    imu_tx_can->tx_buff[3] = 0xDD;
    memset(&imu_tx_can->tx_buff[4], 0, 4);
    CANSetDLC(imu_tx_can, 8);
    CANTransmit(imu_tx_can, 1000U);
}

static void ImuScanRequestOnce(void)
{
    ImuScanRequest(0x01);
    ImuScanRequest(0x02);
    ImuScanRequest(0x03);
    ImuScanRequest(0x04);
}

static void ImuScanPrint(void)
{
    for (uint8_t i = 0; i < IMU_SCAN_ID_COUNT; ++i)
    {
        if (imu_scan_count[i] == 0)
            continue;
        uint16_t rx_id = (uint16_t)(IMU_SCAN_ID_START + i);
        char buffer[120];
        safe_snprintf(buffer, sizeof(buffer),
                      "imu_scan: rx=0x%02X cnt=%lu last_type=0x%02X age=%lu\r\n",
                      (unsigned)rx_id,
                      (unsigned long)imu_scan_count[i],
                      (unsigned)imu_scan_last_type[i],
                      (unsigned long)(HAL_GetTick() - imu_scan_last_ms[i]));
        TelemetrySend(buffer);
    }
}
#endif

#if !IMU_SCAN_ENABLE
static void ImuPrintIfReady(void)
{
    dm_imu_data_t data;
    if (!dm_imu_get_data(&dm_imu, &data))
        return;

    char buffer[180];
    safe_snprintf(buffer, sizeof(buffer),
                  "IMU A[%.3f %.3f %.3f] G[%.3f %.3f %.3f] E[%.2f %.2f %.2f] Q[%.3f %.3f %.3f %.3f] mask=0x%02X\r\n",
                  data.accel[0], data.accel[1], data.accel[2],
                  data.gyro[0], data.gyro[1], data.gyro[2],
                  data.euler[0], data.euler[1], data.euler[2],
                  data.quat[0], data.quat[1], data.quat[2], data.quat[3],
                  (unsigned)data.valid_mask);
    TelemetrySend(buffer);
}
#endif

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN2_Init();
    MX_USART6_UART_Init();

    BSPInit();

    Usart6Init();
#if IMU_SCAN_ENABLE
    ImuScanInit();
#else
    ImuInit();
#endif

    TelemetrySend("wheelleg_imu: start\r\n");

    uint32_t last_request_tick = 0;
    uint32_t last_print_tick = 0;
    uint32_t last_heartbeat_tick = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        if (now - last_request_tick >= IMU_REQUEST_INTERVAL_MS)
        {
            last_request_tick = now;
#if IMU_SCAN_ENABLE
            ImuScanRequestOnce();
#else
            ImuRequestOnce();
#endif
        }

#if IMU_SCAN_ENABLE
        if (now - last_print_tick >= IMU_SCAN_PRINT_INTERVAL_MS)
        {
            last_print_tick = now;
            ImuScanPrint();
        }
#else
        if (now - last_print_tick >= IMU_PRINT_INTERVAL_MS)
        {
            last_print_tick = now;
            ImuPrintIfReady();
        }

        if (now - last_heartbeat_tick >= IMU_HEARTBEAT_INTERVAL_MS)
        {
            last_heartbeat_tick = now;
            uint8_t rid = 0;
            uint8_t ack = 0;
            char buffer[96];
            dm_imu_get_last_ack(&dm_imu, &rid, &ack);
            safe_snprintf(buffer, sizeof(buffer),
                          "wheelleg_imu: waiting rx=0x%02X ack[rid=0x%02X code=0x%02X] alive=%u\r\n",
                          (unsigned)IMU_RX_ID, (unsigned)rid, (unsigned)ack,
                          (unsigned)dm_imu_is_alive(&dm_imu, 500));
            TelemetrySend(buffer);
        }
#endif

        HAL_Delay(2);
    }
}

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

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14)
    {
        HAL_IncTick();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
