/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : DM-IMU demo
 ******************************************************************************
 */
/* USER CODE END Header */

#include "adc.h"
#include "can.h"
#include "cmsis_os.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"

#include "bsp_init.h"
#include "bsp_usart.h"
#include "dm_imu.h"
#include "utils.h"

#include <string.h>

#define DM_IMU_USE_CAN 0
#define DM_IMU_UART_IFACE 0x00

static USARTInstance *usart6_;
static dm_imu_t dm_imu;

void SystemClock_Config(void);
void Error_Handler(void);

static void usart6_init(void)
{
    USART_Init_Config_s config = {
        .module_callback = NULL,
        .recv_buff_size = 1,
        .usart_handle = &huart6,
    };
    usart6_ = USARTRegister(&config);
}

static void dm_imu_init(void)
{
#if DM_IMU_USE_CAN
    dm_imu_can_config_t can_cfg = {
        .can_handle = &hcan1,
        .can_id = 0x01,
        .mst_id = 0x11,
    };
    dm_imu_init_can(&dm_imu, &can_cfg);
    dm_imu_can_set_active(&dm_imu, true);
    dm_imu_can_set_interval_ms(&dm_imu, 1);
    // 如果使用请求模式，可在主循环中定时请求
#else
    dm_imu_uart_config_t uart_cfg = {
        .usart_handle = &huart1,
        .imu_id = 0x01,
        .rx_len = DM_IMU_UART_DEFAULT_RX_LEN,
    };
    dm_imu_init_uart(&dm_imu, &uart_cfg);
    // 确保 IMU 输出接口为 USB/485 且开启全部输出
    dm_imu_uart_enter_settings(&dm_imu);
    dm_imu_uart_set_iface(&dm_imu, DM_IMU_UART_IFACE);
    dm_imu_uart_enable_outputs(&dm_imu, true, true, true, true);
    dm_imu_uart_save_params(&dm_imu);
    dm_imu_uart_exit_settings(&dm_imu);
#endif
}

int main(void)
{
    HAL_Init();

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

    usart6_init();
    dm_imu_init();

    USARTSend(usart6_, (uint8_t *)"DM-IMU demo start\r\n", 19, USART_TRANSFER_DMA);

    while (1)
    {
        dm_imu_data_t data;
#if DM_IMU_USE_CAN
        dm_imu_can_request_accel(&dm_imu);
        dm_imu_can_request_gyro(&dm_imu);
        dm_imu_can_request_euler(&dm_imu);
        dm_imu_can_request_quat(&dm_imu);
#endif
        if (dm_imu_get_data(&dm_imu, &data))
        {
            char buffer[160];
            safe_snprintf(buffer, sizeof(buffer),
                          "A[%.3f %.3f %.3f] G[%.3f %.3f %.3f] E[%.2f %.2f %.2f] Q[%.3f %.3f %.3f %.3f]\r\n",
                          data.accel[0], data.accel[1], data.accel[2],
                          data.gyro[0], data.gyro[1], data.gyro[2],
                          data.euler[0], data.euler[1], data.euler[2],
                          data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
            USARTSend(usart6_, (uint8_t *)buffer, strlen(buffer), USART_TRANSFER_DMA);
        }

        HAL_Delay(200);
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
