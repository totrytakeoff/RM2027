/**
 * @file main.c
 * @brief Robot最小框架主入口
 * @note 单一主循环, 支持ET08遥控和VT图传键鼠
 */

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

#include "robot_app.h"
#include "robot_config.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dji_motor.h"
#include "ins_task.h"

/*============================================================================
 * 前置声明
 *============================================================================*/
void SystemClock_Config(void);
void Error_Handler(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/*============================================================================
 * 私有变量
 *============================================================================*/
static uint32_t last_tick = 0;

/*============================================================================
 * 主函数
 *============================================================================*/
int main(void)
{
    // HAL初始化 (由CubeMX生成)
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


    // BSP初始化
    DWT_Init(168);
    DaemonServiceInit();
    /* Keep the bare-metal comparison shell on the same bounded ingress path
     * as the formal FreeRTOS firmware.  RobotApp_Init() registers UART/CAN
     * endpoints, so the dispatch mode must be selected first. */
    if (!CANConfigureDispatch(CAN_DISPATCH_DEFERRED) ||
        !USARTConfigureDispatch(USART_DISPATCH_DEFERRED)) {
        Error_Handler();
    }
#if MINIMAL_DEBUG_ENABLE && ((MINIMAL_DEBUG_MODE & MINIMAL_DEBUG_MODE_TEXT) != 0)
    BSPLogInit();
#endif
    if (!RobotApp_Init()) {
        Error_Handler();
    }
    
    last_tick = DWT_GetTimeline_ms();
    // 主循环
    while (1) {
        INS_Task();
        DaemonTask();
        /* Parse bounded ingress snapshots before consuming motor feedback. */
        (void)CANDispatchPending(0U);
        (void)USARTDispatchPending(0U);
        RobotApp_MotorStep();

        uint32_t now = DWT_GetTimeline_ms();
        uint32_t dt = now - last_tick;
        
        // 周期控制
        if (dt >= MAIN_LOOP_PERIOD_MS) {
            last_tick = now;
            
            RobotApp_ControlStep(now);
        }
        RobotApp_DiagnosticsStep(now);

        /* 与omni_demo对齐: 限制电机控制发送频率，避免CAN邮箱打满 */
        HAL_Delay(5);
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

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
            RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14) {
        HAL_IncTick();
    }
}


/*============================================================================
 * 错误处理
 *============================================================================*/
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        // 死循环
    }
}
