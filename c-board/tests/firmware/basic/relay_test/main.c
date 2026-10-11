/**
 * @file main.c
 * @brief 继电器测试固件（tests/firmware/basic/relay_test）
 *
 * 继电器控制测试：
 * - 继电器是低电平toggle触发
 * - 检测到一次低电平就切换一次状态
 * - 继电器连接到 SPI2_CS (PB12 引脚)
 * - 每5秒发送一次toggle信号验证控制逻辑
 *
 * ---------------------------- 继电器接线 ----------------------------
 * - VCC -> 3.3V/5V
 * - GND -> GND
 * - IN  -> PB12 (SPI2_CS)
 *
 * ---------------------------- 测试现象 ----------------------------
 * - 每5秒继电器会切换一次状态（咔哒声）
 * - 指示灯会跟随状态变化
 */
#include "main.h"

#include "gpio.h"
#include "spi.h"
#include "usart.h"

#include "bsp_init.h"
#include "bsp_log.h"

/* 继电器控制引脚定义 */
#define RELAY_PIN GPIO_PIN_12
#define RELAY_PORT GPIOB

/* Toggle 间隔时间 (ms) */
#define RELAY_TOGGLE_INTERVAL_MS 5000

/* 低电平脉冲宽度 (ms) - 继电器需要一定的脉冲时间 */
#define RELAY_PULSE_WIDTH_MS 100

static uint8_t relay_state = 0;  // 0=off, 1=on

void SystemClock_Config(void);
static void Relay_Init(void);
static void Relay_Toggle(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_USART6_UART_Init();

    BSPInit();
    LOGINFO("[relay_test] relay test started");
    LOGINFO("[relay_test] relay connected to PB12 (SPI2_CS)");
    LOGINFO("[relay_test] will toggle every %d ms", RELAY_TOGGLE_INTERVAL_MS);

    // 初始化继电器引脚
    Relay_Init();

    // 初始状态为高电平（继电器不触发）
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);
    relay_state = 0;
    LOGINFO("[relay_test] initial state: OFF");

    uint32_t last_toggle_tick = HAL_GetTick();

    while (1) {
        uint32_t now = HAL_GetTick();

        // 每5秒执行一次toggle
        if (now - last_toggle_tick >= RELAY_TOGGLE_INTERVAL_MS) {
            Relay_Toggle();
            last_toggle_tick = now;
        }

        // 主循环延时
        HAL_Delay(10);
    }
}

/**
 * @brief 初始化继电器控制引脚
 */
static void Relay_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 使能GPIOB时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 配置为输出推挽模式
    GPIO_InitStruct.Pin = RELAY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RELAY_PORT, &GPIO_InitStruct);

    // 初始设置为高电平
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);
}

/**
 * @brief 发送继电器toggle信号
 *
 * 继电器是低电平触发：
 * 1. 拉低电平（继电器检测到下降沿，切换状态）
 * 2. 保持一定时间（确保继电器可靠触发）
 * 3. 拉高电平（释放）
 */
static void Relay_Toggle(void) {
    // 拉低电平 - 触发继电器
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);

    // 保持低电平一段时间
    HAL_Delay(RELAY_PULSE_WIDTH_MS);

    // 拉高电平 - 释放
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);

    // 切换状态并打印
    relay_state = !relay_state;
    LOGINFO("[relay_test] toggle: state=%s", relay_state ? "ON" : "OFF");
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM14) {
        HAL_IncTick();
    }
}

void SystemClock_Config(void) {
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

void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
    (void)file;
    (void)line;
}
#endif
