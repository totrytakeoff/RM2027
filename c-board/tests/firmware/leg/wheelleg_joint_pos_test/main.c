/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Wheel-leg joint position observe demo (DM8009P)
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
#include "bsp_can.h"
#include "bsp_usart.h"
#include "dmmotor.h"
#include "utils.h"

#define TELEMETRY_INTERVAL_MS 50U
#define ZERO_TORQUE_INTERVAL_MS 2U
#define ENABLE_INTERVAL_MS 100U

#define FRONT_LEFT_ID 1U
#define REAR_LEFT_ID 2U
#define FRONT_RIGHT_ID 3U
#define REAR_RIGHT_ID 4U
#define DM_MASTER_ID 0x00U

#define DM_P_RANGE 12.5f
#define DM_V_RANGE 45.0f
#define DM_T_RANGE 54.0f

static USARTInstance *telemetry_usart = NULL;
static DMMotor_Handle *front_left = NULL;
static DMMotor_Handle *rear_left = NULL;
static DMMotor_Handle *front_right = NULL;
static DMMotor_Handle *rear_right = NULL;
static uint32_t last_enable_tick = 0;

void SystemClock_Config(void);
void Error_Handler(void);

static void TelemetryInit(void)
{
    USART_Init_Config_s config = {
        .module_callback = NULL,
        .recv_buff_size = 1,
        .usart_handle = &huart6,
    };
    telemetry_usart = USARTRegister(&config);
}

static void TelemetrySend(const char *msg)
{
    if (!msg || !telemetry_usart)
        return;
    USARTSend(telemetry_usart, (uint8_t *)msg, (uint16_t)strlen(msg), USART_TRANSFER_BLOCKING);
}

static DMMotor_Handle *JointInit(uint8_t motor_id)
{
    DMMotor_InitConfig config = {
        .can_handle = &hcan1,
        .motor_id = motor_id,
        .master_id = DM_MASTER_ID,
        .auto_clear_error = true,
        .auto_enable_mit = false,
        .auto_zero_position = false,
        .use_shared_feedback_id = true,
        .position_range = DM_P_RANGE,
        .velocity_range = DM_V_RANGE,
        .torque_range = DM_T_RANGE,
    };

    return DMMotor_Init(&config);
}

static void MotorsEnable(void)
{
    if (front_left) {
        DMMotor_ClearError(front_left, DM_MODE_MIT);
        DMMotor_Enable(front_left, DM_MODE_MIT);
    }
    if (rear_left) {
        DMMotor_ClearError(rear_left, DM_MODE_MIT);
        DMMotor_Enable(rear_left, DM_MODE_MIT);
    }
    if (front_right) {
        DMMotor_ClearError(front_right, DM_MODE_MIT);
        DMMotor_Enable(front_right, DM_MODE_MIT);
    }
    if (rear_right) {
        DMMotor_ClearError(rear_right, DM_MODE_MIT);
        DMMotor_Enable(rear_right, DM_MODE_MIT);
    }
}

static void SendZeroTorque(void)
{
    if (front_left) DMMotor_SendMIT(front_left, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (rear_left) DMMotor_SendMIT(rear_left, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (front_right) DMMotor_SendMIT(front_right, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (rear_right) DMMotor_SendMIT(rear_right, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_USART6_UART_Init();

    BSPInit();

    TelemetryInit();

    front_left = JointInit(FRONT_LEFT_ID);
    rear_left = JointInit(REAR_LEFT_ID);
    front_right = JointInit(FRONT_RIGHT_ID);
    rear_right = JointInit(REAR_RIGHT_ID);

    MotorsEnable();
    last_enable_tick = HAL_GetTick();

    TelemetrySend("wheelleg_joint_pos: start (no torque)\r\n");

    uint32_t last_telemetry_tick = 0;
    uint32_t last_zero_torque_tick = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (now - last_enable_tick >= ENABLE_INTERVAL_MS)
        {
            last_enable_tick = now;
            MotorsEnable();
        }

        if (now - last_zero_torque_tick >= ZERO_TORQUE_INTERVAL_MS)
        {
            last_zero_torque_tick = now;
            SendZeroTorque();
        }

        if (now - last_telemetry_tick >= TELEMETRY_INTERVAL_MS)
        {
            last_telemetry_tick = now;

            const DMMotor_Feedback *fl = front_left ? DMMotor_GetFeedback(front_left) : NULL;
            const DMMotor_Feedback *rl = rear_left ? DMMotor_GetFeedback(rear_left) : NULL;
            const DMMotor_Feedback *fr = front_right ? DMMotor_GetFeedback(front_right) : NULL;
            const DMMotor_Feedback *rr = rear_right ? DMMotor_GetFeedback(rear_right) : NULL;

            float fl_pos = fl ? fl->position_rad : 0.0f;
            float rl_pos = rl ? rl->position_rad : 0.0f;
            float fr_pos = fr ? fr->position_rad : 0.0f;
            float rr_pos = rr ? rr->position_rad : 0.0f;

            float hip_l = fl_pos;
            float hip_r = fr_pos;
            float knee_l = rl_pos - fl_pos;
            float knee_r = rr_pos - fr_pos;

            char buffer[200];
            safe_snprintf(buffer, sizeof(buffer),
                          "pos(rad) FL:%.3f RL:%.3f FR:%.3f RR:%.3f | hip[%.3f %.3f] knee[%.3f %.3f]\r\n",
                          fl_pos, rl_pos, fr_pos, rr_pos,
                          hip_l, hip_r, knee_l, knee_r);
            TelemetrySend(buffer);
        }

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
