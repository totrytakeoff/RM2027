/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Wheel-leg wheel hub basic test (M3508 + ET08)
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
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dji_motor.h"
#include "et08_remote.h"

#define UPDATE_INTERVAL_MS 20U

#define WHEEL_MOTOR_COUNT 2U
static const uint8_t WHEEL_CAN_IDS[WHEEL_MOTOR_COUNT] = {1U, 2U};

#define WHEEL_SPEED_MAX 20000.0f
#define WHEEL_SPEED_MIN (-WHEEL_SPEED_MAX)

#define RC_STICK_MAX 660.0f
#define RC_DEADZONE 30

static USARTInstance *telemetry_usart = NULL;
static ET08_Ctrl_t *et08_ctrl = NULL;

static DJIMotorInstance *wheel_motors[WHEEL_MOTOR_COUNT] = {NULL};

static float wheel_left_ref = 0.0f;
static float wheel_right_ref = 0.0f;

void SystemClock_Config(void);
void Error_Handler(void);

static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

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
    if (!telemetry_usart || !msg)
        return;
    USARTSend(telemetry_usart, (uint8_t *)msg, (uint16_t)strlen(msg), USART_TRANSFER_DMA);
}

static void WheelMotorsInit(void)
{
    for (uint8_t i = 0; i < WHEEL_MOTOR_COUNT; ++i)
    {
        Motor_Init_Config_s config = {
            .can_init_config = {
                .can_handle = &hcan1,
                .tx_id = WHEEL_CAN_IDS[i],
            },
            .controller_param_init_config = {
                .angle_PID = {
                    .Kp = 5.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .MaxOut = WHEEL_SPEED_MAX,
                    .IntegralLimit = 500.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                },
                .speed_PID = {
                    .Kp = 5.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .IntegralLimit = 3000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                               PID_Derivative_On_Measurement,
                    .MaxOut = 12000.0f,
                },
                .current_PID = {
                    .Kp = 0.4f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .IntegralLimit = 3000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                               PID_Derivative_On_Measurement,
                    .MaxOut = 15000.0f,
                },
            },
            .controller_setting_init_config = {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type = SPEED_LOOP,
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
                .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            },
            .motor_type = M3508,
        };

        wheel_motors[i] = DJIMotorInit(&config);
        LOGINFO("[wheelleg_basic] wheel motor id %u ready", WHEEL_CAN_IDS[i]);
    }
}

static int16_t ApplyDeadzone(int16_t value, int16_t deadzone)
{
    if (value > deadzone)
        return value - deadzone;
    if (value < -deadzone)
        return value + deadzone;
    return 0;
}

static float ClampFloat(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static void ProcessRemoteControl(void)
{
    if (!ET08_IsOnline() || et08_ctrl == NULL)
    {
        wheel_left_ref = 0.0f;
        wheel_right_ref = 0.0f;
        return;
    }

    int16_t left_y = ApplyDeadzone(et08_ctrl->left.y, RC_DEADZONE);
    int16_t right_y = ApplyDeadzone(et08_ctrl->right.y, RC_DEADZONE);

    wheel_left_ref = -(float)left_y / RC_STICK_MAX * WHEEL_SPEED_MAX;
    wheel_right_ref = (float)right_y / RC_STICK_MAX * WHEEL_SPEED_MAX;

    wheel_left_ref = ClampFloat(wheel_left_ref, WHEEL_SPEED_MIN, WHEEL_SPEED_MAX);
    wheel_right_ref = ClampFloat(wheel_right_ref, WHEEL_SPEED_MIN, WHEEL_SPEED_MAX);
}

static void UpdateWheelMotors(void)
{
    if (wheel_motors[0])
    {
        DJIMotorOuterLoop(wheel_motors[0], SPEED_LOOP);
        DJIMotorEnable(wheel_motors[0]);
        DJIMotorSetRef(wheel_motors[0], wheel_left_ref);
    }
    if (wheel_motors[1])
    {
        DJIMotorOuterLoop(wheel_motors[1], SPEED_LOOP);
        DJIMotorEnable(wheel_motors[1]);
        DJIMotorSetRef(wheel_motors[1], wheel_right_ref);
    }
}

int main(void)
{
    HAL_Init();
    Debug_DisableWatchdogs();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();

    BSPInit();

    TelemetryInit();
    WheelMotorsInit();

    et08_ctrl = ET08_Init(&huart3);

    TelemetrySend("wheelleg_wheel: start\r\n");
    LOGINFO("[wheelleg_basic] ET08 right.y -> wheel id2, left.y -> wheel id1");

    uint32_t last_update_tick = 0;

    while (1)
    {
        DaemonTask();
        DJIMotorControl();

        uint32_t now = HAL_GetTick();
        if (now - last_update_tick >= UPDATE_INTERVAL_MS)
        {
            last_update_tick = now;
            ProcessRemoteControl();
        }

        UpdateWheelMotors();

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
