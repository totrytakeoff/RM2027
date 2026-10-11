/**
 * @file main.c
 * @brief 发射机构测试固件（tests/firmware/basic/shoot_test）
 *
 * This test drives three M3508 motors:
 * - 2x friction wheels (left/right)
 * - 1x loader (feeding disk)
 *
 * Runtime control via GDB (set variables):
 *   g_friction_enable = 1/0
 *   g_friction_speed_deg_s = target speed (deg/s)
 *   g_loader_mode = 0 stop, 1 speed, 2 periodic angle step
 *   g_loader_speed_deg_s = target speed (deg/s)
 *   g_loader_step_deg = step per interval (deg)
 *   g_loader_step_interval_ms = interval (ms)
 */
#include "main.h"

#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include "bsp_init.h"
#include "bsp_log.h"
#include "daemon.h"
#include "dji_motor.h"

#define FRICTION_MOTOR_COUNT 2U
static const uint8_t FRICTION_CAN_IDS[FRICTION_MOTOR_COUNT] = {1U, 2U};
#define LOADER_CAN_ID 5U

#define FRICTION_SPEED_MAX 30000.0f  // deg/s
#define FRICTION_SPEED_MIN (-FRICTION_SPEED_MAX)
#define LOADER_SPEED_MAX 7200.0f  // deg/s
#define LOADER_SPEED_MIN (-LOADER_SPEED_MAX)
#define LOADER_ANGLE_MAX 36000.0f
#define LOADER_ANGLE_MIN (-LOADER_ANGLE_MAX)

static DJIMotorInstance *friction_motors[FRICTION_MOTOR_COUNT] = {NULL};
static DJIMotorInstance *loader_motor = NULL;

volatile uint8_t g_friction_enable = 0;
volatile float g_friction_speed_deg_s = 20000.0f;

volatile uint8_t g_loader_mode = 1;  // 0 stop, 1 speed, 2 periodic angle step
volatile float g_loader_speed_deg_s = 3000.0f;
volatile float g_loader_step_deg = 36.0f;
volatile uint32_t g_loader_step_interval_ms = 300U;

void SystemClock_Config(void);
static void EnsureFrictionMotorsReady(void);
static void EnsureLoaderMotorReady(void);
static float ClampFloat(float value, float min, float max);

void ShootTest_StopAll(void);
void ShootTest_FrictionStop(void);
void ShootTest_LoaderStop(void);
void ShootTest_FrictionSpeedLoop(float target_speed_deg_s);
void ShootTest_LoaderSpeedLoop(float target_speed_deg_s);
void ShootTest_LoaderPeriodicAngleStep(float step_deg, uint32_t interval_ms);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_USART6_UART_Init();

    BSPInit();
    LOGINFO("[shoot_test] init finished");

    while (1) {
        if (g_friction_enable) {
            ShootTest_FrictionSpeedLoop(g_friction_speed_deg_s);
        } else {
            ShootTest_FrictionStop();
        }

        switch (g_loader_mode) {
            case 0:
                ShootTest_LoaderStop();
                break;
            case 1:
                ShootTest_LoaderSpeedLoop(g_loader_speed_deg_s);
                break;
            case 2:
                ShootTest_LoaderPeriodicAngleStep(g_loader_step_deg, g_loader_step_interval_ms);
                break;
            default:
                break;
        }

        DJIMotorControl();
        DaemonTask();
        HAL_Delay(2);
    }
}

void ShootTest_StopAll(void) {
    ShootTest_FrictionStop();
    ShootTest_LoaderStop();
}

void ShootTest_FrictionStop(void) {
    for (uint8_t i = 0; i < FRICTION_MOTOR_COUNT; ++i) {
        if (friction_motors[i] != NULL) {
            DJIMotorStop(friction_motors[i]);
        }
    }
}

void ShootTest_LoaderStop(void) {
    if (loader_motor != NULL) {
        DJIMotorStop(loader_motor);
    }
}

void ShootTest_FrictionSpeedLoop(float target_speed_deg_s) {
    EnsureFrictionMotorsReady();
    target_speed_deg_s = ClampFloat(target_speed_deg_s, FRICTION_SPEED_MIN, FRICTION_SPEED_MAX);

    for (uint8_t i = 0; i < FRICTION_MOTOR_COUNT; ++i) {
        DJIMotorInstance *motor = friction_motors[i];
        if (motor == NULL) {
            continue;
        }
        DJIMotorOuterLoop(motor, SPEED_LOOP);
        DJIMotorEnable(motor);
        DJIMotorSetRef(motor, target_speed_deg_s);
    }
}

void ShootTest_LoaderSpeedLoop(float target_speed_deg_s) {
    EnsureLoaderMotorReady();
    target_speed_deg_s = ClampFloat(target_speed_deg_s, LOADER_SPEED_MIN, LOADER_SPEED_MAX);
    DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
    DJIMotorEnable(loader_motor);
    DJIMotorSetRef(loader_motor, target_speed_deg_s);
}

void ShootTest_LoaderPeriodicAngleStep(float step_deg, uint32_t interval_ms) {
    EnsureLoaderMotorReady();

    static uint8_t step_initialized = 0;
    static float current_target = 0.0f;
    static uint32_t last_step_tick = 0;

    uint32_t now = HAL_GetTick();
    if (!step_initialized) {
        current_target =
                ClampFloat(loader_motor->measure.total_angle, LOADER_ANGLE_MIN, LOADER_ANGLE_MAX);
        last_step_tick = now;
        step_initialized = 1;
    }

    if (interval_ms != 0U) {
        uint32_t elapsed = now - last_step_tick;
        if (elapsed >= interval_ms) {
            uint32_t steps = elapsed / interval_ms;
            last_step_tick += steps * interval_ms;
            current_target += step_deg * (float)steps;
            current_target = ClampFloat(current_target, LOADER_ANGLE_MIN, LOADER_ANGLE_MAX);
        }
    }

    DJIMotorOuterLoop(loader_motor, ANGLE_LOOP);
    DJIMotorEnable(loader_motor);
    DJIMotorSetRef(loader_motor, current_target);
}

static void EnsureFrictionMotorsReady(void) {
    for (uint8_t i = 0; i < FRICTION_MOTOR_COUNT; ++i) {
        if (friction_motors[i] != NULL) {
            continue;
        }

        Motor_Init_Config_s config = {
                .can_init_config =
                        {
                                .can_handle = &hcan2,
                                .tx_id = FRICTION_CAN_IDS[i],
                        },
                .controller_param_init_config =
                        {
                                .angle_PID =
                                        {
                                                .Kp = 5.0f,
                                                .Ki = 0.0f,
                                                .Kd = 0.0f,
                                                .MaxOut = FRICTION_SPEED_MAX,
                                                .IntegralLimit = 500.0f,
                                                .Improve = PID_Trapezoid_Intergral |
                                                           PID_Integral_Limit,
                                        },
                                .speed_PID =
                                        {
                                                .Kp = 10.0f,
                                                .Ki = 0.0f,
                                                .Kd = 0.0f,
                                                .IntegralLimit = 3000.0f,
                                                .Improve = PID_Trapezoid_Intergral |
                                                           PID_Integral_Limit |
                                                           PID_Derivative_On_Measurement,
                                                .MaxOut = 35000.0f,
                                        },
                                .current_PID =
                                        {
                                                .Kp = 0.5f,
                                                .Ki = 0.0f,
                                                .Kd = 0.0f,
                                                .IntegralLimit = 3000.0f,
                                                .Improve = PID_Trapezoid_Intergral |
                                                           PID_Integral_Limit |
                                                           PID_Derivative_On_Measurement,
                                                .MaxOut = 30000.0f,
                                        },
                        },
                .controller_setting_init_config =
                        {
                                .angle_feedback_source = MOTOR_FEED,
                                .speed_feedback_source = MOTOR_FEED,
                                .outer_loop_type = SPEED_LOOP,
                                .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
                                .motor_reverse_flag = (i == 1U) ? MOTOR_DIRECTION_REVERSE
                                                                : MOTOR_DIRECTION_NORMAL,
                        },
                .motor_type = M3508,
        };

        friction_motors[i] = DJIMotorInit(&config);
    }
}

static void EnsureLoaderMotorReady(void) {
    if (loader_motor != NULL) {
        return;
    }

    Motor_Init_Config_s config = {
            .can_init_config =
                    {
                            .can_handle = &hcan1,
                            .tx_id = LOADER_CAN_ID,
                    },
            .controller_param_init_config =
                    {
                            .angle_PID =
                                    {
                                            .Kp = 5.0f,
                                            .Ki = 0.0f,
                                            .Kd = 0.0f,
                                            .MaxOut = LOADER_SPEED_MAX,
                                            .IntegralLimit = 500.0f,
                                            .Improve = PID_Trapezoid_Intergral |
                                                       PID_Integral_Limit,
                                    },
                            .speed_PID =
                                    {
                                            .Kp = 10.0f,
                                            .Ki = 0.0f,
                                            .Kd = 0.0f,
                                            .IntegralLimit = 3000.0f,
                                            .Improve = PID_Trapezoid_Intergral |
                                                       PID_Integral_Limit |
                                                       PID_Derivative_On_Measurement,
                                            .MaxOut = 12000.0f,
                                    },
                            .current_PID =
                                    {
                                            .Kp = 0.5f,
                                            .Ki = 0.0f,
                                            .Kd = 0.0f,
                                            .IntegralLimit = 3000.0f,
                                            .Improve = PID_Trapezoid_Intergral |
                                                       PID_Integral_Limit |
                                                       PID_Derivative_On_Measurement,
                                            .MaxOut = 15000.0f,
                                    },
                    },
            .controller_setting_init_config =
                    {
                            .angle_feedback_source = MOTOR_FEED,
                            .speed_feedback_source = MOTOR_FEED,
                            .outer_loop_type = SPEED_LOOP,
                            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
                            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    },
            .motor_type = M3508,
    };

    loader_motor = DJIMotorInit(&config);
}

static float ClampFloat(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
void assert_failed(uint8_t *file, uint32_t line) {
    (void)file;
    (void)line;
}
#endif
