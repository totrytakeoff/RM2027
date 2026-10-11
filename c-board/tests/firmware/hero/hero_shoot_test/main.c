/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Hero shooter remote control demo
 ******************************************************************************
 * @attention
 *
 * 控制逻辑：
 * - 左拨杆上：开启摩擦轮
 * - 右拨杆：下单发、中双连发、上连发
 * - 拨轮向下：允许拨弹发射，松开停止拨弹
 * - 右摇杆左右控制云台yaw，右摇杆上下控制云台pitch
 *
 * 电机配置：
 * - 摩擦轮：CAN2 ID 1/2
 * - 拨弹：CAN1 ID 5
 * - yaw 轴 GM6020：CAN1 ID 2
 * - pitch 轴 GM6020：CAN2 ID 5
 ******************************************************************************
 */

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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "bsp_init.h"
#include "bsp_log.h"
#include "daemon.h"
#include "dt7_remote.h"
#include "dji_motor.h"
#include "user_lib.h"

/* Private define ------------------------------------------------------------*/
#define SHOOT_UPDATE_INTERVAL_MS 20U  // 50Hz

#define FRICTION_MOTOR_COUNT 2U
static const uint8_t FRICTION_CAN_IDS[FRICTION_MOTOR_COUNT] = {1U, 2U};
#define LOADER_CAN_ID 5U

#define FRICTION_SPEED_MAX 30000.0f  // deg/s
#define FRICTION_SPEED_MIN (-FRICTION_SPEED_MAX)
#define FRICTION_SPEED_TARGET 30000.0f

#define LOADER_SPEED_MAX 12000.0f
#define LOADER_SPEED_MIN (-LOADER_SPEED_MAX)
#define LOADER_ANGLE_MAX 36000.0f
#define LOADER_ANGLE_MIN (-LOADER_ANGLE_MAX)
#define LOADER_GEAR_RATIO 19.0f
#define LOADER_OUTPUT_STEP_DEG 60.0f
#define LOADER_ANGLE_STEP_DEG (-LOADER_OUTPUT_STEP_DEG * LOADER_GEAR_RATIO)

#define SHOOT_INTERVAL_MS 2000U
#define LOADER_CONTINUOUS_SPEED (-12000.0f)

#define DIAL_DOWN_THRESHOLD -200
#define DIAL_UP_THRESHOLD -100

#define GM6020_SPEED_MAX 3600.0f
#define GM6020_SPEED_MIN (-GM6020_SPEED_MAX)
#define GM6020_ANGLE_MAX 1440.0f
#define GM6020_ANGLE_MIN (-GM6020_ANGLE_MAX)
#define GM6020_SPEED_DEADZONE 30.0f
#define GIMBAL_RC_DEADZONE 50
#define GIMBAL_SPEED_SCALE 0.5f

#define YAW_MOTOR_ID 2U
#define PITCH_MOTOR_ID 5U

#define PITCH_GRAVITY_FF_K -10000.0f
#define PITCH_GRAVITY_FF_MAX 20000.0f
#define PITCH_GRAVITY_FF_OFFSET_DEG 0.0f

#define PITCH_HOLD_KP 12.0f
#define PITCH_HOLD_MAX_SPEED 1200.0f
#define PITCH_HOLD_DEADBAND_DEG 0.2f
#define PITCH_HOLD_KD 0.35f
#define PITCH_HOLD_KI 0.8f
#define PITCH_HOLD_I_LIMIT 800.0f
#define PITCH_FF_LPF 0.9f

/* Private variables ---------------------------------------------------------*/
static RC_ctrl_t *rc_data = NULL;
static DJIMotorInstance *friction_motors[FRICTION_MOTOR_COUNT] = {NULL};
static DJIMotorInstance *loader_motor = NULL;
static DJIMotorInstance *yaw_motor = NULL;
static DJIMotorInstance *pitch_motor = NULL;

static uint8_t friction_enabled = 0;
static uint8_t fire_mode = 0; // 0: single, 1: double, 2: continuous

static uint8_t pending_shots = 0;
static uint8_t dial_active = 0;
static uint8_t dial_last = 0;
static uint8_t loader_initialized = 0;
static float loader_target_angle = 0.0f;
static uint32_t last_shot_tick = 0;
static uint32_t last_step_tick = 0;

static float yaw_speed_ref = 0.0f;
static float pitch_speed_ref = 0.0f;
static float pitch_hold_angle = 0.0f;
static uint8_t pitch_manual_active = 0;
static uint8_t pitch_manual_active_last = 0;
static float pitch_hold_i = 0.0f;
static float pitch_current_ff = 0.0f;
static uint32_t gimbal_last_tick = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Debug_DisableWatchdogs(void);
static void EnsureFrictionMotorsReady(void);
static void EnsureLoaderMotorReady(void);
static void ProcessRemoteControl(void);
static void UpdateFrictionControl(void);
static void UpdateLoaderControl(void);
static void GimbalMotorsInit(void);
static void ProcessGimbalControl(void);
static void UpdateGimbalControl(void);
static float ClampFloat(float value, float min, float max);

/* Private user code ---------------------------------------------------------*/

static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

static void EnsureFrictionMotorsReady(void)
{
    for (uint8_t i = 0; i < FRICTION_MOTOR_COUNT; ++i) {
        if (friction_motors[i] != NULL) {
            continue;
        }

        Motor_Init_Config_s config = {
            .can_init_config = {
                .can_handle = &hcan2,
                .tx_id = FRICTION_CAN_IDS[i],
            },
            .controller_param_init_config = {
                .angle_PID = {
                    .Kp = 5.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .MaxOut = FRICTION_SPEED_MAX,
                    .IntegralLimit = 500.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                },
                .speed_PID = {
                    .Kp = 10.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .IntegralLimit = 3000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                               PID_Derivative_On_Measurement,
                    .MaxOut = 12000.0f,
                },
                .current_PID = {
                    .Kp = 0.5f,
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
                .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
                .motor_reverse_flag = (i == 1U) ? MOTOR_DIRECTION_REVERSE
                                                : MOTOR_DIRECTION_NORMAL,
            },
            .motor_type = M3508,
        };

        friction_motors[i] = DJIMotorInit(&config);
    }
}

static void EnsureLoaderMotorReady(void)
{
    if (loader_motor != NULL) {
        return;
    }

    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = LOADER_CAN_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 5.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = LOADER_SPEED_MAX,
                .IntegralLimit = 500.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
            },
            .speed_PID = {
                .Kp = 12.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = 16000.0f,
            },
            .current_PID = {
                .Kp = 0.6f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = 20000.0f,
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

    loader_motor = DJIMotorInit(&config);
}

static void GimbalMotorsInit(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = YAW_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .DeadBand = 0.1f,
                .IntegralLimit = 100.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = 500.0f,
            },
            .speed_PID = {
                .Kp = 10.0f,
                .Ki = 40.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = 20000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };

    yaw_motor = DJIMotorInit(&config);
    LOGINFO("[hero_shoot] Yaw GM6020 registered on CAN1 id %u", (unsigned)YAW_MOTOR_ID);

    config.controller_param_init_config.current_feedforward_ptr = &pitch_current_ff;
    config.controller_setting_init_config.feedforward_flag = CURRENT_FEEDFORWARD;
    config.controller_param_init_config.speed_PID.MaxOut = 30000.0f;
    config.controller_param_init_config.speed_PID.Ki = 20.0f;
    config.can_init_config.can_handle = &hcan2;
    config.can_init_config.tx_id = PITCH_MOTOR_ID;
    pitch_motor = DJIMotorInit(&config);
    LOGINFO("[hero_shoot] Pitch GM6020 registered on CAN2 id %u", (unsigned)PITCH_MOTOR_ID);

    if (yaw_motor != NULL) {
        DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
        DJIMotorStop(yaw_motor);
    }

    if (pitch_motor != NULL) {
        DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
        DJIMotorStop(pitch_motor);
    }
}

static void ProcessRemoteControl(void)
{
    if (rc_data == NULL || !RemoteControlIsOnline()) {
        friction_enabled = 0;
        return;
    }

    const RC_ctrl_t *rc = &rc_data[TEMP];

    friction_enabled = switch_is_up(rc->rc.switch_left);

    if (switch_is_down(rc->rc.switch_right)) {
        fire_mode = 0;
    } else if (switch_is_mid(rc->rc.switch_right)) {
        fire_mode = 1;
    } else {
        fire_mode = 2;
    }

    if (!friction_enabled) {
        dial_active = 0;
        return;
    }

    if (!dial_active && rc->rc.dial < DIAL_DOWN_THRESHOLD) {
        dial_active = 1;
    } else if (dial_active && rc->rc.dial > DIAL_UP_THRESHOLD) {
        dial_active = 0;
    }

    uint8_t dial_edge = (dial_active && !dial_last);
    dial_last = dial_active;

    if (fire_mode == 2) {
        pending_shots = 0;
        return;
    }

    if (dial_edge) {
        pending_shots = (fire_mode == 0) ? 1 : 2;
        last_shot_tick = HAL_GetTick();
        last_step_tick = last_shot_tick - SHOOT_INTERVAL_MS;
    }

    if (pending_shots == 0) {
        uint32_t now = HAL_GetTick();
        if (now - last_shot_tick >= SHOOT_INTERVAL_MS) {
            pending_shots = (fire_mode == 0) ? 1 : 2;
            last_shot_tick = now;
            last_step_tick = now - SHOOT_INTERVAL_MS;
        }
    }
}

static void ProcessGimbalControl(void)
{
    if (rc_data == NULL || !RemoteControlIsOnline()) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        pitch_manual_active = 0;
        return;
    }

    const RC_ctrl_t *rc = &rc_data[TEMP];

    uint8_t all_in_deadzone = (fabs(rc->rc.rocker_r_) < GIMBAL_RC_DEADZONE) &&
                              (fabs(rc->rc.rocker_r1) < GIMBAL_RC_DEADZONE);

    if (all_in_deadzone) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        pitch_manual_active = 0;
        return;
    }

    yaw_speed_ref = (rc->rc.rocker_r_ / 660.0f) * GM6020_SPEED_MAX * GIMBAL_SPEED_SCALE;
    pitch_speed_ref = (-rc->rc.rocker_r1 / 660.0f) * GM6020_SPEED_MAX * GIMBAL_SPEED_SCALE;

    yaw_speed_ref = float_deadband(yaw_speed_ref, -GM6020_SPEED_DEADZONE, GM6020_SPEED_DEADZONE);
    pitch_speed_ref = float_deadband(pitch_speed_ref, -GM6020_SPEED_DEADZONE, GM6020_SPEED_DEADZONE);

    pitch_manual_active = (fabs(rc->rc.rocker_r1) >= GIMBAL_RC_DEADZONE);
}

static void UpdateFrictionControl(void)
{
    EnsureFrictionMotorsReady();

    if (!friction_enabled) {
        for (uint8_t i = 0; i < FRICTION_MOTOR_COUNT; ++i) {
            if (friction_motors[i] != NULL) {
                DJIMotorStop(friction_motors[i]);
            }
        }
        return;
    }

    float target_speed = ClampFloat(FRICTION_SPEED_TARGET, FRICTION_SPEED_MIN, FRICTION_SPEED_MAX);
    for (uint8_t i = 0; i < FRICTION_MOTOR_COUNT; ++i) {
        DJIMotorInstance *motor = friction_motors[i];
        if (motor == NULL) {
            continue;
        }
        DJIMotorOuterLoop(motor, SPEED_LOOP);
        DJIMotorEnable(motor);
        DJIMotorSetRef(motor, target_speed);
    }
}

static void UpdateLoaderControl(void)
{
    EnsureLoaderMotorReady();

    if (!friction_enabled || loader_motor == NULL) {
        if (loader_motor != NULL) {
            DJIMotorStop(loader_motor);
        }
        pending_shots = 0;
        loader_initialized = 0;
        return;
    }

    if (!dial_active) {
        DJIMotorStop(loader_motor);
        pending_shots = 0;
        loader_initialized = 0;
        return;
    }

    uint32_t now = HAL_GetTick();
    if (fire_mode == 2) {
        pending_shots = 0;
        loader_initialized = 0;
        float speed_ref = ClampFloat(LOADER_CONTINUOUS_SPEED, LOADER_SPEED_MIN, LOADER_SPEED_MAX);
        DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
        DJIMotorEnable(loader_motor);
        DJIMotorSetRef(loader_motor, speed_ref);
        return;
    }

    if (!loader_initialized) {
        loader_target_angle = ClampFloat(loader_motor->measure.total_angle, LOADER_ANGLE_MIN, LOADER_ANGLE_MAX);
        if (pending_shots > 0) {
            last_step_tick = now - SHOOT_INTERVAL_MS;
        } else {
            last_step_tick = now;
        }
        loader_initialized = 1;
    }

    if (pending_shots > 0) {
        uint32_t elapsed = now - last_step_tick;
        if (elapsed >= SHOOT_INTERVAL_MS) {
            uint32_t steps = elapsed / SHOOT_INTERVAL_MS;
            if (steps > pending_shots) {
                steps = pending_shots;
            }
            last_step_tick += steps * SHOOT_INTERVAL_MS;
            loader_target_angle += LOADER_ANGLE_STEP_DEG * (float)steps;
            loader_target_angle = ClampFloat(loader_target_angle, LOADER_ANGLE_MIN, LOADER_ANGLE_MAX);
            pending_shots -= (uint8_t)steps;
        }
    } else {
        DJIMotorStop(loader_motor);
        return;
    }

    DJIMotorOuterLoop(loader_motor, ANGLE_LOOP);
    DJIMotorEnable(loader_motor);
    DJIMotorSetRef(loader_motor, loader_target_angle);
}

static void UpdateGimbalControl(void)
{
    if (yaw_motor == NULL || pitch_motor == NULL) {
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt = 0.02f;
    if (gimbal_last_tick != 0) {
        dt = (now - gimbal_last_tick) / 1000.0f;
        if (dt <= 0.0f) {
            dt = 0.02f;
        }
    }
    dt = float_constrain(dt, 0.001f, 0.05f);
    gimbal_last_tick = now;

    yaw_speed_ref = float_constrain(yaw_speed_ref, GM6020_SPEED_MIN, GM6020_SPEED_MAX);

    float pitch_ff_raw =
        PITCH_GRAVITY_FF_K *
        sinf((pitch_motor->measure.total_angle - PITCH_GRAVITY_FF_OFFSET_DEG) * PI / 180.0f);
    pitch_ff_raw = float_constrain(pitch_ff_raw, -PITCH_GRAVITY_FF_MAX, PITCH_GRAVITY_FF_MAX);
    pitch_current_ff = pitch_current_ff * PITCH_FF_LPF + pitch_ff_raw * (1.0f - PITCH_FF_LPF);

    DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
    DJIMotorEnable(yaw_motor);
    DJIMotorSetRef(yaw_motor, yaw_speed_ref);

    if (pitch_manual_active_last && !pitch_manual_active) {
        pitch_hold_angle = pitch_motor->measure.total_angle;
        pitch_hold_i = 0.0f;
    }
    pitch_manual_active_last = pitch_manual_active;

    float pitch_hold_speed = 0.0f;
    if (!pitch_manual_active) {
        float error = pitch_hold_angle - pitch_motor->measure.total_angle;
        if (fabsf(error) <= PITCH_HOLD_DEADBAND_DEG) {
            pitch_hold_i = 0.0f;
        } else {
            pitch_hold_i += error * dt;
            pitch_hold_i = float_constrain(pitch_hold_i, -PITCH_HOLD_I_LIMIT, PITCH_HOLD_I_LIMIT);
            pitch_hold_speed = error * PITCH_HOLD_KP +
                               pitch_hold_i * PITCH_HOLD_KI -
                               pitch_motor->measure.speed_aps * PITCH_HOLD_KD;
            pitch_hold_speed = float_constrain(pitch_hold_speed, -PITCH_HOLD_MAX_SPEED, PITCH_HOLD_MAX_SPEED);
        }
    }

    float pitch_speed_cmd = pitch_speed_ref + pitch_hold_speed;
    pitch_speed_cmd = float_constrain(pitch_speed_cmd, GM6020_SPEED_MIN, GM6020_SPEED_MAX);

    DJIMotorOuterLoop(pitch_motor, SPEED_LOOP);
    DJIMotorEnable(pitch_motor);
    DJIMotorSetRef(pitch_motor, pitch_speed_cmd);
}

static float ClampFloat(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

int main(void)
{
    HAL_Init();
    Debug_DisableWatchdogs();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();

    BSPInit();

    GimbalMotorsInit();

    rc_data = RemoteControlInit(&huart3);

    LOGINFO("[hero_shoot] demo initialized");
    LOGINFO("[hero_shoot] left switch up -> friction on");
    LOGINFO("[hero_shoot] right switch: down single, mid double, up continuous");
    LOGINFO("[hero_shoot] dial down to feed, release to stop");
    LOGINFO("[hero_shoot] right stick left/right -> yaw, up/down -> pitch");

    uint32_t last_update_tick = 0;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

    while (1) {
        DaemonTask();
        DJIMotorControl();

        uint32_t now = HAL_GetTick();
        if (now - last_update_tick >= SHOOT_UPDATE_INTERVAL_MS) {
            last_update_tick = now;

            ProcessRemoteControl();
            ProcessGimbalControl();
            UpdateFrictionControl();
            UpdateLoaderControl();
            UpdateGimbalControl();
        }

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
