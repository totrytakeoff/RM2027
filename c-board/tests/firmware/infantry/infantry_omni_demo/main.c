
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Infantry omni demo (omni chassis + gimbal + shooter, ET08)
 ******************************************************************************
 * @attention
 *
 * Control mapping (ET08):
 * - SA: friction (up = on, down = off)
 * - SB: loader (up = continuous, mid = stop, down = single per toggle)
 * - SD: gimbal follow (up = follow, down = no follow)
 * - SC: unused
 * - Right stick: chassis rotate (x), gimbal pitch (y)
 * - Left stick: chassis translation (x = strafe, y = forward/back)
 * - LD/RD: unused
 *
 * CAN mapping:
 * - Chassis motors: CAN1 (M3508)
 * - Shooter motors: CAN2 (friction 1/2, loader 6)
 * - Gimbal: yaw CAN1 id1, pitch CAN2 id1
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
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_init.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dji_motor.h"
#include "et08_remote.h"
#include "user_lib.h"

/* Private define ------------------------------------------------------------*/
#define UPDATE_INTERVAL_MS 20U // 50Hz
#define DEMO_CHASSIS_ONLY 0
#define CHASSIS_FORCE_TEST 0

// ---------------- ET08 SBUS ----------------
#define SBUS_FRAME_SIZE 25U
#define SBUS_START_BYTE 0x0F
#define SBUS_BAUDRATE 100000U
#define SBUS_ONLINE_TIMEOUT_MS 1000U

// ---------------- Shooter motors (CAN2) ----------------
#define FRICTION_MOTOR_COUNT 2U
static const uint8_t FRICTION_CAN_IDS[FRICTION_MOTOR_COUNT] = {1U, 2U};
#define LOADER_CAN_ID 6U

#define FRICTION_SPEED_MAX 30000.0f
#define FRICTION_SPEED_MIN (-FRICTION_SPEED_MAX)
#define FRICTION_DIRECTION_SIGN (-1.0f)
#define FRICTION_SPEED_TARGET (FRICTION_DIRECTION_SIGN * 30000.0f)

#define LOADER_SPEED_MAX 12000.0f
#define LOADER_SPEED_MIN (-LOADER_SPEED_MAX)
#define LOADER_ANGLE_MAX 36000.0f
#define LOADER_ANGLE_MIN (-LOADER_ANGLE_MAX)

#define LOADER_GEAR_RATIO 13.0f
#define LOADER_OUTPUT_STEP_DEG 45.0f
#define LOADER_DIRECTION_SIGN (1.0f)
#define LOADER_SINGLE_DIRECTION_SIGN (LOADER_DIRECTION_SIGN)
#define LOADER_SINGLE_REF_SIGN (1.0f)
#define LOADER_ANGLE_STEP_DEG (LOADER_OUTPUT_STEP_DEG * LOADER_GEAR_RATIO)
#define LOADER_SINGLE_SETTLE_EPS_DEG 5.0f
#define LOADER_ANGLE_FEEDBACK_SIGN (1)
#define LOADER_SINGLE_SPEED (LOADER_DIRECTION_SIGN * 7000.0f)

#define SHOOT_INTERVAL_MS 2000U
#define LOADER_CONTINUOUS_SPEED (LOADER_DIRECTION_SIGN * 20000.0f)

#define LOADER_ANGLE_KP_BASE 3.5f
#define LOADER_ANGLE_KP_BOOST 6.0f
#define LOADER_ANGLE_KD_BASE 0.5f
#define LOADER_ANGLE_KD_BOOST 4.0f
#define LOADER_ANGLE_MAXOUT_BASE LOADER_SPEED_MAX
#define LOADER_ANGLE_MAXOUT_BOOST 15000.0f
#define LOADER_SPEED_KP_BASE 4.5f
#define LOADER_SPEED_KP_BOOST 9.0f
#define LOADER_SPEED_KD_BASE 0.2f
#define LOADER_SPEED_KD_BOOST 0.2f
#define LOADER_SPEED_MAXOUT_BASE 22000.0f
#define LOADER_SPEED_MAXOUT_BOOST 30000.0f
#define LOADER_CURRENT_MAXOUT_BASE 28000.0f
#define LOADER_CURRENT_MAXOUT_BOOST 30000.0f
#define LOADER_CONTINUOUS_SLEW_PER_MS 40.0f

// ---------------- ET08 mapping ----------------
#define ET08_SWITCH_POS_UP 0U
#define ET08_SWITCH_POS_MID 1U
#define ET08_SWITCH_POS_DOWN 2U
#define ET08_SWITCH_INVALID 0xFFu
#define SB_SWITCH_DEBOUNCE_MS 30U
#define LOADER_CONTINUOUS_HOLD_MS 250U
#define SB_RAW_UP 1896U
#define SB_RAW_MID 1694U
#define SB_RAW_DOWN 1493U
#define SB_RAW_TOLERANCE 200U
#define SA_SB_RAW_TOLERANCE 200U

// Stick channel mapping (adjust to your radio)
// 0: left stick = CH3/CH4, right stick = CH1/CH2
// 1: left stick = CH1/CH2, right stick = CH3/CH4
#define ET08_MAPPING_MODE 0
#if ET08_MAPPING_MODE == 0
#define ET08_LEFT_X_CH ET08_CH4
#define ET08_LEFT_Y_CH ET08_CH3
#define ET08_RIGHT_X_CH ET08_CH1
#define ET08_RIGHT_Y_CH ET08_CH2
#else
#define ET08_LEFT_X_CH ET08_CH1
#define ET08_LEFT_Y_CH ET08_CH2
#define ET08_RIGHT_X_CH ET08_CH4
#define ET08_RIGHT_Y_CH ET08_CH3
#endif

// ---------------- Gimbal motors ----------------
#define GM6020_SPEED_MAX 3600.0f
#define GM6020_SPEED_MIN (-GM6020_SPEED_MAX)
#define GM6020_SPEED_DEADZONE 30.0f

#define GIMBAL_RC_DEADZONE 50
#define GIMBAL_SPEED_SCALE 0.5f
#define GIMBAL_YAW_SWITCH_SPEED 1200.0f
#define GIMBAL_NO_FOLLOW_WZ_SCALE 4.5f
#define PITCH_SPEED_SCALE 0.15f
#define ET08_STICK_SCALE_DEN 660.0f

#define YAW_MOTOR_ID 1U   // CAN1
#define PITCH_MOTOR_ID 1U // CAN2

#define PITCH_GRAVITY_FF_K -6000.0f
#define PITCH_GRAVITY_FF_MAX 120000.0f
#define PITCH_GRAVITY_FF_OFFSET_DEG 0.0f

#define PITCH_HOLD_KP 12.0f
#define PITCH_HOLD_MAX_SPEED 1200.0f
#define PITCH_HOLD_DEADBAND_DEG 0.2f
#define PITCH_HOLD_KD 0.35f
#define PITCH_HOLD_KI 0.8f
#define PITCH_HOLD_I_LIMIT 800.0f
#define PITCH_FF_LPF 0.9f

// ---------------- Chassis (CAN1) ----------------
#define CHASSIS_MOTOR_COUNT 4U
#define CHASSIS_WHEEL_RADIUS 0.075f
#define CHASSIS_WHEEL_BASE 0.34f
#define CHASSIS_MAX_VEL 20.0f
#define CHASSIS_MAX_ROTATE 47.0f

#define M3508_SPEED_MAX 30000.0f
#define M3508_SPEED_MIN (-M3508_SPEED_MAX)

#define CHASSIS_SPEED_MULTIPLIER 1.5f

#define CHASSIS_DEADZONE_VX 0.15f
#define CHASSIS_DEADZONE_VY 0.15f
#define CHASSIS_DEADZONE_WZ 0.2f
#define CHASSIS_SPEED_DEADZONE 120.0f

#define MOTOR_STABILIZE_TIME_MS 2000U

#define MOTOR_FRONT_RIGHT 1U
#define MOTOR_FRONT_LEFT 2U
#define MOTOR_BACK_RIGHT 3U
#define MOTOR_BACK_LEFT 4U

#define SPEED_FILTER_COEF 0.4f

/* Private variables ---------------------------------------------------------*/
static ET08_Ctrl_t *et08_ctrl = NULL;
static USARTInstance *sbus_usart = NULL;
static uint32_t sbus_last_tick = 0U;
static uint32_t sbus_frame_count = 0U;
static uint32_t sbus_bad_count = 0U;

static DJIMotorInstance *friction_motors[FRICTION_MOTOR_COUNT] = {NULL};
static DJIMotorInstance *loader_motor = NULL;
static DJIMotorInstance *yaw_motor = NULL;
static DJIMotorInstance *pitch_motor = NULL;

static uint8_t friction_enabled = 0;
static uint8_t loader_enabled = 0;
static uint8_t loader_enabled_last = 0;
static uint8_t loader_continuous = 0;
static uint8_t sb_pos = ET08_SWITCH_POS_MID;
static uint8_t sb_last_pos = ET08_SWITCH_POS_MID;
static uint32_t sb_last_change_ms = 0U;
static uint8_t single_last_pos = ET08_SWITCH_POS_MID;
static uint32_t single_last_change_ms = 0U;
static uint8_t sb_down_armed = 1U;
static uint8_t pending_shots = 0;
static uint8_t loader_initialized = 0;
static uint8_t single_shot_active = 0;
static float single_shot_start_angle = 0.0f;
static float loader_target_angle = 0.0f;
static uint32_t last_shot_tick = 0;
static uint32_t last_step_tick = 0;
static uint8_t loader_pid_boosted = 0;
static float loader_speed_cmd = 0.0f;
static uint32_t loader_speed_tick = 0;
static uint32_t loader_continuous_start_ms = 0U;
static uint8_t loader_continuous_last = 0U;

static float yaw_speed_ref = 0.0f;
static float pitch_speed_ref = 0.0f;
static float pitch_hold_angle = 0.0f;
static uint8_t pitch_manual_active = 0;
static uint8_t pitch_manual_active_last = 0;
static float pitch_hold_i = 0.0f;
static float pitch_current_ff = 0.0f;
static uint32_t gimbal_last_tick = 0;

static DJIMotorInstance *chassis_motors[CHASSIS_MOTOR_COUNT] = {NULL};
static float chassis_vx = 0.0f;
static float chassis_vy = 0.0f;
static float chassis_wz = 0.0f;
static float wheel_speeds[CHASSIS_MOTOR_COUNT] = {0.0f};
static uint8_t motors_stabilized = 0;
static uint32_t system_start_time = 0;
static uint8_t zero_speed_control = 1;
static float filtered_vx = 0.0f;
static float filtered_vy = 0.0f;
static float filtered_wz = 0.0f;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Debug_DisableWatchdogs(void);
static void Et08SbusInit(UART_HandleTypeDef *uart_handle);
static uint8_t Et08InputValid(void);
static void EnsureFrictionMotorsReady(void);
static void EnsureLoaderMotorReady(void);
static void ApplyLoaderPidProfile(uint8_t single_shot_boost);
static void GimbalMotorsInit(void);
static float ClampFloat(float value, float min, float max);
static void SetAllMotorsZero(void);
static uint8_t Et08GetSaPosFromState(uint8_t state);
static uint8_t Et08GetSbPosFromState(uint8_t state);
static uint8_t Et08GetSdPosFromState(uint8_t state);
static uint8_t Et08MapUpperSwitchPos(uint8_t state);
static uint8_t Et08MapLowerSwitchPos(uint8_t state);
static uint8_t Et08MapSwitchStateRaw(uint16_t raw_value);

static uint8_t Et08MapUpperSwitchPos(uint8_t state)
{
    if (state == ET08_SWITCH_INVALID || state > 5U) {
        return ET08_SWITCH_INVALID;
    }
    return (state <= 2U) ? ET08_SWITCH_POS_UP : ET08_SWITCH_POS_DOWN;
}

static uint8_t Et08MapLowerSwitchPos(uint8_t state)
{
    if (state == ET08_SWITCH_INVALID || state > 5U) {
        return ET08_SWITCH_INVALID;
    }
    return (state <= 2U) ? state : (uint8_t)(state - 3U);
}

static uint8_t Et08GetSaPosFromState(uint8_t state)
{
    return Et08MapUpperSwitchPos(state);
}

static uint8_t Et08GetSbPosFromState(uint8_t state)
{
    return Et08MapLowerSwitchPos(state);
}

static uint8_t Et08GetSdPosFromState(uint8_t state)
{
    return Et08MapUpperSwitchPos(state);
}

static uint8_t Et08MapSwitchStateRaw(uint16_t raw_value)
{
    const int16_t levels[ET08_SWITCH_LEVEL_COUNT] = {
        ET08_SWITCH_LEVEL_0,
        ET08_SWITCH_LEVEL_1,
        ET08_SWITCH_LEVEL_2,
        ET08_SWITCH_LEVEL_3,
        ET08_SWITCH_LEVEL_4,
        ET08_SWITCH_LEVEL_5,
    };

    uint8_t best_index = ET08_SWITCH_INVALID;
    uint16_t best_diff = 0xFFFFu;

    for (uint8_t i = 0; i < ET08_SWITCH_LEVEL_COUNT; ++i) {
        uint16_t level_raw = (levels[i] < 0) ? (uint16_t)(levels[i] + ET08_CHANNEL_CENTER)
                                             : (uint16_t)levels[i];
        uint16_t diff = (raw_value > level_raw) ? (raw_value - level_raw) : (level_raw - raw_value);
        if (diff < best_diff) {
            best_diff = diff;
            best_index = i;
        }
    }

    if (best_diff <= SA_SB_RAW_TOLERANCE) {
        return best_index;
    }
    return ET08_SWITCH_INVALID;
}

static void Et08SbusInit(UART_HandleTypeDef *uart_handle)
{
    et08_ctrl = ET08_Init(uart_handle);
}

static uint8_t Et08InputValid(void)
{
    et08_ctrl = ET08_GetCtrl();
    if (et08_ctrl == NULL) {
        return 0U;
    }
    if (!ET08_IsOnline()) {
        return 0U;
    }
    if (et08_ctrl->frame_lost || et08_ctrl->failsafe) {
        return 0U;
    }
    return 1U;
}

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
            .can_handle = &hcan2,
            .tx_id = LOADER_CAN_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = LOADER_ANGLE_KP_BASE,
                .Ki = 0.0f,
                .Kd = LOADER_ANGLE_KD_BASE,
                .MaxOut = LOADER_ANGLE_MAXOUT_BASE,
                .IntegralLimit = 500.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
            },
            .speed_PID = {
                .Kp = LOADER_SPEED_KP_BASE,
                .Ki = 0.0f,
                .Kd = LOADER_SPEED_KD_BASE,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = LOADER_SPEED_MAXOUT_BASE,
            },
            .current_PID = {
                .Kp = 0.6f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = LOADER_CURRENT_MAXOUT_BASE,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };

    loader_motor = DJIMotorInit(&config);
    if (loader_motor != NULL) {
        loader_motor->angle_feedback_sign = LOADER_ANGLE_FEEDBACK_SIGN;
        loader_motor->angle_feedback_locked = 1U;
    }
}

static void ApplyLoaderPidProfile(uint8_t single_shot_boost)
{
    if (loader_motor == NULL) {
        return;
    }

    if (single_shot_boost && !loader_pid_boosted) {
        loader_motor->motor_controller.angle_PID.Kp = LOADER_ANGLE_KP_BOOST;
        loader_motor->motor_controller.angle_PID.Kd = LOADER_ANGLE_KD_BOOST;
        loader_motor->motor_controller.angle_PID.MaxOut = LOADER_ANGLE_MAXOUT_BOOST;

        loader_motor->motor_controller.speed_PID.Kp = LOADER_SPEED_KP_BOOST;
        loader_motor->motor_controller.speed_PID.Kd = LOADER_SPEED_KD_BOOST;
        loader_motor->motor_controller.speed_PID.MaxOut = LOADER_SPEED_MAXOUT_BOOST;

        loader_motor->motor_controller.current_PID.MaxOut = LOADER_CURRENT_MAXOUT_BOOST;
        loader_pid_boosted = 1;
    } else if (!single_shot_boost && loader_pid_boosted) {
        loader_motor->motor_controller.angle_PID.Kp = LOADER_ANGLE_KP_BASE;
        loader_motor->motor_controller.angle_PID.Kd = LOADER_ANGLE_KD_BASE;
        loader_motor->motor_controller.angle_PID.MaxOut = LOADER_ANGLE_MAXOUT_BASE;

        loader_motor->motor_controller.speed_PID.Kp = LOADER_SPEED_KP_BASE;
        loader_motor->motor_controller.speed_PID.Kd = LOADER_SPEED_KD_BASE;
        loader_motor->motor_controller.speed_PID.MaxOut = LOADER_SPEED_MAXOUT_BASE;

        loader_motor->motor_controller.current_PID.MaxOut = LOADER_CURRENT_MAXOUT_BASE;
        loader_pid_boosted = 0;
    }
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
    config.controller_param_init_config.current_feedforward_ptr = &pitch_current_ff;
    config.controller_setting_init_config.feedforward_flag = CURRENT_FEEDFORWARD;
    config.controller_param_init_config.speed_PID.MaxOut = 30000.0f;
    config.controller_param_init_config.speed_PID.Ki = 20.0f;
    config.can_init_config.can_handle = &hcan2;
    config.can_init_config.tx_id = PITCH_MOTOR_ID;
    pitch_motor = DJIMotorInit(&config);

    if (yaw_motor != NULL) {
        DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
        DJIMotorStop(yaw_motor);
    }

    if (pitch_motor != NULL) {
        DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
        DJIMotorStop(pitch_motor);
    }
}
static void ProcessShootRemote(void)
{
    if (!Et08InputValid()) {
        friction_enabled = 0;
        loader_enabled = 0;
        loader_continuous = 0;
        pending_shots = 0;
        return;
    }

    uint32_t now = HAL_GetTick();
    uint8_t sa_sb_state = et08_ctrl->switch_sa_sb_state;
    uint8_t raw_state = Et08MapSwitchStateRaw(et08_ctrl->switch_sa_sb_raw);
    if (raw_state != ET08_SWITCH_INVALID) {
        sa_sb_state = raw_state;
    }
    uint8_t sa_pos = Et08GetSaPosFromState(sa_sb_state);
    if (sa_pos == ET08_SWITCH_POS_UP) {
        friction_enabled = 1;
    } else if (sa_pos == ET08_SWITCH_POS_DOWN) {
        friction_enabled = 0;
    }

    sb_pos = Et08GetSbPosFromState(sa_sb_state);
    if (sb_pos == ET08_SWITCH_INVALID) {
        sb_pos = ET08_SWITCH_POS_MID;
    }

    loader_continuous = (friction_enabled && sb_pos == ET08_SWITCH_POS_UP) ? 1U : 0U;
    loader_enabled = friction_enabled && (sb_pos != ET08_SWITCH_POS_MID);

    if (!loader_enabled) {
        pending_shots = 0;
        sb_last_pos = sb_pos;
        loader_enabled_last = 0;
        sb_down_armed = 1U;
        return;
    }

    if (!loader_enabled_last && sb_pos == ET08_SWITCH_POS_DOWN) {
        pending_shots = 1;
        last_shot_tick = now;
        last_step_tick = last_shot_tick - SHOOT_INTERVAL_MS;
        sb_last_pos = sb_pos;
        sb_last_change_ms = now;
        loader_enabled_last = 1;
        sb_down_armed = 0U;
        return;
    }

    if (sb_pos != ET08_SWITCH_POS_DOWN) {
        sb_down_armed = 1U;
    }

    if (sb_pos != sb_last_pos) {
        if ((now - sb_last_change_ms) >= SB_SWITCH_DEBOUNCE_MS) {
            sb_last_change_ms = now;
            if (sb_pos == ET08_SWITCH_POS_DOWN && sb_down_armed) {
                pending_shots = 1;
                last_shot_tick = now;
                last_step_tick = last_shot_tick - SHOOT_INTERVAL_MS;
                sb_down_armed = 0U;
            } else if (sb_pos == ET08_SWITCH_POS_UP) {
                pending_shots = 0;
            }
            sb_last_pos = sb_pos;
        }
    }
    loader_enabled_last = 1;
}

static void ProcessGimbalControl(void)
{
    if (!Et08InputValid()) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        pitch_manual_active = 0;
        return;
    }

    int16_t pitch_in = et08_ctrl->right.y;
    uint8_t sd_pos = Et08GetSdPosFromState(et08_ctrl->switch_sd_sc_state);
    if (sd_pos == ET08_SWITCH_POS_UP) {
        yaw_speed_ref = 0.0f;
    } else {
        yaw_speed_ref = -chassis_wz * GIMBAL_NO_FOLLOW_WZ_SCALE;
    }

    pitch_speed_ref = 0.0f;
    pitch_manual_active = (abs(pitch_in) >= GIMBAL_RC_DEADZONE);
    if (pitch_manual_active) {
        float pitch_ratio = float_constrain((float)pitch_in / ET08_STICK_SCALE_DEN, -1.0f, 1.0f);
        pitch_speed_ref = pitch_ratio * GM6020_SPEED_MAX * PITCH_SPEED_SCALE;
        pitch_speed_ref = float_deadband(pitch_speed_ref, -GM6020_SPEED_DEADZONE, GM6020_SPEED_DEADZONE);
    }
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
    uint32_t now = HAL_GetTick();

    if (!loader_enabled || loader_motor == NULL) {
        if (loader_motor != NULL) {
            DJIMotorStop(loader_motor);
        }
        pending_shots = 0;
        loader_initialized = 0;
        single_shot_active = 0;
        loader_speed_cmd = 0.0f;
        loader_speed_tick = now;
        ApplyLoaderPidProfile(0);
        return;
    }
    if (loader_continuous) {
        pending_shots = 0;
        loader_initialized = 0;
        single_shot_active = 0;
        ApplyLoaderPidProfile(0);
        if (!loader_continuous_last) {
            loader_speed_cmd = 0.0f;
            loader_speed_tick = now;
        }
        float speed_target = ClampFloat(LOADER_CONTINUOUS_SPEED, LOADER_SPEED_MIN, LOADER_SPEED_MAX);
        if (loader_speed_tick == 0U) {
            loader_speed_tick = now;
            loader_speed_cmd = speed_target;
        } else {
            float dt_ms = (float)(now - loader_speed_tick);
            loader_speed_tick = now;
            float max_delta = LOADER_CONTINUOUS_SLEW_PER_MS * dt_ms;
            if (speed_target > loader_speed_cmd + max_delta) {
                loader_speed_cmd += max_delta;
            } else if (speed_target < loader_speed_cmd - max_delta) {
                loader_speed_cmd -= max_delta;
            } else {
                loader_speed_cmd = speed_target;
            }
        }
        DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
        DJIMotorEnable(loader_motor);
        DJIMotorSetRef(loader_motor, loader_speed_cmd);
        loader_continuous_last = 1;
        return;
    }

    loader_speed_cmd = 0.0f;
    loader_speed_tick = now;
    loader_continuous_last = 0;

    if (!loader_initialized) {
        loader_target_angle = ClampFloat(loader_motor->measure.total_angle, LOADER_ANGLE_MIN, LOADER_ANGLE_MAX);
        if (pending_shots > 0) {
            last_step_tick = now - SHOOT_INTERVAL_MS;
        } else {
            last_step_tick = now;
        }
        loader_initialized = 1;
    }

    if (pending_shots == 0 && sb_pos == ET08_SWITCH_POS_DOWN && sb_down_armed) {
        pending_shots = 1;
        last_shot_tick = now;
        last_step_tick = now - SHOOT_INTERVAL_MS;
        sb_down_armed = 0U;
    }

    if (pending_shots > 0 && !single_shot_active) {
        loader_target_angle += LOADER_SINGLE_DIRECTION_SIGN * LOADER_ANGLE_STEP_DEG;
        loader_target_angle = ClampFloat(loader_target_angle, LOADER_ANGLE_MIN, LOADER_ANGLE_MAX);
        pending_shots -= 1U;
        single_shot_active = 1;
        single_shot_start_angle = loader_motor->measure.total_angle;
        last_step_tick = now;
    } else if (pending_shots == 0 && !single_shot_active) {
        DJIMotorStop(loader_motor);
        ApplyLoaderPidProfile(0);
        return;
    }

    if (single_shot_active) {
        float delta = fabsf(loader_motor->measure.total_angle - single_shot_start_angle);
        if (delta >= LOADER_ANGLE_STEP_DEG) {
            single_shot_active = 0;
            DJIMotorStop(loader_motor);
            ApplyLoaderPidProfile(0);
            return;
        }
        ApplyLoaderPidProfile(1);
        DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
        DJIMotorEnable(loader_motor);
        DJIMotorSetRef(loader_motor, ClampFloat(LOADER_SINGLE_SPEED, LOADER_SPEED_MIN, LOADER_SPEED_MAX));
        return;
    }
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

static void ChassisMotorsInit(void)
{
    const uint8_t motor_ids[CHASSIS_MOTOR_COUNT] = {
        MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT, MOTOR_BACK_RIGHT, MOTOR_BACK_LEFT
    };

    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        Motor_Init_Config_s config = {
            .can_init_config = {
                .can_handle = &hcan1,
                .tx_id = motor_ids[i],
            },
            .controller_param_init_config = {
                .angle_PID = {
                    .Kp = 5.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .MaxOut = M3508_SPEED_MAX,
                    .IntegralLimit = 500.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                },
                .speed_PID = {
                    .Kp = 4.0f,
                    .Ki = 0.04f,
                    .Kd = 0.0f,
                    .IntegralLimit = 1000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .MaxOut = 10000.0f,
                },
                .current_PID = {
                    .Kp = 0.35f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .IntegralLimit = 1000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .MaxOut = 8000.0f,
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

        chassis_motors[i] = DJIMotorInit(&config);

        if (chassis_motors[i] != NULL) {
            DJIMotorOuterLoop(chassis_motors[i], SPEED_LOOP);
            DJIMotorStop(chassis_motors[i]);
        }
    }
}

static void ProcessChassisControl(void)
{
#if CHASSIS_FORCE_TEST
    zero_speed_control = 0;
    chassis_vx = CHASSIS_MAX_VEL * 0.3f;
    chassis_vy = 0.0f;
    chassis_wz = 0.0f;
    return;
#endif
    if (!Et08InputValid()) {
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        chassis_wz = 0.0f;
        zero_speed_control = 1;
        return;
    }

    uint8_t all_in_deadzone = (abs(et08_ctrl->left.x) < 50) &&
                              (abs(et08_ctrl->left.y) < 50) &&
                              (abs(et08_ctrl->right.x) < 50);

    if (all_in_deadzone) {
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        chassis_wz = 0.0f;
        zero_speed_control = 1;
        return;
    }

    zero_speed_control = 0;

    chassis_vx = et08_ctrl->left.x / ET08_STICK_SCALE_DEN * CHASSIS_MAX_VEL;
    chassis_vy = -et08_ctrl->left.y / ET08_STICK_SCALE_DEN * CHASSIS_MAX_VEL;
    chassis_wz = et08_ctrl->right.x / ET08_STICK_SCALE_DEN * CHASSIS_MAX_ROTATE;

    chassis_vx = float_deadband(chassis_vx, -CHASSIS_DEADZONE_VX, CHASSIS_DEADZONE_VX);
    chassis_vy = float_deadband(chassis_vy, -CHASSIS_DEADZONE_VY, CHASSIS_DEADZONE_VY);
    chassis_wz = float_deadband(chassis_wz, -CHASSIS_DEADZONE_WZ, CHASSIS_DEADZONE_WZ);
}

static void UpdateChassisKinematics(void)
{
    if (!motors_stabilized) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - system_start_time >= MOTOR_STABILIZE_TIME_MS) {
            for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                if (chassis_motors[i] != NULL) {
                    DJIMotorEnable(chassis_motors[i]);
                }
            }
            filtered_vx = 0.0f;
            filtered_vy = 0.0f;
            filtered_wz = 0.0f;
            motors_stabilized = 1;
        } else {
            SetAllMotorsZero();
            return;
        }
    }

    if (zero_speed_control) {
        SetAllMotorsZero();
        filtered_vx = 0.0f;
        filtered_vy = 0.0f;
        filtered_wz = 0.0f;
        return;
    }

    filtered_vx = filtered_vx * SPEED_FILTER_COEF + chassis_vx * (1.0f - SPEED_FILTER_COEF);
    filtered_vy = filtered_vy * SPEED_FILTER_COEF + chassis_vy * (1.0f - SPEED_FILTER_COEF);
    filtered_wz = filtered_wz * SPEED_FILTER_COEF + chassis_wz * (1.0f - SPEED_FILTER_COEF);

    float L = CHASSIS_WHEEL_BASE / 2.0f;

    float v1 = filtered_vy - filtered_vx - (L * filtered_wz);
    float v2 = filtered_vy + filtered_vx - (L * filtered_wz);
    float v3 = -filtered_vy + filtered_vx - (L * filtered_wz);
    float v4 = -filtered_vy - filtered_vx - (L * filtered_wz);

    wheel_speeds[0] = v1 / CHASSIS_WHEEL_RADIUS * CHASSIS_SPEED_MULTIPLIER;
    wheel_speeds[1] = v2 / CHASSIS_WHEEL_RADIUS * CHASSIS_SPEED_MULTIPLIER;
    wheel_speeds[2] = v3 / CHASSIS_WHEEL_RADIUS * CHASSIS_SPEED_MULTIPLIER;
    wheel_speeds[3] = v4 / CHASSIS_WHEEL_RADIUS * CHASSIS_SPEED_MULTIPLIER;

    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        wheel_speeds[i] = wheel_speeds[i] * 180.0f / PI;

        if (fabs(wheel_speeds[i]) < CHASSIS_SPEED_DEADZONE) {
            wheel_speeds[i] = 0.0f;
        }

        wheel_speeds[i] = float_constrain(wheel_speeds[i], M3508_SPEED_MIN, M3508_SPEED_MAX);

        if (chassis_motors[i] != NULL) {
            DJIMotorSetRef(chassis_motors[i], wheel_speeds[i]);
        }
    }
}

static void SetAllMotorsZero(void)
{
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (chassis_motors[i] != NULL) {
            DJIMotorSetRef(chassis_motors[i], 0.0f);
        }
    }
    chassis_vx = 0.0f;
    chassis_vy = 0.0f;
    chassis_wz = 0.0f;
}

static void ForceMotorZero(void)
{
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (chassis_motors[i] != NULL) {
            DJIMotorStop(chassis_motors[i]);
            DJIMotorSetRef(chassis_motors[i], 0.0f);
            HAL_Delay(10);
            DJIMotorEnable(chassis_motors[i]);
            DJIMotorSetRef(chassis_motors[i], 0.0f);
        }
    }

    chassis_vx = 0.0f;
    chassis_vy = 0.0f;
    chassis_wz = 0.0f;
    zero_speed_control = 1;

    filtered_vx = 0.0f;
    filtered_vy = 0.0f;
    filtered_wz = 0.0f;
}
int main(void)
{
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

    ChassisMotorsInit();
    ForceMotorZero();
#if !DEMO_CHASSIS_ONLY
    GimbalMotorsInit();
#endif

    Et08SbusInit(&huart3);
    system_start_time = HAL_GetTick();

    LOGINFO("[inf_omni] demo initialized (frames=%lu bad=%lu)",
            (unsigned long)sbus_frame_count,
            (unsigned long)sbus_bad_count);

    uint32_t last_update_tick = 0;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

    while (1) {
        DaemonTask();
        DJIMotorControl();

        uint32_t now = HAL_GetTick();
        if (now - last_update_tick >= UPDATE_INTERVAL_MS) {
            last_update_tick = now;

#if !DEMO_CHASSIS_ONLY
            ProcessShootRemote();
#endif
            ProcessChassisControl();
            UpdateChassisKinematics();
#if !DEMO_CHASSIS_ONLY
            ProcessGimbalControl();
            UpdateFrictionControl();
            UpdateLoaderControl();
            UpdateGimbalControl();
#endif
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

















