/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Wheel-leg all-in-one test (balance + gimbal + shooter)
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"

#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_init.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dji_motor.h"
#include "dm_imu.h"
#include "dmmotor.h"
#include "et08_remote.h"
#include "user_lib.h"
#include "utils.h"

#define CONTROL_INTERVAL_MS 5U
#define MIT_SEND_INTERVAL_MS 2U
#define ENABLE_INTERVAL_MS 100U
#define TELEMETRY_INTERVAL_MS 50U
#define SHOOTER_UPDATE_INTERVAL_MS 20U

#define IMU_REQUEST_INTERVAL_MS 20U
#define IMU_TIMEOUT_MS 200U

#define IMU_ID 0x01
#define IMU_MASTER_ID 0x00
#define IMU_RX_ID (IMU_MASTER_ID)
#define IMU_VALID_ACCEL (1U << 0)
#define IMU_VALID_GYRO (1U << 1)
#define IMU_VALID_EULER (1U << 2)

#define FRONT_LEFT_ID 1U
#define REAR_LEFT_ID 2U
#define FRONT_RIGHT_ID 3U
#define REAR_RIGHT_ID 4U
#define DM_MASTER_ID 0x00U

#define DM_P_RANGE 12.5f
#define DM_V_RANGE 45.0f
#define DM_T_RANGE 54.0f

/* Stand pose (rad) from loaded stable stance.
 * Update these from "joint pos" logs when the robot is stable on the ground. */

// 电机逆时针转为正方向
#define HIP_TARGET_L -0.10f // 下摆 顺转
#define KNEE_TARGET_L 1.35f // 收起顺转 伸展逆转

#define HIP_TARGET_R -0.25f // 下摆 逆转
#define KNEE_TARGET_R -0.65f //收起逆转 伸展顺转

/* Joint stiffness (per motor).
 * Note: MIT KD is clamped by the driver (<= 5.0). Keep KD in [0..5]. */
//left
#define FL_KP 50.0f
#define FL_KD 1.5f
#define RL_KP 60.0f
#define RL_KD 2.5f

//right
#define FR_KP 50.0f
#define FR_KD 1.5f
#define RR_KP 60.0f
#define RR_KD 1.5f

/* Torque feedforward (TFF) to fight gravity.
 * Use the sign/magnitude from "tq[]" logs at stable stance, then tune in small steps. */
#define HIP_TFF_L -6.0f
#define KNEE_TFF_L 8.0f

#define HIP_TFF_R 6.0f
#define KNEE_TFF_R -8.0f
/* Hip TFF sharing between the two motors on the same leg.
 * rear motor also contributes to hip due to series transmission (rear ~= hip+knee),
 * so put part of hip TFF on rear to reduce "motor fighting" / backlash buzz. */
#define HIP_TFF_SHARE 0.5f /* 0..1: front gets share, rear gets (1-share) */

/* Balance controller (deg-based) */
#define BALANCE_PITCH_KP 950.0f
#define BALANCE_PITCH_KD 70.0f
#define BALANCE_PITCH_KI 0.00f
#define BALANCE_PITCH_I_LIMIT 50.0f
#define BALANCE_OUT_MAX 20000.0f
#define BALANCE_TILT_MAX_DEG 40.0f
#define BALANCE_PITCH_SIGN 1.0f
#define BALANCE_REF_FIXED_DEG -4.70f

/* Wheel command limits */
#define WHEEL_SPEED_MAX 25000.0f
#define WHEEL_SPEED_MIN (-WHEEL_SPEED_MAX)

/* RC mapping */
#define RC_STICK_MAX 660.0f
#define RC_DEADZONE 30
#define RC_FWD_MAX WHEEL_SPEED_MAX
#define RC_YAW_MAX WHEEL_SPEED_MAX

// ---------------- Shooter motors ----------------
#define FRICTION_MOTOR_COUNT 2U
static const uint8_t FRICTION_CAN_IDS[FRICTION_MOTOR_COUNT] = {1U, 2U}; // CAN2
#define LOADER_CAN_ID 5U                                              // CAN1

#define FRICTION_SPEED_MAX 30000.0f // deg/s
#define FRICTION_SPEED_MIN (-FRICTION_SPEED_MAX)
// Overall direction sign for both friction wheels (keep opposite rotation via motor_reverse_flag).
#define FRICTION_DIRECTION_SIGN (1.0f)
#define FRICTION_SPEED_TARGET (FRICTION_DIRECTION_SIGN * 30000.0f)

#define LOADER_SPEED_MAX 20000.0f
#define LOADER_SPEED_MIN (-LOADER_SPEED_MAX)
#define LOADER_ANGLE_MAX 36000.0f
#define LOADER_ANGLE_MIN (-LOADER_ANGLE_MAX)

// 拨弹使用 M2006：减速比按 1:13（与 wheelleg_shoot_test 一致，按实际机构调整）
#define LOADER_GEAR_RATIO 13.0f
#define LOADER_OUTPUT_STEP_DEG 45.0f
// Loader direction sign for both angle-step and continuous mode.
#define LOADER_DIRECTION_SIGN (1.0f)
#define LOADER_ANGLE_STEP_DEG (LOADER_DIRECTION_SIGN * LOADER_OUTPUT_STEP_DEG * LOADER_GEAR_RATIO)

#define SHOOT_INTERVAL_MS 1000U
#define LOADER_CONTINUOUS_SPEED (LOADER_DIRECTION_SIGN * 15000.0f)

// Loader PID (tune here)
// - Angle loop output unit is (deg/s) as speed reference for speed loop.
// - Speed loop output is current command.
#define LOADER_ANGLE_PID_KP 8.0f
#define LOADER_ANGLE_PID_KI 0.0f
#define LOADER_ANGLE_PID_KD 0.0f
#define LOADER_ANGLE_PID_MAXOUT 6000.0f
#define LOADER_ANGLE_PID_ILIMIT 500.0f

#define LOADER_SPEED_PID_KP 12.0f
#define LOADER_SPEED_PID_KI 0.0f
#define LOADER_SPEED_PID_KD 0.0f
#define LOADER_SPEED_PID_MAXOUT 10000.0f
#define LOADER_SPEED_PID_ILIMIT 3000.0f

#define LOADER_CURRENT_PID_KP 0.6f
#define LOADER_CURRENT_PID_KI 0.0f
#define LOADER_CURRENT_PID_KD 0.0f
#define LOADER_CURRENT_PID_MAXOUT 10000.0f
#define LOADER_CURRENT_PID_ILIMIT 3000.0f

// ---------------- ET08 mapping (tunable) ----------------
// CH5 switch group (SA/SB): friction enable if state in [0,2]
#define ET08_FRICTION_ON_STATE_MIN 0U
#define ET08_FRICTION_ON_STATE_MAX 2U

// CH6 (SD/SC): raw clusters for SC positions
#define ET08_SC_RAW_TOP 1493U
#define ET08_SC_RAW_MID 1024U
#define ET08_SC_RAW_BOTTOM 554U
#define ET08_SC_RAW_TOLERANCE 200U

// KnobRight acts like "dial": negative direction = trigger
#define ET08_DIAL_ON_THRESHOLD (-200)
#define ET08_DIAL_OFF_THRESHOLD (-100)

// ---------------- Gimbal motors ----------------
#define GM6020_SPEED_MAX 3600.0f
#define GM6020_SPEED_MIN (-GM6020_SPEED_MAX)
#define GM6020_ANGLE_MAX 1440.0f
#define GM6020_ANGLE_MIN (-GM6020_ANGLE_MAX)
#define GM6020_SPEED_DEADZONE 30.0f

#define GIMBAL_RC_DEADZONE 80
#define GIMBAL_MANUAL_ON 100
#define GIMBAL_MANUAL_OFF 50
#define YAW_SPEED_SCALE 0.80f
#define PITCH_SPEED_SCALE 0.50f
#define ET08_STICK_SCALE_DEN 660.0f // ET08 centered stick typically ~[-660,660], adjust if needed.

#define YAW_MOTOR_ID 5U   // CAN1
#define PITCH_MOTOR_ID 1U // CAN2
#define YAW_DIRECTION_SIGN (-1.0f)
#define PITCH_DIRECTION_SIGN (-1.0f)
#define YAW_REF_LPF_ALPHA 0.50f
#define YAW_SPEED_DEADZONE 20.0f
#define YAW_SPEED_MAX GM6020_SPEED_MAX
#define YAW_SPEED_MIN (-YAW_SPEED_MAX)
#define PITCH_SPEED_MAX_UP 3000.0f
#define PITCH_SPEED_MAX_DOWN 2500.0f

#define PITCH_RANGE_UP_DEG 25.0f
#define PITCH_RANGE_DOWN_DEG 35.0f
#define PITCH_LIMIT_MARGIN_DEG 1.0f

// Pitch gravity FF + release hold (copied from wheelleg_shoot_test defaults; tune on real gimbal)
#define PITCH_FF_ENABLE 0
#define PITCH_GRAVITY_FF_SIGN 1.0f
#define PITCH_HOLD_KNOB_ON_THRESHOLD (-200)
#define PITCH_HOLD_KNOB_OFF_THRESHOLD (200)
#define PITCH_GRAVITY_FF_K 10000.0f
#define PITCH_GRAVITY_FF_MAX 25000.0f
#define PITCH_GRAVITY_FF_OFFSET_DEG 0.0f

#define YAW_HOLD_KP 5.0f
#define YAW_HOLD_MAX_SPEED 1000.0f
#define YAW_HOLD_DEADBAND_DEG 0.8f
#define YAW_HOLD_KD 0.4f
#define YAW_HOLD_KI 0.0f
#define YAW_HOLD_I_LIMIT 0.0f
#define YAW_HOLD_LPF_ALPHA 0.85f
#define YAW_HOLD_STATIC_COMP 80.0f

#define PITCH_HOLD_KP 10.0f
#define PITCH_HOLD_MAX_SPEED 1500.0f
#define PITCH_HOLD_DEADBAND_DEG 0.6f
#define PITCH_HOLD_KD 0.4f
#define PITCH_HOLD_KI 0.05f
#define PITCH_HOLD_I_LIMIT 200.0f
#define PITCH_HOLD_LPF_ALPHA 0.85f
#define PITCH_HOLD_STATIC_COMP 100.0f
#define PITCH_FF_LPF 0.9f
#define PITCH_ANGLE_PID_KP 15.0f
#define PITCH_ANGLE_PID_MAXOUT 1500.0f
#define PITCH_SPEED_PID_KP 50.0f
#define PITCH_SPEED_PID_KI 350.0f
#define PITCH_SPEED_PID_KD 0.15f
#define PITCH_SPEED_PID_ILIMIT 2500.0f
#define PITCH_SPEED_PID_MAXOUT 20000.0f

typedef struct
{
    DMMotor_Handle *handle;
    float target_p;
    float kp;
    float kd;
    float t_ff;
} JointMotor;

typedef enum
{
    BALANCE_MODE_DISABLE = 0,
    BALANCE_MODE_ACTIVE = 1,
} BalanceMode;

static USARTInstance *telemetry_usart = NULL;
static ET08_Ctrl_t *et08_ctrl = NULL;
static dm_imu_t dm_imu;

static DJIMotorInstance *wheel_left = NULL;
static DJIMotorInstance *wheel_right = NULL;

static JointMotor front_left;
static JointMotor rear_left;
static JointMotor front_right;
static JointMotor rear_right;

static DJIMotorInstance *friction_motors[FRICTION_MOTOR_COUNT] = {NULL};
static DJIMotorInstance *loader_motor = NULL;
static DJIMotorInstance *yaw_motor = NULL;
static DJIMotorInstance *pitch_motor = NULL;

static float pitch_deg = 0.0f;
static float pitch_rate_deg_s = 0.0f;
static uint32_t imu_last_ms = 0;
static uint8_t imu_valid = 0;

static float pitch_ref_deg = BALANCE_REF_FIXED_DEG;
static float balance_i = 0.0f;
static uint8_t balance_active = 0;
static float balance_err_deg = 0.0f;
static float balance_out = 0.0f;
static float forward_cmd = 0.0f;
static float yaw_cmd = 0.0f;
static uint8_t imu_alive = 0;

static float wheel_left_ref = 0.0f;
static float wheel_right_ref = 0.0f;
static uint8_t wheel_enable = 0;

static BalanceMode balance_mode = BALANCE_MODE_DISABLE;
static uint8_t joint_enable = 0;
static uint32_t last_enable_tick = 0;

static uint8_t friction_enabled = 0;
static uint8_t fire_mode = 0; // 0: single, 1: double, 2: continuous

static uint8_t pending_shots = 0;
static uint8_t dial_active = 0;
static uint8_t dial_last = 0;
static uint8_t loader_initialized = 0;
static float loader_target_angle = 0.0f;
static uint32_t last_shot_tick = 0;
static uint32_t last_step_tick = 0;
static uint8_t loader_cont_initialized = 0;
static float loader_cont_target_angle = 0.0f;
static uint32_t loader_cont_last_tick = 0;

static float yaw_speed_ref = 0.0f;
static float pitch_speed_ref = 0.0f;
static float pitch_current_ff = 0.0f;
static uint8_t pitch_manual_active = 0;
static uint8_t yaw_manual_active = 0;
static float yaw_hold_angle = 0.0f;
static float pitch_hold_angle = 0.0f;
static uint8_t yaw_hold_inited = 0;
static uint8_t pitch_hold_inited = 0;
static float pitch_center_angle = 0.0f;
static uint8_t pitch_center_inited = 0;
static uint8_t pitch_hold_enabled = 1;
static uint8_t pitch_hold_enabled_last = 1;
static uint32_t gimbal_last_tick = 0;

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
    USARTSend(telemetry_usart, (uint8_t *)msg, (uint16_t)strlen(msg), USART_TRANSFER_BLOCKING);
}

static float ClampFloat(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static int16_t ApplyDeadzone(int16_t value, int16_t deadzone)
{
    if (value > deadzone)
        return value - deadzone;
    if (value < -deadzone)
        return value + deadzone;
    return 0;
}

static uint16_t SwitchLevelToRaw(int16_t level)
{
    if (level < 0)
        return (uint16_t)(level + ET08_CHANNEL_CENTER);
    return (uint16_t)level;
}

static float WrapAngle180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float CalcTargetTotalAngle(float target_single, float current_single, float current_total)
{
    float delta = WrapAngle180(target_single - current_single);
    return current_total + delta;
}

static float PitchDeltaFromCenter(float angle)
{
    return WrapAngle180(angle - pitch_center_angle);
}

static float ClampPitchTarget(float target)
{
    float delta = PitchDeltaFromCenter(target);
    delta = ClampFloat(delta, -PITCH_RANGE_DOWN_DEG, PITCH_RANGE_UP_DEG);
    return WrapAngle180(pitch_center_angle + delta);
}

static void ResetPidState(PIDInstance *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->ITerm = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
    pid->dt = 0.0f;
    pid->DWT_CNT = 0U;
}

static void ResetPitchPidRuntime(void)
{
    if (pitch_motor == NULL) {
        return;
    }
    ResetPidState(&pitch_motor->motor_controller.angle_PID);
    ResetPidState(&pitch_motor->motor_controller.speed_PID);
    ResetPidState(&pitch_motor->motor_controller.current_PID);
}

static uint8_t GimbalFeedbackReady(const DJIMotorInstance *motor)
{
    return (motor != NULL && motor->feed_cnt != 0U);
}

static uint8_t MapSwitchClosest(uint16_t raw_value)
{
    const int16_t levels[ET08_SWITCH_LEVEL_COUNT] = {
        ET08_SWITCH_LEVEL_0,
        ET08_SWITCH_LEVEL_1,
        ET08_SWITCH_LEVEL_2,
        ET08_SWITCH_LEVEL_3,
        ET08_SWITCH_LEVEL_4,
        ET08_SWITCH_LEVEL_5,
    };

    uint8_t best_index = 0U;
    uint16_t best_diff = 0xFFFFU;

    for (uint8_t i = 0; i < ET08_SWITCH_LEVEL_COUNT; ++i)
    {
        uint16_t level_raw = SwitchLevelToRaw(levels[i]);
        uint16_t diff = (raw_value > level_raw) ? (raw_value - level_raw) : (level_raw - raw_value);
        if (diff < best_diff)
        {
            best_diff = diff;
            best_index = i;
        }
    }

    return best_index;
}

static void WheelMotorsInit(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
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
                .MaxOut = 20000.0f,
            },
            .current_PID = {
                .Kp = 0.4f,
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

    wheel_left = DJIMotorInit(&config);
    LOGINFO("[wl_all] wheel left id 1 ready");

    config.can_init_config.tx_id = 2;
    wheel_right = DJIMotorInit(&config);
    LOGINFO("[wl_all] wheel right id 2 ready");
}

static void WheelMotorsUpdate(void)
{
    if (wheel_left)
    {
        DJIMotorOuterLoop(wheel_left, SPEED_LOOP);
        if (wheel_enable)
            DJIMotorEnable(wheel_left);
        else
            DJIMotorStop(wheel_left);
        DJIMotorSetRef(wheel_left, wheel_left_ref);
    }
    if (wheel_right)
    {
        DJIMotorOuterLoop(wheel_right, SPEED_LOOP);
        if (wheel_enable)
            DJIMotorEnable(wheel_right);
        else
            DJIMotorStop(wheel_right);
        DJIMotorSetRef(wheel_right, wheel_right_ref);
    }
}

static void JointMotorInit(JointMotor *joint, uint8_t motor_id, float kp, float kd, float t_ff, float target_p)
{
    if (!joint)
        return;

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

    joint->handle = DMMotor_Init(&config);
    joint->target_p = ClampFloat(target_p, -DM_P_RANGE, DM_P_RANGE);
    joint->kp = kp;
    joint->kd = kd;
    joint->t_ff = t_ff;
}

static void JointMotorsInit(void)
{
    const float hip_front_tff_l = HIP_TFF_L * HIP_TFF_SHARE;
    const float hip_rear_tff_l = HIP_TFF_L * (1.0f - HIP_TFF_SHARE);
    const float hip_front_tff_r = HIP_TFF_R * HIP_TFF_SHARE;
    const float hip_rear_tff_r = HIP_TFF_R * (1.0f - HIP_TFF_SHARE);

    JointMotorInit(&front_left, FRONT_LEFT_ID, FL_KP, FL_KD, hip_front_tff_l, HIP_TARGET_L);
    JointMotorInit(&front_right, FRONT_RIGHT_ID, FR_KP, FR_KD, hip_front_tff_r, HIP_TARGET_R);

    JointMotorInit(&rear_left, REAR_LEFT_ID, RL_KP, RL_KD, KNEE_TFF_L + hip_rear_tff_l,
                   HIP_TARGET_L + KNEE_TARGET_L);
    JointMotorInit(&rear_right, REAR_RIGHT_ID, RR_KP, RR_KD, KNEE_TFF_R + hip_rear_tff_r,
                   HIP_TARGET_R + KNEE_TARGET_R);
}

static void JointMotorsEnable(void)
{
    if (front_left.handle) {
        DMMotor_ClearError(front_left.handle, DM_MODE_MIT);
        DMMotor_Enable(front_left.handle, DM_MODE_MIT);
    }
    if (rear_left.handle) {
        DMMotor_ClearError(rear_left.handle, DM_MODE_MIT);
        DMMotor_Enable(rear_left.handle, DM_MODE_MIT);
    }
    if (front_right.handle) {
        DMMotor_ClearError(front_right.handle, DM_MODE_MIT);
        DMMotor_Enable(front_right.handle, DM_MODE_MIT);
    }
    if (rear_right.handle) {
        DMMotor_ClearError(rear_right.handle, DM_MODE_MIT);
        DMMotor_Enable(rear_right.handle, DM_MODE_MIT);
    }
}

static void JointMotorsDisable(void)
{
    if (front_left.handle)
        DMMotor_Disable(front_left.handle, DM_MODE_MIT);
    if (rear_left.handle)
        DMMotor_Disable(rear_left.handle, DM_MODE_MIT);
    if (front_right.handle)
        DMMotor_Disable(front_right.handle, DM_MODE_MIT);
    if (rear_right.handle)
        DMMotor_Disable(rear_right.handle, DM_MODE_MIT);
}

static void JointMotorsSendMIT(void)
{
    if (front_left.handle)
        DMMotor_SendMIT(front_left.handle, front_left.target_p, 0.0f, front_left.kp, front_left.kd, front_left.t_ff);
    if (rear_left.handle)
        DMMotor_SendMIT(rear_left.handle, rear_left.target_p, 0.0f, rear_left.kp, rear_left.kd, rear_left.t_ff);
    if (front_right.handle)
        DMMotor_SendMIT(front_right.handle, front_right.target_p, 0.0f, front_right.kp, front_right.kd, front_right.t_ff);
    if (rear_right.handle)
        DMMotor_SendMIT(rear_right.handle, rear_right.target_p, 0.0f, rear_right.kp, rear_right.kd, rear_right.t_ff);
}

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
}

static void ImuUpdate(void)
{
    dm_imu_data_t data;
    if (!dm_imu_get_data(&dm_imu, &data))
        return;
    if ((data.valid_mask & IMU_VALID_EULER) == 0)
        return;

    pitch_deg = data.euler[1];
    if (data.valid_mask & IMU_VALID_GYRO)
    {
        /* DM-IMU gyro range is ~[-34.88, 34.88]; treat it as rad/s and convert to deg/s. */
        pitch_rate_deg_s = data.gyro[1] * 57.2957795f;
    }
    else
    {
        pitch_rate_deg_s = 0.0f;
    }
    imu_last_ms = HAL_GetTick();
    imu_valid = 1;
}

static uint8_t ET08_IsFrictionEnabled(const ET08_Ctrl_t *rc)
{
    if (rc == NULL)
        return 0;

    if (rc->switch_sa_sb_state != 0xFFu) {
        return (rc->switch_sa_sb_state >= ET08_FRICTION_ON_STATE_MIN &&
                rc->switch_sa_sb_state <= ET08_FRICTION_ON_STATE_MAX);
    }

    return (rc->switch_sa_sb_centered > 0);
}

static uint8_t ET08_MapFireModeSB(const ET08_Ctrl_t *rc, uint8_t last_mode)
{
    if (rc == NULL)
        return 0;

    uint8_t st = rc->switch_sa_sb_state;
    if (st == 0xFFu) {
        st = MapSwitchClosest(rc->switch_sa_sb_raw);
    }

    return (uint8_t)(st % 3U);
}

static uint8_t ET08_IsJointEnabled(const ET08_Ctrl_t *rc)
{
    if (rc == NULL)
        return 0;

    if (rc->switch_sd_sc_state != 0xFFu)
        return (rc->switch_sd_sc_state <= 2U) ? 1U : 0U;

    return (rc->switch_sd_sc_centered > 0) ? 1U : 0U;
}

static void UpdateControl(float dt_sec)
{
    (void)dt_sec;

    if (!ET08_IsOnline() || et08_ctrl == NULL) {
        balance_mode = BALANCE_MODE_DISABLE;
        joint_enable = 0;
        wheel_enable = 0;
        wheel_left_ref = 0.0f;
        wheel_right_ref = 0.0f;
        balance_active = 0;
        balance_i = 0.0f;
        balance_out = 0.0f;
        forward_cmd = 0.0f;
        yaw_cmd = 0.0f;
        return;
    }

    balance_mode = BALANCE_MODE_DISABLE;
    joint_enable = ET08_IsJointEnabled(et08_ctrl);
    wheel_enable = 1;

    int16_t left_y = ApplyDeadzone(et08_ctrl->left.y, RC_DEADZONE);
    int16_t left_x = ApplyDeadzone(et08_ctrl->left.x, RC_DEADZONE);

    forward_cmd = (float)left_y / RC_STICK_MAX * RC_FWD_MAX;
    yaw_cmd = (float)left_x / RC_STICK_MAX * RC_YAW_MAX;

    balance_active = 0;
    balance_i = 0.0f;
    balance_out = 0.0f;

    float base_speed = forward_cmd;
    /* Left wheel is mirrored (command sign inverted). Use differential model in robot frame. */
    wheel_left_ref = -(base_speed - yaw_cmd);
    wheel_right_ref = (base_speed + yaw_cmd);

    wheel_left_ref = ClampFloat(wheel_left_ref, WHEEL_SPEED_MIN, WHEEL_SPEED_MAX);
    wheel_right_ref = ClampFloat(wheel_right_ref, WHEEL_SPEED_MIN, WHEEL_SPEED_MAX);
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
                .Kp = LOADER_ANGLE_PID_KP,
                .Ki = LOADER_ANGLE_PID_KI,
                .Kd = LOADER_ANGLE_PID_KD,
                .MaxOut = LOADER_ANGLE_PID_MAXOUT,
                .IntegralLimit = LOADER_ANGLE_PID_ILIMIT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
            },
            .speed_PID = {
                .Kp = LOADER_SPEED_PID_KP,
                .Ki = LOADER_SPEED_PID_KI,
                .Kd = LOADER_SPEED_PID_KD,
                .IntegralLimit = LOADER_SPEED_PID_ILIMIT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = LOADER_SPEED_PID_MAXOUT,
            },
            .current_PID = {
                .Kp = LOADER_CURRENT_PID_KP,
                .Ki = LOADER_CURRENT_PID_KI,
                .Kd = LOADER_CURRENT_PID_KD,
                .IntegralLimit = LOADER_CURRENT_PID_ILIMIT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = LOADER_CURRENT_PID_MAXOUT,
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
                .Kp = 50.0f,
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
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };

    yaw_motor = DJIMotorInit(&config);
    LOGINFO("[wl_all] Yaw GM6020 registered on CAN1 id %u", (unsigned)YAW_MOTOR_ID);
    if (yaw_motor && yaw_motor->daemon) {
        (void)DaemonSetTimeout(yaw_motor->daemon, 500U);
    }

    config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    config.controller_param_init_config.current_feedforward_ptr = &pitch_current_ff;
    config.controller_setting_init_config.feedforward_flag =
        PITCH_FF_ENABLE ? CURRENT_FEEDFORWARD : FEEDFORWARD_NONE;
    config.controller_param_init_config.angle_PID.Kp = PITCH_ANGLE_PID_KP;
    config.controller_param_init_config.angle_PID.MaxOut = PITCH_ANGLE_PID_MAXOUT;
    config.controller_param_init_config.speed_PID.MaxOut = PITCH_SPEED_PID_MAXOUT;
    config.controller_param_init_config.speed_PID.Kp = PITCH_SPEED_PID_KP;
    config.controller_param_init_config.speed_PID.Ki = PITCH_SPEED_PID_KI;
    config.controller_param_init_config.speed_PID.Kd = PITCH_SPEED_PID_KD;
    config.controller_param_init_config.speed_PID.IntegralLimit = PITCH_SPEED_PID_ILIMIT;
    config.can_init_config.can_handle = &hcan2;
    config.can_init_config.tx_id = PITCH_MOTOR_ID;
    pitch_motor = DJIMotorInit(&config);
    LOGINFO("[wl_all] Pitch GM6020 registered on CAN2 id %u", (unsigned)PITCH_MOTOR_ID);
    if (pitch_motor && pitch_motor->daemon) {
        (void)DaemonSetTimeout(pitch_motor->daemon, 500U);
    }

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
    if (et08_ctrl == NULL || !ET08_IsOnline()) {
        friction_enabled = 0;
        dial_active = 0;
        pending_shots = 0;
        return;
    }

    const ET08_Ctrl_t *rc = et08_ctrl;

    friction_enabled = ET08_IsFrictionEnabled(rc);
    fire_mode = ET08_MapFireModeSB(rc, fire_mode);

    if (!friction_enabled) {
        dial_active = 0;
        pending_shots = 0;
        return;
    }

    int16_t dial = rc->knob_right;
    if (!dial_active && dial < ET08_DIAL_ON_THRESHOLD) {
        dial_active = 1;
    } else if (dial_active && dial > ET08_DIAL_OFF_THRESHOLD) {
        dial_active = 0;
    }

    uint8_t dial_edge = (dial_active && !dial_last);
    dial_last = dial_active;

    if (fire_mode == 2) {
        pending_shots = 0;
        return;
    }

    if (!dial_active) {
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
    if (et08_ctrl == NULL || !ET08_IsOnline()) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        pitch_manual_active = 0;
        yaw_manual_active = 0;
        pitch_hold_enabled = 1;
        if (pitch_hold_enabled != pitch_hold_enabled_last) {
            pitch_hold_inited = 0;
            pitch_center_inited = 0;
            pitch_hold_enabled_last = pitch_hold_enabled;
            ResetPitchPidRuntime();
        }
        return;
    }

    const ET08_Ctrl_t *rc = et08_ctrl;

    int16_t yaw_in = ApplyDeadzone(rc->right.x, GIMBAL_RC_DEADZONE);
    int16_t pitch_in = ApplyDeadzone(rc->right.y, GIMBAL_RC_DEADZONE);
    int16_t hold_knob = rc->knob_left;

    float yaw_ratio = float_constrain((float)yaw_in / ET08_STICK_SCALE_DEN, -1.0f, 1.0f);
    float pitch_ratio = float_constrain((float)pitch_in / ET08_STICK_SCALE_DEN, -1.0f, 1.0f);

    float yaw_cmd = YAW_DIRECTION_SIGN * yaw_ratio * YAW_SPEED_MAX * YAW_SPEED_SCALE;
    float pitch_cmd = PITCH_DIRECTION_SIGN * (-pitch_ratio) * PITCH_SPEED_MAX_UP * PITCH_SPEED_SCALE;

    yaw_cmd = float_deadband(yaw_cmd, -YAW_SPEED_DEADZONE, YAW_SPEED_DEADZONE);
    pitch_cmd = float_deadband(pitch_cmd, -GM6020_SPEED_DEADZONE, GM6020_SPEED_DEADZONE);

    if (yaw_manual_active) {
        if (abs(yaw_in) <= GIMBAL_MANUAL_OFF) {
            yaw_manual_active = 0;
        }
    } else if (abs(yaw_in) >= GIMBAL_MANUAL_ON) {
        yaw_manual_active = 1;
    }

    if (pitch_manual_active) {
        if (abs(pitch_in) <= GIMBAL_MANUAL_OFF) {
            pitch_manual_active = 0;
        }
    } else if (abs(pitch_in) >= GIMBAL_MANUAL_ON) {
        pitch_manual_active = 1;
    }

    if (pitch_hold_enabled) {
        if (hold_knob > PITCH_HOLD_KNOB_OFF_THRESHOLD) {
            pitch_hold_enabled = 0;
        }
    } else {
        if (hold_knob < PITCH_HOLD_KNOB_ON_THRESHOLD) {
            pitch_hold_enabled = 1;
        }
    }
    if (pitch_hold_enabled != pitch_hold_enabled_last) {
        pitch_hold_inited = 0;
        pitch_center_inited = 0;
        pitch_hold_enabled_last = pitch_hold_enabled;
        ResetPitchPidRuntime();
    }

    if (yaw_manual_active) {
        yaw_speed_ref = yaw_speed_ref * YAW_REF_LPF_ALPHA + yaw_cmd * (1.0f - YAW_REF_LPF_ALPHA);
    } else {
        yaw_speed_ref = 0.0f;
    }

    if (pitch_manual_active) {
        pitch_speed_ref = pitch_cmd;
    } else {
        pitch_speed_ref = 0.0f;
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

    if (!friction_enabled || loader_motor == NULL) {
        if (loader_motor != NULL) {
            DJIMotorStop(loader_motor);
        }
        pending_shots = 0;
        loader_initialized = 0;
        loader_cont_initialized = 0;
        return;
    }

    if (!dial_active) {
        DJIMotorStop(loader_motor);
        pending_shots = 0;
        loader_initialized = 0;
        loader_cont_initialized = 0;
        return;
    }

    uint32_t now = HAL_GetTick();
    if (fire_mode == 2) {
        pending_shots = 0;
        loader_initialized = 0;
        loader_cont_initialized = 0;
        float speed_ref = ClampFloat(LOADER_CONTINUOUS_SPEED, LOADER_SPEED_MIN, LOADER_SPEED_MAX);
        DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
        DJIMotorEnable(loader_motor);
        DJIMotorSetRef(loader_motor, speed_ref);
        return;
    }

    loader_cont_initialized = 0;

    if (!loader_initialized) {
        // Keep target aligned to current multi-turn angle to avoid large error after long-time rotation.
        loader_target_angle = loader_motor->measure.total_angle;
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
    if (pitch_motor == NULL) {
        return;
    }

    uint8_t yaw_ready = GimbalFeedbackReady(yaw_motor) &&
                        yaw_motor->daemon && DaemonIsOnline(yaw_motor->daemon);
    uint8_t pitch_ready = GimbalFeedbackReady(pitch_motor);

    uint32_t now = HAL_GetTick();
    float dt = 0.02f;
    if (gimbal_last_tick != 0U) {
        dt = (now - gimbal_last_tick) / 1000.0f;
        if (dt <= 0.0f) {
            dt = 0.02f;
        }
    }
    dt = float_constrain(dt, 0.001f, 0.05f);
    gimbal_last_tick = now;

    yaw_speed_ref = float_constrain(yaw_speed_ref, YAW_SPEED_MIN, YAW_SPEED_MAX);
    pitch_speed_ref = float_constrain(pitch_speed_ref, -PITCH_SPEED_MAX_DOWN, PITCH_SPEED_MAX_UP);

    if (pitch_ready) {
        if (!pitch_hold_enabled) {
            pitch_hold_inited = 0;
            pitch_center_inited = 0;
            pitch_current_ff = 0.0f;
            DJIMotorStop(pitch_motor);
        } else {
            if (PITCH_FF_ENABLE) {
                float pitch_ff_raw =
                    PITCH_GRAVITY_FF_SIGN * PITCH_GRAVITY_FF_K *
                    sinf((pitch_motor->measure.total_angle - PITCH_GRAVITY_FF_OFFSET_DEG) * PI / 180.0f);
                pitch_ff_raw = float_constrain(pitch_ff_raw, -PITCH_GRAVITY_FF_MAX, PITCH_GRAVITY_FF_MAX);
                pitch_current_ff = pitch_current_ff * PITCH_FF_LPF + pitch_ff_raw * (1.0f - PITCH_FF_LPF);
            } else {
                pitch_current_ff = 0.0f;
            }

            float pitch_current_single = WrapAngle180(pitch_motor->measure.angle_single_round);
            if (!pitch_center_inited) {
                pitch_center_angle = pitch_current_single;
                pitch_center_inited = 1;
            }

            if (!pitch_hold_inited) {
                pitch_hold_angle = CalcTargetTotalAngle(
                    ClampPitchTarget(pitch_current_single),
                    pitch_current_single,
                    pitch_motor->measure.total_angle);
                pitch_hold_inited = 1;
            }

            float pitch_target_single = WrapAngle180(pitch_hold_angle);
            if (pitch_manual_active) {
                pitch_target_single = WrapAngle180(pitch_target_single + pitch_speed_ref * dt);
                pitch_target_single = ClampPitchTarget(pitch_target_single);
                pitch_hold_angle = CalcTargetTotalAngle(
                    pitch_target_single,
                    pitch_current_single,
                    pitch_motor->measure.total_angle);
            }

            DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DJIMotorEnable(pitch_motor);
            DJIMotorSetRef(pitch_motor, pitch_hold_angle);
        }
    } else {
        pitch_hold_inited = 0;
        pitch_center_inited = 0;
        ResetPitchPidRuntime();
        DJIMotorStop(pitch_motor);
    }

    if (yaw_ready) {
        float yaw_current_single = WrapAngle180(yaw_motor->measure.angle_single_round);
        if (!yaw_hold_inited) {
            yaw_hold_angle = yaw_motor->measure.total_angle;
            yaw_hold_inited = 1;
        }
        if (yaw_manual_active) {
            yaw_hold_angle += yaw_speed_ref * dt;
        } else {
            yaw_hold_angle = CalcTargetTotalAngle(
                WrapAngle180(yaw_hold_angle),
                yaw_current_single,
                yaw_motor->measure.total_angle);
        }
        DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
        DJIMotorEnable(yaw_motor);
        DJIMotorSetRef(yaw_motor, yaw_hold_angle);
    } else {
        yaw_hold_inited = 0;
        DJIMotorStop(yaw_motor);
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
    MX_CAN2_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();

    BSPInit();

    TelemetryInit();
    WheelMotorsInit();
    JointMotorsInit();
    GimbalMotorsInit();

    et08_ctrl = ET08_Init(&huart3);
    ImuInit();

    TelemetrySend("wheelleg_all: start\r\n");

    uint32_t last_control_tick = 0;
    uint32_t last_mit_tick = 0;
    uint32_t last_imu_req_tick = 0;
    uint32_t last_telemetry_tick = 0;
    uint32_t last_shooter_tick = 0;
    uint8_t joint_enable_last = 0;

    while (1)
    {
        DaemonTask();
        DJIMotorControl();

        uint32_t now = HAL_GetTick();

        if (now - last_imu_req_tick >= IMU_REQUEST_INTERVAL_MS)
        {
            last_imu_req_tick = now;
            ImuRequestOnce();
        }
        ImuUpdate();

        if (now - last_control_tick >= CONTROL_INTERVAL_MS)
        {
            float dt = (now - last_control_tick) * 0.001f;
            last_control_tick = now;
            UpdateControl(dt);
        }

        if (joint_enable && !joint_enable_last) {
            last_enable_tick = now - ENABLE_INTERVAL_MS;
            last_mit_tick = now - MIT_SEND_INTERVAL_MS;
        }

        if (now - last_enable_tick >= ENABLE_INTERVAL_MS)
        {
            last_enable_tick = now;
            if (joint_enable) {
                JointMotorsEnable();
            }
        }

        if (now - last_mit_tick >= MIT_SEND_INTERVAL_MS)
        {
            last_mit_tick = now;
            if (joint_enable) {
                JointMotorsSendMIT();
            }
        }

        if (!joint_enable && joint_enable_last) {
            JointMotorsDisable();
        }
        joint_enable_last = joint_enable;

        if (now - last_shooter_tick >= SHOOTER_UPDATE_INTERVAL_MS)
        {
            last_shooter_tick = now;
            ProcessRemoteControl();
            ProcessGimbalControl();
            UpdateFrictionControl();
            UpdateLoaderControl();
            UpdateGimbalControl();
        }

        WheelMotorsUpdate();

        if (now - last_telemetry_tick >= TELEMETRY_INTERVAL_MS)
        {
            last_telemetry_tick = now;
            char buffer[200];
            safe_snprintf(buffer, sizeof(buffer),
                          "BAL,mode=%u,rc=%u,imu=%u,pitch_deg=%.2f,ref_deg=%.2f,err_deg=%.2f,rate_dps=%.2f,out=%.0f,i=%.2f,cmdF=%.0f,cmdY=%.0f,wL=%.0f,wR=%.0f\r\n",
                          (unsigned)balance_mode,
                          (unsigned)ET08_IsOnline(),
                          (unsigned)imu_alive,
                          pitch_deg,
                          pitch_ref_deg,
                          balance_err_deg,
                          pitch_rate_deg_s,
                          balance_out,
                          balance_i,
                          forward_cmd,
                          yaw_cmd,
                          wheel_left_ref,
                          wheel_right_ref);
            TelemetrySend(buffer);

            const DMMotor_Feedback *fl = front_left.handle ? DMMotor_GetFeedback(front_left.handle) : NULL;
            const DMMotor_Feedback *rl = rear_left.handle ? DMMotor_GetFeedback(rear_left.handle) : NULL;
            const DMMotor_Feedback *fr = front_right.handle ? DMMotor_GetFeedback(front_right.handle) : NULL;
            const DMMotor_Feedback *rr = rear_right.handle ? DMMotor_GetFeedback(rear_right.handle) : NULL;

            float fl_pos = fl ? fl->position_rad : 0.0f;
            float rl_pos = rl ? rl->position_rad : 0.0f;
            float fr_pos = fr ? fr->position_rad : 0.0f;
            float rr_pos = rr ? rr->position_rad : 0.0f;

            float fl_tq = fl ? fl->torque : 0.0f;
            float rl_tq = rl ? rl->torque : 0.0f;
            float fr_tq = fr ? fr->torque : 0.0f;
            float rr_tq = rr ? rr->torque : 0.0f;

            uint8_t fl_err = fl ? fl->error_state : 0;
            uint8_t rl_err = rl ? rl->error_state : 0;
            uint8_t fr_err = fr ? fr->error_state : 0;
            uint8_t rr_err = rr ? rr->error_state : 0;

            char jbuf[240];
            safe_snprintf(jbuf, sizeof(jbuf),
                          "JNT,hipL=%.3f,kneeL=%.3f,hipR=%.3f,kneeR=%.3f,posFL=%.3f,posRL=%.3f,posFR=%.3f,posRR=%.3f,tqFL=%.2f,tqRL=%.2f,tqFR=%.2f,tqRR=%.2f,errFL=%u,errRL=%u,errFR=%u,errRR=%u\r\n",
                          fl_pos, (rl_pos - fl_pos),
                          fr_pos, (rr_pos - fr_pos),
                          fl_pos, rl_pos, fr_pos, rr_pos,
                          fl_tq, rl_tq, fr_tq, rr_tq,
                          (unsigned)fl_err, (unsigned)rl_err,
                          (unsigned)fr_err, (unsigned)rr_err);
            TelemetrySend(jbuf);

            const DJIMotorInstance *yaw = yaw_motor;
            const DJIMotorInstance *pitch = pitch_motor;
            float yaw_pos = yaw ? yaw->measure.total_angle : 0.0f;
            float pitch_pos = pitch ? pitch->measure.total_angle : 0.0f;
            float yaw_speed = yaw ? yaw->measure.speed_aps : 0.0f;
            float pitch_speed = pitch ? pitch->measure.speed_aps : 0.0f;
            char gbuf[200];
            safe_snprintf(gbuf, sizeof(gbuf),
                          "GIM,yaw_pos=%.2f,yaw_spd=%.2f,yaw_ref=%.2f,yaw_hold=%.2f,pitch_pos=%.2f,pitch_spd=%.2f,pitch_ref=%.2f,pitch_hold=%.2f,ff=%.1f\r\n",
                          yaw_pos,
                          yaw_speed,
                          yaw_speed_ref,
                          yaw_hold_angle,
                          pitch_pos,
                          pitch_speed,
                          pitch_speed_ref,
                          pitch_hold_angle,
                          pitch_current_ff);
            TelemetrySend(gbuf);
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
