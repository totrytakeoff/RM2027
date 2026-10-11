/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Integrated VT keyboard/mouse demo (mecanum chassis + gimbal + hero shooter)
 ******************************************************************************
 * @attention
 *
 * 图传键鼠映射（VT03/VT13）：
 * - 底盘：W/S 前后, A/D 左右, Q/E 旋转；Shift 加速，Ctrl 慢速
 *        右摇杆兜底：ch0->vx, ch1->vy, ch3->wz（用于定位客户端键鼠问题）
 * - 云台：鼠标 X 控制 yaw，鼠标 Y 控制 pitch（gear=S 时生效）
 *        鼠标无输入时，左摇杆兜底：ch3->yaw, ch2->pitch
 * - 射击：R 切换摩擦轮；鼠标左键单发；鼠标中键双发；鼠标右键按住连发
 * - 安全：pause 键急停
 *
 * 统一 CAN 与电机 ID（避免冲突）：
 * - CAN1: 麦轮底盘 M3508 → ID1 左前, ID2 右前, ID3 右后, ID4 左后
 * - CAN1: 拨弹 M3508 → ID5
 * - CAN2: 摩擦轮 M3508 → ID1 / ID2（ID2 反向）
 * - CAN2: Pitch GM6020 → ID365E6 12
 * - CAN2: Yaw   GM6020 → ID4
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
#include <string.h>

#include "bsp_dwt.h"
#include "bsp_init.h"
#include "bsp_log.h"
#include "daemon.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "vt_remote.h"
#include "user_lib.h"

/* Common --------------------------------------------------------------------*/
void SystemClock_Config(void);
static void Debug_DisableWatchdogs(void);
static VT_Ctrl_t *vt_data = NULL;

enum
{
    KEY_W_BIT = 0,
    KEY_S_BIT = 1,
    KEY_A_BIT = 2,
    KEY_D_BIT = 3,
    KEY_SHIFT_BIT = 4,
    KEY_CTRL_BIT = 5,
    KEY_Q_BIT = 6,
    KEY_E_BIT = 7,
    KEY_R_BIT = 8,
    KEY_F_BIT = 9,
    KEY_G_BIT = 10,
    KEY_Z_BIT = 11,
    KEY_X_BIT = 12,
    KEY_C_BIT = 13,
    KEY_V_BIT = 14,
    KEY_B_BIT = 15,
};

static uint16_t vt_last_keyboard = 0u;
static uint8_t vt_last_mouse_left = 0u;
static uint8_t vt_last_mouse_middle = 0u;

#define KM_CHASSIS_VEL_BASE 8.0f
#define KM_CHASSIS_WZ_BASE 60.0f
#define KM_FAST_SCALE 1.6f
#define KM_SLOW_SCALE 0.45f
#define KM_MOUSE_YAW_GAIN 15.0f
#define KM_MOUSE_PITCH_GAIN 12.0f
#define KM_MOUSE_DEADZONE 5
#define KM_STICK_DEADZONE 50
#define KM_STICK_SCALE_DEN 660.0f

/* 本测试使用 USART6 图传串口 */
#define VT_UART_HANDLE huart6
#define VT_UART_LABEL "huart6"
#define VT_DIAG_LOG_INTERVAL_MS 500U
#define VT_REQUIRE_CRC_OK 1u
#define VT_ENABLE_PAUSE_ESTOP 1u
#define VT_REQUIRE_ONLINE 1u
#define VT_DEBUG_FORCE_DRIVE_ON_VALID_FRAME 0u

/* ============================== 底盘 ============================== */
#define CHASSIS_MOTOR_COUNT 4U
#define CHASSIS_UPDATE_INTERVAL_MS 20U
#define CHASSIS_WHEEL_RADIUS 0.075f
#define CHASSIS_WHEEL_BASE 0.34f
#define CHASSIS_MAX_VEL 20.0f
#define CHASSIS_MAX_ROTATE 120.0f
#define M3508_SPEED_MAX 50000.0f
#define M3508_SPEED_MIN (-M3508_SPEED_MAX)
#define CHASSIS_SPEED_DEADZONE 120.0f
#define CHASSIS_DEADZONE_VX 0.15f
#define CHASSIS_DEADZONE_VY 0.15f
#define CHASSIS_DEADZONE_WZ 0.2f
#define SPEED_FILTER_COEF 0.65f
#define MOTOR_STABILIZE_TIME_MS 2000U

#define MOTOR_FRONT_LEFT 1U
#define MOTOR_FRONT_RIGHT 2U
#define MOTOR_BACK_RIGHT 3U
#define MOTOR_BACK_LEFT 4U

static DJIMotorInstance *chassis_motors[CHASSIS_MOTOR_COUNT] = {NULL};
static float chassis_vx = 0.0f;
static float chassis_vy = 0.0f;
static float chassis_wz = 0.0f;
static float wheel_speeds[CHASSIS_MOTOR_COUNT] = {0.0f};
static uint8_t chassis_zero_speed = 1;
static uint8_t chassis_stabilized = 0;
static uint32_t system_start_time = 0;
static float filtered_vx = 0.0f;
static float filtered_vy = 0.0f;
static float filtered_wz = 0.0f;

static void ChassisMotorsInit(void);
static void ChassisForceZero(void);
static void ChassisProcessRC(void);
static void ChassisUpdateKinematics(void);
static uint8_t VTKeyDown(uint16_t key_mask, uint8_t bit_index);
static void VTInputDiag(const VT_Ctrl_t *ctrl);
static uint8_t VTControlBlocked(const VT_Ctrl_t *ctrl);
static void ChassisCmdDiag(void);

/* ============================== 云台 ============================== */
#define GIMBAL_UPDATE_INTERVAL_MS 20U
#define INS_TASK_PERIOD_MS 1U
#define YAW_SPEED_MAX 3600.0f
#define YAW_SPEED_MIN (-YAW_SPEED_MAX)
#define PITCH_SPEED_MAX_UP 2000.0f
#define PITCH_SPEED_MAX_DOWN 5000.0f
#define GM6020_SPEED_DEADZONE 30.0f
#define RC_DEADZONE 50
#define GIMBAL_ANGLE_KP 30.0f
#define GIMBAL_ANGLE_MAXOUT 2000.0f
#define YAW_HOLD_KP 14.0f
#define YAW_HOLD_KI 2.0f
#define YAW_HOLD_I_LIMIT 8000.0f
#define YAW_HOLD_STARTUP_KP 24.0f
#define YAW_HOLD_STARTUP_KI 4.0f
#define YAW_HOLD_STARTUP_SPEED_MAX 6000.0f
#define YAW_HOLD_TARGET_ANGLE 199.510f   // Initial yaw hold target (deg, single round)
#define PITCH_HOLD_KP 28.0f
#define PITCH_HOLD_KI 3.5f
#define PITCH_HOLD_I_LIMIT 12000.0f
#define PITCH_HOLD_STARTUP_KP 40.0f
#define PITCH_HOLD_STARTUP_KI 6.0f
#define PITCH_HOLD_STARTUP_SPEED_MAX_UP 4000.0f
#define PITCH_HOLD_DIR_SIGN 1.0f
#define PITCH_GRAVITY_COMP_BASE 0.0f
#define PITCH_HOLD_TARGET_ANGLE 255.628f  // Initial pitch hold target (deg, single round)
#define PITCH_GRAVITY_COMP_DOWN_BONUS 60000.0f
#define YAW_SPEED_PID_MAXOUT 15000.0f
#define PITCH_SPEED_PID_MAXOUT 22000.0f
#define GIMBAL_POS_LOG_INTERVAL_MS 200U
#define GIMBAL_STARTUP_HOLD_TIMEOUT_MS 6000U
#define GIMBAL_STARTUP_HOLD_TOLERANCE_DEG 1.0f

#define YAW_MOTOR_ID 2U   // CAN1
#define PITCH_MOTOR_ID 5U // CAN2

static DJIMotorInstance *yaw_motor = NULL;
static DJIMotorInstance *pitch_motor = NULL;
static attitude_t *gimbal_imu = NULL;
static float yaw_imu_speed_fdb = 0.0f;
static float yaw_speed_ref = 0.0f;
static float pitch_speed_ref = 0.0f;
static uint8_t gimbal_zero_speed = 1;
static uint8_t pitch_hold_enabled = 0;
static uint32_t pitch_ecd_log_tick = 0;
static float pitch_hold_angle = 0.0f;
static float pitch_hold_i = 0.0f;
static uint32_t gimbal_pos_log_tick = 0;
static uint8_t yaw_hold_enabled = 0;
static float yaw_hold_angle = 0.0f;
static float yaw_hold_i = 0.0f;
static uint8_t pitch_hold_active = 0;
static uint8_t yaw_hold_active = 0;
static float yaw_hold_error_prev = 0.0f;
static float pitch_hold_error_prev = 0.0f;
static uint8_t gimbal_startup_hold_active = 0;
static uint32_t gimbal_startup_hold_tick = 0;

static void GimbalMotorsInit(void);
static void GimbalProcessRC(void);
static void GimbalUpdate(void);
static float WrapAngle180(float angle);
static float CalcTargetTotalAngle(float target_single, float current_single, float current_total);
static void GimbalLogPosition(void);
static void GimbalEnterHoldTarget(void);

/* ============================== 射击 ============================== */
#define SHOOT_UPDATE_INTERVAL_MS 20U

#define FRICTION_MOTOR_COUNT 2U
static const uint8_t FRICTION_CAN_IDS[FRICTION_MOTOR_COUNT] = {1U, 2U}; // CAN2
#define LOADER_CAN_ID 5U                                               // CAN1

#define FRICTION_SPEED_MAX 30000.0f
#define FRICTION_SPEED_MIN (-FRICTION_SPEED_MAX)
#define FRICTION_SPEED_TARGET 30000.0f  // 提高摩擦轮转速
#define FRICTION_RC_DEADZONE 50

#define LOADER_SPEED_MAX 12000.0f
#define LOADER_SPEED_MIN (-LOADER_SPEED_MAX)
#define LOADER_ANGLE_MAX 36000.0f
#define LOADER_ANGLE_MIN (-LOADER_ANGLE_MAX)
#define LOADER_GEAR_RATIO 19.0f
#define LOADER_OUTPUT_STEP_DEG 60.0f
#define LOADER_ANGLE_STEP_DEG (LOADER_OUTPUT_STEP_DEG * LOADER_GEAR_RATIO)
#define LOADER_STEP_DIR (1.0f)  // 单射方向与连射方向一致
#define LOADER_DIAL_ENABLE_THRESH 100  // 拨轮下拨超过此值才允许拨弹
#define LOADER_CONTINUOUS_SPEED 12000.0f

// 基于连射模式角速度计算的时间间隔
#define LOADER_ANGULAR_SPEED (LOADER_CONTINUOUS_SPEED)  // 连射角速度（度/秒）
#define SINGLE_SHOT_ANGLE_DEG 60.0f                     // 单射转动角度（60度）
#define DOUBLE_SHOT_ANGLE_DEG 120.0f                    // 双射转动角度（120度）
#define SINGLE_SHOT_INTERVAL_MS 1U  // 单射时间间隔（缩短中断时间）
#define DOUBLE_SHOT_INTERVAL_MS 2U  // 双射时间间隔（缩短中断时间）

static DJIMotorInstance *friction_motors[FRICTION_MOTOR_COUNT] = {NULL};
static DJIMotorInstance *loader_motor = NULL;
static uint8_t friction_enabled = 0;
static uint8_t fire_mode = 0; // 0 single, 2 continuous
static uint8_t pending_shots = 0;
static uint8_t loader_initialized = 0;
static uint8_t loader_trigger_allowed = 0;
static float friction_manual_speed = 0.0f;
static float loader_target_angle = 0.0f;
static uint32_t last_shot_tick = 0;
static uint32_t last_step_tick = 0;

static void EnsureFrictionMotorsReady(void);
static void EnsureLoaderMotorReady(void);
static void ShootProcessRC(void);
static void UpdateFrictionControl(void);
static void UpdateLoaderControl(void);
static float ClampFloat(float value, float min, float max);

/* ============================== Common Helpers ============================== */
static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

/* ============================== 底盘实现 ============================== */
static void ChassisMotorsInit(void)
{
    const uint8_t motor_ids[CHASSIS_MOTOR_COUNT] = {
        MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, MOTOR_BACK_RIGHT, MOTOR_BACK_LEFT
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
                .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
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

static void ChassisForceZero(void)
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
    chassis_vx = chassis_vy = chassis_wz = 0.0f;
    filtered_vx = filtered_vy = filtered_wz = 0.0f;
    chassis_zero_speed = 1;
}

static void ChassisProcessRC(void)
{
    if (vt_data == NULL || (VT_REQUIRE_ONLINE && !VT_IsOnline())) {
        chassis_vx = chassis_vy = chassis_wz = 0.0f;
        chassis_zero_speed = 1;
        VTInputDiag(NULL);
        return;
    }

    vt_data = VT_GetCtrl();
    if (VTControlBlocked(vt_data)) {
        chassis_vx = chassis_vy = chassis_wz = 0.0f;
        chassis_zero_speed = 1;
        VTInputDiag(vt_data);
        return;
    }

#if VT_DEBUG_FORCE_DRIVE_ON_VALID_FRAME
    if (vt_data->frame_count > 20u) {
        chassis_vx = 0.0f;
        chassis_vy = 4.0f;
        chassis_wz = 0.0f;
        chassis_zero_speed = 0u;
        VTInputDiag(vt_data);
        return;
    }
#endif

    uint16_t key_mask = vt_data->keyboard_value;
    float speed_scale = 1.0f;
    if (VTKeyDown(key_mask, KEY_SHIFT_BIT))
        speed_scale *= KM_FAST_SCALE;
    if (VTKeyDown(key_mask, KEY_CTRL_BIT))
        speed_scale *= KM_SLOW_SCALE;

    float vx_cmd = 0.0f;
    float vy_cmd = 0.0f;
    float wz_cmd = 0.0f;

    if (VTKeyDown(key_mask, KEY_W_BIT))
        vy_cmd += KM_CHASSIS_VEL_BASE * speed_scale;
    if (VTKeyDown(key_mask, KEY_S_BIT))
        vy_cmd -= KM_CHASSIS_VEL_BASE * speed_scale;
    if (VTKeyDown(key_mask, KEY_D_BIT))
        vx_cmd += KM_CHASSIS_VEL_BASE * speed_scale;
    if (VTKeyDown(key_mask, KEY_A_BIT))
        vx_cmd -= KM_CHASSIS_VEL_BASE * speed_scale;
    if (VTKeyDown(key_mask, KEY_E_BIT))
        wz_cmd += KM_CHASSIS_WZ_BASE * speed_scale;
    if (VTKeyDown(key_mask, KEY_Q_BIT))
        wz_cmd -= KM_CHASSIS_WZ_BASE * speed_scale;

    /* Stick fallback to help isolate keyboard/client issues */
    if (abs(vt_data->ch0_right_x.centered) >= KM_STICK_DEADZONE)
        vx_cmd += CHASSIS_MAX_VEL * (float)vt_data->ch0_right_x.centered / KM_STICK_SCALE_DEN;
    if (abs(vt_data->ch1_right_y.centered) >= KM_STICK_DEADZONE)
        vy_cmd += CHASSIS_MAX_VEL * (float)vt_data->ch1_right_y.centered / KM_STICK_SCALE_DEN;
    if ((vt_data->gear != VT_GEAR_S) && (abs(vt_data->ch3_left_x.centered) >= KM_STICK_DEADZONE))
        wz_cmd += CHASSIS_MAX_ROTATE * (float)vt_data->ch3_left_x.centered / KM_STICK_SCALE_DEN;

    vx_cmd = ClampFloat(vx_cmd, -CHASSIS_MAX_VEL, CHASSIS_MAX_VEL);
    vy_cmd = ClampFloat(vy_cmd, -CHASSIS_MAX_VEL, CHASSIS_MAX_VEL);
    wz_cmd = ClampFloat(wz_cmd, -CHASSIS_MAX_ROTATE, CHASSIS_MAX_ROTATE);

    chassis_vx = float_deadband(vx_cmd, -CHASSIS_DEADZONE_VX, CHASSIS_DEADZONE_VX);
    chassis_vy = float_deadband(vy_cmd, -CHASSIS_DEADZONE_VY, CHASSIS_DEADZONE_VY);
    chassis_wz = float_deadband(wz_cmd, -CHASSIS_DEADZONE_WZ, CHASSIS_DEADZONE_WZ);

    chassis_zero_speed = (fabsf(chassis_vx) < 1e-4f && fabsf(chassis_vy) < 1e-4f && fabsf(chassis_wz) < 1e-4f) ? 1u : 0u;
    VTInputDiag(vt_data);
}

static void ChassisUpdateKinematics(void)
{
    if (!chassis_stabilized) {
        uint32_t now = HAL_GetTick();
        if (now - system_start_time >= MOTOR_STABILIZE_TIME_MS) {
            for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                if (chassis_motors[i] != NULL) {
                    DJIMotorEnable(chassis_motors[i]);
                }
            }
            chassis_stabilized = 1;
            filtered_vx = filtered_vy = filtered_wz = 0.0f;
        } else {
            for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                if (chassis_motors[i] != NULL) {
                    DJIMotorSetRef(chassis_motors[i], 0.0f);
                }
            }
            return;
        }
    }

    if (chassis_zero_speed) {
        for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
            if (chassis_motors[i] != NULL) {
                DJIMotorSetRef(chassis_motors[i], 0.0f);
            }
        }
        filtered_vx = filtered_vy = filtered_wz = 0.0f;
        return;
    }

    filtered_vx = filtered_vx * SPEED_FILTER_COEF + chassis_vx * (1.0f - SPEED_FILTER_COEF);
    filtered_vy = filtered_vy * SPEED_FILTER_COEF + chassis_vy * (1.0f - SPEED_FILTER_COEF);
    filtered_wz = filtered_wz * SPEED_FILTER_COEF + chassis_wz * (1.0f - SPEED_FILTER_COEF);

    float L = CHASSIS_WHEEL_BASE / 2.0f;
    float v1 = filtered_vy + filtered_vx + (L * filtered_wz);
    float v2 = filtered_vy + filtered_vx - (L * filtered_wz);
    float v3 = filtered_vy - filtered_vx - (L * filtered_wz);
    float v4 = filtered_vy - filtered_vx + (L * filtered_wz);

    wheel_speeds[0] = v1 / CHASSIS_WHEEL_RADIUS;
    wheel_speeds[1] = v2 / CHASSIS_WHEEL_RADIUS;
    wheel_speeds[2] = v3 / CHASSIS_WHEEL_RADIUS;
    wheel_speeds[3] = v4 / CHASSIS_WHEEL_RADIUS;

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
    ChassisCmdDiag();
}

/* ============================== 云台实现 ============================== */
static void GimbalMotorsInit(void)
{
    gimbal_imu = INS_Init();

    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = YAW_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = GIMBAL_ANGLE_KP,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .DeadBand = 0.1f,
                .IntegralLimit = 100.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = GIMBAL_ANGLE_MAXOUT,
            },
            .speed_PID = {
                .Kp = 4.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = YAW_SPEED_PID_MAXOUT,
            },
            .other_angle_feedback_ptr = (gimbal_imu != NULL) ? &gimbal_imu->YawTotalAngle : NULL,
            .other_speed_feedback_ptr = &yaw_imu_speed_fdb,
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

    config.can_init_config.can_handle = &hcan2;
    config.can_init_config.tx_id = PITCH_MOTOR_ID;
    config.controller_param_init_config.speed_PID.MaxOut = PITCH_SPEED_PID_MAXOUT;
    config.controller_param_init_config.other_angle_feedback_ptr = NULL;
    pitch_motor = DJIMotorInit(&config);

    if (yaw_motor != NULL) {
        DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
        DJIMotorStop(yaw_motor);
        yaw_hold_angle = (gimbal_imu != NULL) ? gimbal_imu->YawTotalAngle : YAW_HOLD_TARGET_ANGLE;
    }
    if (pitch_motor != NULL) {
        DJIMotorOuterLoop(pitch_motor, SPEED_LOOP);
        DJIMotorStop(pitch_motor);
    }
}

static void GimbalProcessRC(void)
{
    if (gimbal_startup_hold_active) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        GimbalEnterHoldTarget();
        if (yaw_motor != NULL && pitch_motor != NULL) {
            float yaw_current = (gimbal_imu != NULL) ? gimbal_imu->YawTotalAngle : yaw_motor->measure.total_angle;
            float yaw_err = WrapAngle180(yaw_hold_angle - yaw_current);
            float pitch_err =
                WrapAngle180(pitch_hold_angle - pitch_motor->measure.angle_single_round);
            uint8_t yaw_ready = (fabsf(yaw_err) <= GIMBAL_STARTUP_HOLD_TOLERANCE_DEG);
            uint8_t pitch_ready = (fabsf(pitch_err) <= GIMBAL_STARTUP_HOLD_TOLERANCE_DEG);
            uint32_t now = HAL_GetTick();
            if ((yaw_ready && pitch_ready) ||
                (now - gimbal_startup_hold_tick >= GIMBAL_STARTUP_HOLD_TIMEOUT_MS)) {
                gimbal_startup_hold_active = 0;
                pitch_hold_enabled = 0;
                yaw_hold_enabled = 0;
                pitch_hold_active = 0;
                yaw_hold_active = 0;
            }
        }
        return;
    }

    if (vt_data == NULL || (VT_REQUIRE_ONLINE && !VT_IsOnline())) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        gimbal_zero_speed = 1;
        pitch_hold_enabled = 0;
        yaw_hold_enabled = 0;
        pitch_hold_active = 0;
        yaw_hold_active = 0;
        return;
    }

    vt_data = VT_GetCtrl();
    if (VTControlBlocked(vt_data)) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        gimbal_zero_speed = 1;
        pitch_hold_enabled = 0;
        yaw_hold_enabled = 0;
        pitch_hold_active = 0;
        yaw_hold_active = 0;
        return;
    }

    /* Only allow gimbal in combat gear to avoid accidental movement */
    if (vt_data->gear != VT_GEAR_S) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        gimbal_zero_speed = 1;
        pitch_hold_enabled = 0;
        yaw_hold_enabled = 0;
        pitch_hold_active = 0;
        yaw_hold_active = 0;
        return;
    }

    uint8_t mouse_active = (abs(vt_data->mouse_x) >= KM_MOUSE_DEADZONE) ||
                           (abs(vt_data->mouse_y) >= KM_MOUSE_DEADZONE);
    uint8_t stick_active = (abs(vt_data->ch3_left_x.centered) >= KM_STICK_DEADZONE) ||
                           (abs(vt_data->ch2_left_y.centered) >= KM_STICK_DEADZONE);

    if (!mouse_active && !stick_active) {
        yaw_hold_angle = (gimbal_imu != NULL) ? gimbal_imu->YawTotalAngle : yaw_motor->measure.total_angle;
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        gimbal_zero_speed = 0;
        pitch_hold_enabled = 1;
        yaw_hold_enabled = 1;
        pitch_hold_active = 1;
        yaw_hold_active = 1;
        yaw_hold_i = 0.0f;
        return;
    }

    gimbal_zero_speed = 0;
    pitch_hold_enabled = 0;
    yaw_hold_enabled = 0;
    pitch_hold_active = 0;
    yaw_hold_active = 0;

    if (mouse_active) {
        yaw_speed_ref = (float)vt_data->mouse_x * KM_MOUSE_YAW_GAIN;
        pitch_speed_ref = -(float)vt_data->mouse_y * KM_MOUSE_PITCH_GAIN;
    } else {
        yaw_speed_ref = YAW_SPEED_MAX * (float)vt_data->ch3_left_x.centered / KM_STICK_SCALE_DEN;
        pitch_speed_ref = -PITCH_SPEED_MAX_UP * (float)vt_data->ch2_left_y.centered / KM_STICK_SCALE_DEN;
    }

    yaw_speed_ref = float_deadband(yaw_speed_ref, -GM6020_SPEED_DEADZONE, GM6020_SPEED_DEADZONE);
    pitch_speed_ref = float_deadband(pitch_speed_ref, -GM6020_SPEED_DEADZONE, GM6020_SPEED_DEADZONE);
}

static void GimbalEnterHoldTarget(void)
{
    gimbal_zero_speed = 0;
    pitch_hold_enabled = 1;
    yaw_hold_enabled = 1;
    if (!pitch_hold_active && pitch_motor != NULL) {
        pitch_hold_angle = PITCH_HOLD_TARGET_ANGLE;
        pitch_hold_i = 0.0f;
        pitch_hold_active = 1;
    }
    if (!yaw_hold_active && yaw_motor != NULL) {
        yaw_hold_angle = (gimbal_imu != NULL) ? gimbal_imu->YawTotalAngle : YAW_HOLD_TARGET_ANGLE;
        yaw_hold_i = 0.0f;
        yaw_hold_active = 1;
    }
}

static void GimbalUpdate(void)
{
    if (gimbal_imu != NULL) {
        yaw_imu_speed_fdb = gimbal_imu->Gyro[2];
    }

    if (gimbal_startup_hold_active) {
        if (yaw_motor != NULL) {
            float current_yaw = (gimbal_imu != NULL) ? gimbal_imu->YawTotalAngle : yaw_motor->measure.total_angle;
            float target_total = CalcTargetTotalAngle(
                yaw_hold_angle,
                current_yaw,
                current_yaw);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, (gimbal_imu != NULL) ? OTHER_FEED : MOTOR_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, (gimbal_imu != NULL) ? OTHER_FEED : MOTOR_FEED);
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
            DJIMotorEnable(yaw_motor);
            DJIMotorSetRef(yaw_motor, target_total);
        }
        if (pitch_motor != NULL) {
            float target_total = CalcTargetTotalAngle(
                PITCH_HOLD_TARGET_ANGLE,
                pitch_motor->measure.angle_single_round,
                pitch_motor->measure.total_angle);
            DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DJIMotorEnable(pitch_motor);
            DJIMotorSetRef(pitch_motor, target_total);
        }
        return;
    }

    if (gimbal_zero_speed) {
        if (yaw_motor != NULL) {
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, (gimbal_imu != NULL) ? OTHER_FEED : MOTOR_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, (gimbal_imu != NULL) ? OTHER_FEED : MOTOR_FEED);
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
            float current_yaw = (gimbal_imu != NULL) ? gimbal_imu->YawTotalAngle : yaw_motor->measure.total_angle;
            DJIMotorSetRef(yaw_motor, current_yaw);
        }
        if (pitch_motor != NULL) {
            DJIMotorOuterLoop(pitch_motor, SPEED_LOOP);
            DJIMotorSetRef(pitch_motor, PITCH_GRAVITY_COMP_BASE);
        }
        return;
    }

    yaw_speed_ref = float_constrain(yaw_speed_ref, YAW_SPEED_MIN, YAW_SPEED_MAX);
    if (!pitch_hold_enabled && pitch_speed_ref > 0.0f) {
        pitch_speed_ref += PITCH_GRAVITY_COMP_DOWN_BONUS;
    }
    pitch_speed_ref = float_constrain(pitch_speed_ref, -PITCH_SPEED_MAX_UP, PITCH_SPEED_MAX_DOWN);

    if (yaw_motor != NULL) {
        if (yaw_hold_enabled) {
            float current = (gimbal_imu != NULL) ? gimbal_imu->YawTotalAngle : yaw_motor->measure.total_angle;
            float error = WrapAngle180(yaw_hold_angle - current);
            if ((error * yaw_hold_error_prev) < 0.0f) {
                yaw_hold_i = 0.0f;
            }
            float kp = gimbal_startup_hold_active ? YAW_HOLD_STARTUP_KP : YAW_HOLD_KP;
            float ki = gimbal_startup_hold_active ? YAW_HOLD_STARTUP_KI : YAW_HOLD_KI;
            float speed_max = gimbal_startup_hold_active ? YAW_HOLD_STARTUP_SPEED_MAX : YAW_SPEED_MAX;
            yaw_hold_i += error * 0.02f;
            yaw_hold_i = float_constrain(yaw_hold_i, -YAW_HOLD_I_LIMIT, YAW_HOLD_I_LIMIT);
            float hold_speed = error * kp + yaw_hold_i * ki;
            yaw_speed_ref = float_constrain(hold_speed, -speed_max, speed_max);
            yaw_hold_error_prev = error;
        }
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, (gimbal_imu != NULL) ? OTHER_FEED : MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, (gimbal_imu != NULL) ? OTHER_FEED : MOTOR_FEED);
        DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
        DJIMotorEnable(yaw_motor);
        DJIMotorSetRef(yaw_motor, yaw_speed_ref);
    }
    if (pitch_motor != NULL) {
        DJIMotorOuterLoop(pitch_motor, SPEED_LOOP);
        if (pitch_hold_enabled) {
            uint32_t now = HAL_GetTick();
            if (now - pitch_ecd_log_tick >= 500U) {
                pitch_ecd_log_tick = now;
                LOGINFO("[pitch] level ecd=%u", (unsigned)pitch_motor->measure.ecd);
            }
            float current = pitch_motor->measure.angle_single_round;
            float error = WrapAngle180(pitch_hold_angle - current);
            if ((error * pitch_hold_error_prev) < 0.0f) {
                pitch_hold_i = 0.0f;
            }
            float kp = gimbal_startup_hold_active ? PITCH_HOLD_STARTUP_KP : PITCH_HOLD_KP;
            float ki = gimbal_startup_hold_active ? PITCH_HOLD_STARTUP_KI : PITCH_HOLD_KI;
            float speed_max_up =
                gimbal_startup_hold_active ? PITCH_HOLD_STARTUP_SPEED_MAX_UP : PITCH_SPEED_MAX_UP;
            pitch_hold_i += error * 0.02f;
            pitch_hold_i = float_constrain(pitch_hold_i, -PITCH_HOLD_I_LIMIT, PITCH_HOLD_I_LIMIT);
            float hold_speed =
                PITCH_HOLD_DIR_SIGN * (error * kp + pitch_hold_i * ki);
            pitch_speed_ref = float_constrain(hold_speed, -speed_max_up, PITCH_SPEED_MAX_DOWN);
            pitch_hold_error_prev = error;
        }
        DJIMotorEnable(pitch_motor);
        DJIMotorSetRef(pitch_motor, pitch_speed_ref);
    }
}

static void GimbalLogPosition(void)
{
    if (yaw_motor == NULL || pitch_motor == NULL) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if (now - gimbal_pos_log_tick < GIMBAL_POS_LOG_INTERVAL_MS) {
        return;
    }
    gimbal_pos_log_tick = now;

    char yaw_angle[32];
    char pitch_angle[32];
    char yaw_total[32];
    char pitch_total[32];
    Float2Str(yaw_angle, yaw_motor->measure.angle_single_round);
    Float2Str(pitch_angle, pitch_motor->measure.angle_single_round);
    Float2Str(yaw_total, yaw_motor->measure.total_angle);
    Float2Str(pitch_total, pitch_motor->measure.total_angle);
    LOG("GIMBAL_POS yaw=%s yaw_total=%s pitch=%s pitch_total=%s",
        yaw_angle,
        yaw_total,
        pitch_angle,
        pitch_total);
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

/* ============================== 射击实现 ============================== */
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
                    .Kp = 10.0f,
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
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .MaxOut = 12000.0f,
                },
                .current_PID = {
                    .Kp = 0.5f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .IntegralLimit = 3000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .MaxOut = 15000.0f,
                },
            },
            .controller_setting_init_config = {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type = SPEED_LOOP,
                .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
                .motor_reverse_flag = (i == 1U) ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL,
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
                    .Kp = 2.0f,
                    .Ki = 0.005f,
                    .Kd = 0.05f,
                    .MaxOut = LOADER_SPEED_MAX,
                    .IntegralLimit = 800.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                },
                .speed_PID = {
                    .Kp = 5.0f,
                    .Ki = 0.01f,
                    .Kd = 0.05f,
                    .IntegralLimit = 2000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .MaxOut = 10000.0f,
                },
            .current_PID = {
                .Kp = 0.6f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 20000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
            // Loader installed opposite to logical forward; reverse direction to match trigger intent.
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };

    loader_motor = DJIMotorInit(&config);
}

static void ShootProcessRC(void)
{
    if (vt_data == NULL || (VT_REQUIRE_ONLINE && !VT_IsOnline())) {
        friction_enabled = 0;
        return;
    }

    vt_data = VT_GetCtrl();
    if (VTControlBlocked(vt_data)) {
        friction_enabled = 0;
        loader_trigger_allowed = 0;
        pending_shots = 0;
        fire_mode = 0;
        return;
    }

    uint16_t key_mask = vt_data->keyboard_value;
    uint8_t key_r = VTKeyDown(key_mask, KEY_R_BIT);
    uint8_t mouse_left = vt_data->mouse_left_pressed ? 1u : 0u;
    uint8_t mouse_middle = vt_data->mouse_middle_pressed ? 1u : 0u;
    uint8_t mouse_right = vt_data->mouse_right_pressed ? 1u : 0u;

    uint8_t key_r_rise = key_r && !VTKeyDown(vt_last_keyboard, KEY_R_BIT);
    uint8_t mouse_left_rise = mouse_left && !vt_last_mouse_left;
    uint8_t mouse_middle_rise = mouse_middle && !vt_last_mouse_middle;

    if (key_r_rise)
        friction_enabled = friction_enabled ? 0u : 1u;

    loader_trigger_allowed = friction_enabled && (vt_data->gear == VT_GEAR_S);
    friction_manual_speed = 0.0f;

    if (!friction_enabled || !loader_trigger_allowed) {
        pending_shots = 0;
        loader_initialized = 0;
        fire_mode = 0;
        vt_last_keyboard = key_mask;
        vt_last_mouse_left = mouse_left;
        vt_last_mouse_middle = mouse_middle;
        return;
    }

    if (mouse_right) {
        fire_mode = 2;
        pending_shots = 0;
    } else {
        uint32_t now = HAL_GetTick();
        if (mouse_middle_rise && (now - last_shot_tick >= DOUBLE_SHOT_INTERVAL_MS)) {
            fire_mode = 1;
            pending_shots = 2;
            last_shot_tick = now;
            last_step_tick = now;
        } else if (mouse_left_rise && (now - last_shot_tick >= SINGLE_SHOT_INTERVAL_MS)) {
            fire_mode = 0;
            pending_shots = 1;
            last_shot_tick = now;
            last_step_tick = now;
        } else if (pending_shots == 0) {
            fire_mode = 0;
        }
    }

    vt_last_keyboard = key_mask;
    vt_last_mouse_left = mouse_left;
    vt_last_mouse_middle = mouse_middle;
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
        /* CAN2 ID1（索引0）用左摇杆前后手动控制，ID2保持固定目标 */
        DJIMotorSetRef(motor, target_speed);
    }
}

static void UpdateLoaderControl(void)
{
    EnsureLoaderMotorReady();

    if (!friction_enabled || !loader_trigger_allowed || loader_motor == NULL) {
        if (loader_motor != NULL) {
            DJIMotorStop(loader_motor);
        }
        pending_shots = 0;
        loader_initialized = 0;
        return;
    }

    // 连射模式：拨弹盘持续转动
    if (fire_mode == 2) {
        DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
        DJIMotorEnable(loader_motor);
        float speed_ref = ClampFloat(-LOADER_STEP_DIR * LOADER_CONTINUOUS_SPEED, LOADER_SPEED_MIN, LOADER_SPEED_MAX);
        DJIMotorSetRef(loader_motor, speed_ref);
        pending_shots = 0;
        loader_initialized = 0;
        return;
    }

    // 单射/双射模式：完全停止后停顿再继续
    if (pending_shots > 0) {
        uint32_t now = HAL_GetTick();
        uint32_t interval = (fire_mode == 0) ? SINGLE_SHOT_INTERVAL_MS : DOUBLE_SHOT_INTERVAL_MS;
        
        // 检查是否达到时间间隔
        if (now - last_step_tick >= interval) {
            // 发射子弹：先转动一段时间
            if (loader_initialized == 0) {
                // 开始转动
                DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
                DJIMotorEnable(loader_motor);
                float speed_ref = ClampFloat(-LOADER_STEP_DIR * LOADER_CONTINUOUS_SPEED, LOADER_SPEED_MIN, LOADER_SPEED_MAX);
                DJIMotorSetRef(loader_motor, speed_ref);
                loader_initialized = 1;
                last_step_tick = now;
            } else if (now - last_step_tick >= 20) { // 转动20ms后停止
                // 停止电机
                DJIMotorStop(loader_motor);
                loader_initialized = 0;
                pending_shots--;
                last_step_tick = now;
                
                // 如果所有子弹发射完成，完全停止
                if (pending_shots == 0) {
                    loader_initialized = 0;
                }
            }
        } else {
            // 在间隔时间内保持停止状态
            DJIMotorStop(loader_motor);
        }
    } else {
        // 没有子弹要发射，拨弹盘停止
        DJIMotorStop(loader_motor);
        loader_initialized = 0;
    }
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

/* ============================== 定时器中断处理 ============================== */


/* ============================== 公共工具 ============================== */
static uint8_t VTKeyDown(uint16_t key_mask, uint8_t bit_index)
{
    return (uint8_t)((key_mask >> bit_index) & 0x01u);
}

static uint8_t VTControlBlocked(const VT_Ctrl_t *ctrl)
{
    if (ctrl == NULL)
        return 1u;
    if (VT_REQUIRE_CRC_OK && (!ctrl->crc_ok))
        return 1u;
    if (VT_ENABLE_PAUSE_ESTOP && ctrl->pause_pressed)
        return 1u;
    return 0u;
}

static void VTInputDiag(const VT_Ctrl_t *ctrl)
{
    static uint32_t last_log_tick = 0u;
    uint32_t now = HAL_GetTick();
    if (now - last_log_tick < VT_DIAG_LOG_INTERVAL_MS)
        return;
    last_log_tick = now;

    if (ctrl == NULL) {
        LOGINFO("[vt-diag] online=%u ctrl=null", (unsigned)VT_IsOnline());
        return;
    }

    LOGINFO("[vt-diag] online=%u crc=%u gear=%u pause=%u key=0x%04X ch0=%d ch1=%d ch2=%d ch3=%d mouse=(%d,%d) frame=%lu bad=%lu block=%u",
            (unsigned)VT_IsOnline(),
            (unsigned)ctrl->crc_ok,
            (unsigned)ctrl->gear,
            (unsigned)ctrl->pause_pressed,
            (unsigned)ctrl->keyboard_value,
            (int)ctrl->ch0_right_x.centered,
            (int)ctrl->ch1_right_y.centered,
            (int)ctrl->ch2_left_y.centered,
            (int)ctrl->ch3_left_x.centered,
            (int)ctrl->mouse_x,
            (int)ctrl->mouse_y,
            (unsigned long)ctrl->frame_count,
            (unsigned long)ctrl->bad_count,
            (unsigned)VTControlBlocked(ctrl));
}

static void ChassisCmdDiag(void)
{
    static uint32_t last_log_tick = 0u;
    uint32_t now = HAL_GetTick();
    if (now - last_log_tick < VT_DIAG_LOG_INTERVAL_MS)
        return;
    last_log_tick = now;

    LOGINFO("[chassis-diag] zero=%u cmd(vx=%.2f vy=%.2f wz=%.2f) wheel_ref=(%.1f %.1f %.1f %.1f)",
            (unsigned)chassis_zero_speed,
            (double)chassis_vx,
            (double)chassis_vy,
            (double)chassis_wz,
            (double)wheel_speeds[0],
            (double)wheel_speeds[1],
            (double)wheel_speeds[2],
            (double)wheel_speeds[3]);
}

/* ============================== 主函数 ============================== */
int main(void)
{
    HAL_Init();
    Debug_DisableWatchdogs();
    SystemClock_Config();
    DWT_Init(168);

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
    ChassisForceZero();
    system_start_time = HAL_GetTick();

    GimbalMotorsInit();

    gimbal_startup_hold_active = 1;
    gimbal_startup_hold_tick = HAL_GetTick();

    vt_data = VT_Init(&VT_UART_HANDLE);



    LOGINFO("[integrated-vt-km] demo initialized");
    LOGINFO("[integrated-vt-km] VT UART=%s", VT_UART_LABEL);
    LOGINFO("[integrated] CAN1: chassis 1-4, loader 5, yaw 2; CAN2: friction 1/2, pitch 5");

    uint32_t last_chassis = 0;
    uint32_t last_gimbal = 0;
    uint32_t last_shoot = 0;
    uint32_t last_ins = 0;

    while (1) {
        DaemonTask();
        DJIMotorControl();

        uint32_t now = HAL_GetTick();
        if (now - last_ins >= INS_TASK_PERIOD_MS) {
            last_ins = now;
            INS_Task();
        }
        if (now - last_chassis >= CHASSIS_UPDATE_INTERVAL_MS) {
            last_chassis = now;
            ChassisProcessRC();
            ChassisUpdateKinematics();
        }
        if (now - last_gimbal >= GIMBAL_UPDATE_INTERVAL_MS) {
            last_gimbal = now;
            GimbalProcessRC();
            GimbalUpdate();
            GimbalLogPosition();
        }
        if (now - last_shoot >= SHOOT_UPDATE_INTERVAL_MS) {
            last_shoot = now;
            ShootProcessRC();
            UpdateFrictionControl();
            UpdateLoaderControl();
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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
