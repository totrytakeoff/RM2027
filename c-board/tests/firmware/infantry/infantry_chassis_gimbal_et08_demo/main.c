/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Infantry ET08 chassis-gimbal fusion demo
 ******************************************************************************
 * @attention
 *
 * 控制定义:
 * - 左摇杆: 底盘平移,方向始终以云台当前朝向为正方向
 * - 右摇杆: 云台控制(右X=Yaw, 右Y=Pitch)
 * - SD上: FOLLOW, 底盘跟随云台
 * - SD下: SEPARATE, 底盘不跟随云台
 *
 * 关键分工:
 * - 云台Yaw: 始终使用IMU YawTotalAngle闭环,对齐世界坐标系
 * - 底盘: 使用Yaw电机编码器反馈计算云台相对底盘夹角,完成坐标变换与FOLLOW闭环
 *
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
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dji_motor.h"
#include "et08_remote.h"
#include "ins_task.h"
#include "minimal_config.h"

/* Private define ------------------------------------------------------------*/
#define CHASSIS_MOTOR_COUNT 4U
#define TELEMETRY_RX_DUMMY 32U
#define TELEMETRY_PERIOD_MS 100U
#define VOFA_PERIOD_MS 20U
#define INS_TASK_PERIOD_MS 1U
#define DAEMON_TASK_PERIOD_MS 1U
#define MOTOR_CONTROL_PERIOD_MS 2U
#define RC_INPUT_DEADZONE 50
#define WHEEL_SPEED_LIMIT_DPS 30000.0f
#define ET08_SWITCH_POS_UP 0U
#define ET08_SWITCH_POS_DOWN 2U
#define PITCH_BRAKE_SPEED_EPS 20.0f
#define PITCH_BRAKE_STABLE_COUNT 3U
#define PITCH_BRAKE_TIMEOUT_MS 120U
#define PITCH_RELEASE_SPEED_PREDICT_GAIN 0.02f

/*
 * yaw编码器原始相对角的物理参考.
 * 该值仅用于从编码器单圈角恢复“云台相对底盘”的原始机械夹角.
 */
#define YAW_CHASSIS_ALIGN_ECD 2711U
#define YAW_ALIGN_ANGLE_DEG (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)

/*
 * 逻辑零位定义:
 * - IMU逻辑零位: 云台指向底盘默认正方向时, IMU YawTotalAngle 的参考值
 * - YawOffset逻辑零位: 同一姿态下, 编码器原始相对角的参考值
 *
 * 上述两项来自用户最新采集的目标零位数据.
 */
#define IMU_YAW_LOGIC_ZERO_TOTAL_DEG 364.30f
#define YAW_OFFSET_LOGIC_ZERO_DEG (-118.915f)

/*
 * 现场确认的底盘轮子物理排布(云台零位朝上看):
 *   2 3
 *   1 4
 * 即:
 * - FL = 2
 * - FR = 3
 * - BL = 1
 * - BR = 4
 *
 * 不直接沿用 minimal_config.h 里的 FR/FL/BR/BL 命名,避免解算正确但下发到错误电机.
 */
#define CHASSIS_MOTOR_ID_FL 2U
#define CHASSIS_MOTOR_ID_FR 3U
#define CHASSIS_MOTOR_ID_BL 1U
#define CHASSIS_MOTOR_ID_BR 4U

#define GIMBAL_YAW_SPEED_SCALE_DEMO 0.23f
#define DEMO_GIMBAL_SPEED_DEADZONE 30.0f
#define DEMO_ET08_PITCH_SPEED_SCALE 0.15f

/*
 * 小陀螺模式: 在SEPARATE模式下叠加自旋速度
 * SA上: 开启小陀螺
 * SA下: 关闭小陀螺
 */
#define DEMO_SPIN_ROTATE_SPEED_RAD_S 40.0f  /* 小陀螺自旋角速度 rad/s */

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    CHASSIS_MODE_FOLLOW = 0,
    CHASSIS_MODE_SEPARATE = 1,
} ChassisMode_t;

typedef enum {
    AXIS_CTRL_ANGLE = 0,
    AXIS_CTRL_SPEED = 1,
    AXIS_CTRL_BRAKE = 2,
} AxisCtrlMode_t;

typedef struct {
    /*
     * 与 infantry_omni_demo / minimal_chassis 保持一致:
     * - vx: 横移, 左为正
     * - vy: 前后, 前为正
     */
    float vx_cmd;
    float vy_cmd;
    float wz_cmd;
    float vx_body;
    float vy_body;
    float imu_yaw_logic_deg;
    float yaw_offset_raw_deg;
    float yaw_offset_deg;
} ChassisCmd_t;

/* Private variables ---------------------------------------------------------*/
static ET08_Ctrl_t *et08_ctrl = NULL;
static USARTInstance *telemetry_usart = NULL;
static attitude_t *gimbal_imu = NULL;

static DJIMotorInstance *motor_yaw = NULL;
static DJIMotorInstance *motor_pitch = NULL;
static DJIMotorInstance *motor_fr = NULL;
static DJIMotorInstance *motor_fl = NULL;
static DJIMotorInstance *motor_br = NULL;
static DJIMotorInstance *motor_bl = NULL;

static uint8_t gimbal_enabled = 0U;
static uint8_t chassis_enabled = 0U;
static uint8_t last_online_state = 0U;
static ChassisMode_t chassis_mode = CHASSIS_MODE_FOLLOW;
static ChassisMode_t last_chassis_mode = CHASSIS_MODE_FOLLOW;
static uint8_t spin_mode_enabled = 0U;  /* 小陀螺模式: SA上开, SA下关 */
static uint8_t last_spin_mode_enabled = 0U;

static float yaw_speed_ref = 0.0f;
static float pitch_speed_ref = 0.0f;
static float yaw_hold_ref = 0.0f;
static float pitch_hold_ref = 0.0f;
static float pitch_release_hold_ref = 0.0f;
static float pitch_current_ff = 0.0f;
static float pitch_imu_speed_fdb = 0.0f;
static AxisCtrlMode_t pitch_ctrl_mode = AXIS_CTRL_ANGLE;

static float wheel_speed_ref[CHASSIS_MOTOR_COUNT] = {0.0f};
static ChassisCmd_t last_cmd = {0.0f};
static float filtered_vx = 0.0f;
static float filtered_vy = 0.0f;
static float filtered_wz = 0.0f;
static float follow_wz_integral = 0.0f;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

static void Debug_DisableWatchdogs(void);
static void InitTelemetryUsart(void);
static void TelemetrySendBuffer(const uint8_t *buffer, uint16_t len);
static void TelemetrySendString(const char *str);
static void TelemetryPrintf(const char *fmt, ...);

static int16_t ApplyDeadzone(int16_t v, int16_t deadzone);
static float ClampFloat(float value, float min_value, float max_value);
static float WrapAngleDeg180(float angle_deg);
static float GetImuYawLogicDeg(void);
static uint8_t ET08_MapUpperSwitchPos(uint8_t state);
static void OmniInverseKinematics(float vx, float vy, float wz, float out[4]);

static void GimbalMotorsInit(void);
static void ChassisMotorsInit(void);
static void ResetPidState(PIDInstance *pid);
static void ResetPitchPidRuntime(void);
static void UpdatePitchGravityFeedforward(void);
static float GetPitchSpeedFeedback(void);
static bool GimbalUpdateFromET08(float dt_s);
static float GetYawOffsetRawDeg(void);
static float GetYawOffsetLogicDeg(void);
static bool BuildChassisCommandFromEt08(ChassisCmd_t *cmd);
static void ChassisStop(void);
static void GimbalStop(void);
static void ChassisApplyCommand(const ChassisCmd_t *cmd);
static void SendTelemetry(void);
static void SendVofaFrame(void);

/* Private user code ---------------------------------------------------------*/
static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

static void InitTelemetryUsart(void)
{
    USART_Init_Config_s config = {
        .module_callback = NULL,
        .recv_buff_size = TELEMETRY_RX_DUMMY,
        .usart_handle = &huart6,
    };
    telemetry_usart = USARTRegister(&config);
    TelemetrySendString("[chs_gmb_et08] USART6 telemetry ready\r\n");
}

static void TelemetrySendBuffer(const uint8_t *buffer, uint16_t len)
{
    if (telemetry_usart == NULL || buffer == NULL || len == 0U)
        return;

    USARTSend(telemetry_usart, (uint8_t *)buffer, len, USART_TRANSFER_BLOCKING);
}

static void TelemetrySendString(const char *str)
{
    if (str == NULL)
        return;
    TelemetrySendBuffer((const uint8_t *)str, (uint16_t)strlen(str));
}

static void TelemetryPrintf(const char *fmt, ...)
{
    if (telemetry_usart == NULL || fmt == NULL)
        return;

    char msg[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    if (n <= 0)
        return;

    TelemetrySendBuffer((const uint8_t *)msg,
                        (uint16_t)((n < (int)sizeof(msg)) ? n : (int)(sizeof(msg) - 1U)));
}

static int16_t ApplyDeadzone(int16_t v, int16_t deadzone)
{
    if (v > -deadzone && v < deadzone) {
        return 0;
    }
    return v;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float WrapAngleDeg180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float GetImuYawLogicDeg(void)
{
    if (gimbal_imu == NULL) {
        return 0.0f;
    }

    return WrapAngleDeg180(gimbal_imu->YawTotalAngle - IMU_YAW_LOGIC_ZERO_TOTAL_DEG);
}

static uint8_t ET08_MapUpperSwitchPos(uint8_t state)
{
    if (state == 0xFFU || state > 5U) {
        return 0xFFU;
    }
    return (state <= 2U) ? ET08_SWITCH_POS_UP : ET08_SWITCH_POS_DOWN;
}

static void OmniInverseKinematics(float vx, float vy, float wz, float out[4])
{
    const float l = CHASSIS_WHEEL_BASE * 0.5f;
    /*
     * 该车底盘平移基向量以实测结果重建:
     * - vx(左为正)  -> FR/FL/BR/BL = [+,+,-,-]
     * - vy(前为正)  -> FR/FL/BR/BL = [-,+,-,+]
     * - wz(CCW为正) -> 四轮同号
     */
    const float v_fr = vx - vy - l * wz;
    const float v_fl = vx + vy - l * wz;
    const float v_br = -vx - vy - l * wz;
    const float v_bl = -vx + vy - l * wz;

    out[0] = v_fr / CHASSIS_WHEEL_RADIUS;
    out[1] = v_fl / CHASSIS_WHEEL_RADIUS;
    out[2] = v_br / CHASSIS_WHEEL_RADIUS;
    out[3] = v_bl / CHASSIS_WHEEL_RADIUS;
}

static void GimbalMotorsInit(void)
{
    Motor_Init_Config_s yaw_config = {
        .motor_type = GM6020,
        .can_init_config = {
            .can_handle = &YAW_CAN,
            .tx_id = YAW_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = YAW_SEPARATE_ANGLE_KP,
                .Ki = YAW_SEPARATE_ANGLE_KI,
                .Kd = YAW_SEPARATE_ANGLE_KD,
                .IntegralLimit = 500.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = YAW_SEPARATE_ANGLE_MAX_OUT,
            },
            .speed_PID = {
                .Kp = YAW_SEPARATE_SPEED_KP,
                .Ki = YAW_SEPARATE_SPEED_KI,
                .Kd = YAW_SEPARATE_SPEED_KD,
                .IntegralLimit = 1000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = YAW_SEPARATE_SPEED_MAX_OUT,
            },
            .other_angle_feedback_ptr = (gimbal_imu != NULL) ? &gimbal_imu->YawTotalAngle : NULL,
            .other_speed_feedback_ptr = (gimbal_imu != NULL) ? &gimbal_imu->Gyro[2] : NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    Motor_Init_Config_s pitch_config = {
        .motor_type = GM6020,
        .can_init_config = {
            .can_handle = &PITCH_CAN,
            .tx_id = PITCH_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = PITCH_ANGLE_KP,
                .Ki = PITCH_ANGLE_KI,
                .Kd = PITCH_ANGLE_KD,
                .IntegralLimit = 500.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = PITCH_ANGLE_MAX_OUT,
            },
            .speed_PID = {
                .Kp = PITCH_SPEED_KP,
                .Ki = PITCH_SPEED_KI,
                .Kd = PITCH_SPEED_KD,
                .IntegralLimit = 1000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = PITCH_SPEED_MAX_OUT,
            },
            .other_speed_feedback_ptr = &pitch_imu_speed_fdb,
            .current_feedforward_ptr = &pitch_current_ff,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    motor_yaw = DJIMotorInit(&yaw_config);
    motor_pitch = DJIMotorInit(&pitch_config);

    if (motor_yaw != NULL) {
        DJIMotorOuterLoop(motor_yaw, ANGLE_LOOP);
        DJIMotorStop(motor_yaw);
        yaw_hold_ref = (gimbal_imu != NULL) ? gimbal_imu->YawTotalAngle : 0.0f;
    }
    if (motor_pitch != NULL) {
        DJIMotorOuterLoop(motor_pitch, SPEED_LOOP);
        DJIMotorStop(motor_pitch);
        pitch_hold_ref = motor_pitch->measure.total_angle;
        pitch_release_hold_ref = pitch_hold_ref;
    }

    TelemetrySendString("[chs_gmb_et08] gimbal motors initialized\r\n");
}

static void ChassisMotorsInit(void)
{
    Motor_Init_Config_s config = {
        .motor_type = M3508,
        .can_init_config = {
            .can_handle = &CHASSIS_CAN,
            .tx_id = CHASSIS_MOTOR_ID_FR,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = CHASSIS_SPEED_KP,
                .Ki = CHASSIS_SPEED_KI,
                .Kd = CHASSIS_SPEED_KD,
                .MaxOut = CHASSIS_SPEED_MAX_OUT,
                .IntegralLimit = 1000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
            .current_PID = {
                .Kp = 0.35f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 1000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .MaxOut = 8000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    motor_fr = DJIMotorInit(&config);
    if (motor_fr != NULL) {
        DJIMotorOuterLoop(motor_fr, SPEED_LOOP);
        DJIMotorStop(motor_fr);
    }

    config.can_init_config.tx_id = CHASSIS_MOTOR_ID_FL;
    motor_fl = DJIMotorInit(&config);
    if (motor_fl != NULL) {
        DJIMotorOuterLoop(motor_fl, SPEED_LOOP);
        DJIMotorStop(motor_fl);
    }

    config.can_init_config.tx_id = CHASSIS_MOTOR_ID_BR;
    motor_br = DJIMotorInit(&config);
    if (motor_br != NULL) {
        DJIMotorOuterLoop(motor_br, SPEED_LOOP);
        DJIMotorStop(motor_br);
    }

    config.can_init_config.tx_id = CHASSIS_MOTOR_ID_BL;
    motor_bl = DJIMotorInit(&config);
    if (motor_bl != NULL) {
        DJIMotorOuterLoop(motor_bl, SPEED_LOOP);
        DJIMotorStop(motor_bl);
    }

    TelemetrySendString("[chs_gmb_et08] chassis motors initialized\r\n");
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
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
    pid->DWT_CNT = 0U;
    pid->dt = 0.0f;
}

static void ResetPitchPidRuntime(void)
{
    if (motor_pitch == NULL) {
        return;
    }

    ResetPidState(&motor_pitch->motor_controller.angle_PID);
    ResetPidState(&motor_pitch->motor_controller.speed_PID);
    ResetPidState(&motor_pitch->motor_controller.current_PID);
}

static void UpdatePitchGravityFeedforward(void)
{
    if (motor_pitch == NULL) {
        pitch_current_ff = 0.0f;
        return;
    }

    float pitch_ff_raw =
        PITCH_GRAVITY_FF_K *
        sinf((motor_pitch->measure.total_angle - PITCH_GRAVITY_FF_OFFSET_DEG) * (float)M_PI / 180.0f);
    pitch_ff_raw = ClampFloat(pitch_ff_raw, -PITCH_GRAVITY_FF_MAX, PITCH_GRAVITY_FF_MAX);
    pitch_current_ff = pitch_current_ff * PITCH_FF_LPF + pitch_ff_raw * (1.0f - PITCH_FF_LPF);
}

static float GetPitchSpeedFeedback(void)
{
    if (gimbal_imu != NULL) {
        pitch_imu_speed_fdb = gimbal_imu->Gyro[0];
    } else if (motor_pitch != NULL) {
        pitch_imu_speed_fdb = motor_pitch->measure.speed_aps;
    } else {
        pitch_imu_speed_fdb = 0.0f;
    }
    return pitch_imu_speed_fdb;
}

static bool GimbalUpdateFromET08(float dt_s)
{
    static uint8_t pitch_cmd_active = 0U;
    static uint8_t pitch_enter_cnt = 0U;
    static uint8_t pitch_exit_cnt = 0U;
    static uint8_t pitch_brake_stable_count = 0U;
    static uint32_t pitch_brake_start_ms = 0U;
    static AxisCtrlMode_t last_pitch_ctrl_mode = AXIS_CTRL_ANGLE;

    if (et08_ctrl == NULL || gimbal_imu == NULL || motor_yaw == NULL || motor_pitch == NULL) {
        return false;
    }

    if (!ET08_IsOnline() || et08_ctrl->failsafe || et08_ctrl->frame_lost) {
        return false;
    }

    if (!gimbal_enabled) {
        DJIMotorEnable(motor_yaw);
        DJIMotorEnable(motor_pitch);
        gimbal_enabled = 1U;
        yaw_hold_ref = gimbal_imu->YawTotalAngle;
        pitch_hold_ref = motor_pitch->measure.total_angle;
        pitch_release_hold_ref = pitch_hold_ref;
    }

    UpdatePitchGravityFeedforward();
    GetPitchSpeedFeedback();

    const int16_t yaw_in = ApplyDeadzone(et08_ctrl->right.x, RC_INPUT_DEADZONE);
    const int16_t pitch_in = ApplyDeadzone(et08_ctrl->right.y, RC_INPUT_DEADZONE);
    const int16_t pitch_abs = (pitch_in >= 0) ? pitch_in : (int16_t)(-pitch_in);
    const uint32_t now_ms = DWT_GetTimeline_ms();

    {
        float yaw_ratio = ClampFloat((float)yaw_in / RC_STICK_SCALE, -1.0f, 1.0f);
        yaw_speed_ref = yaw_ratio * GM6020_SPEED_MAX * GIMBAL_YAW_SPEED_SCALE_DEMO;
        if (fabsf(yaw_speed_ref) < DEMO_GIMBAL_SPEED_DEADZONE) {
            yaw_speed_ref = 0.0f;
        }
        yaw_hold_ref += yaw_speed_ref * dt_s;
    }

    DJIMotorChangeFeed(motor_yaw, ANGLE_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(motor_yaw, SPEED_LOOP, OTHER_FEED);
    DJIMotorOuterLoop(motor_yaw, ANGLE_LOOP);
    DJIMotorSetRef(motor_yaw, yaw_hold_ref);

    {
        float pitch_ratio = ClampFloat((float)pitch_in / RC_STICK_SCALE, -1.0f, 1.0f);
        pitch_speed_ref = pitch_ratio * GM6020_SPEED_MAX * DEMO_ET08_PITCH_SPEED_SCALE;
        if (fabsf(pitch_speed_ref) < DEMO_GIMBAL_SPEED_DEADZONE) {
            pitch_speed_ref = 0.0f;
        }
    }

    if (pitch_cmd_active != 0U) {
        if (pitch_abs <= 90) {
            if (pitch_exit_cnt < 255U) {
                pitch_exit_cnt++;
            }
        } else {
            pitch_exit_cnt = 0U;
        }
        pitch_enter_cnt = 0U;
        if (pitch_exit_cnt >= 10U) {
            pitch_exit_cnt = 0U;
            pitch_cmd_active = 0U;
        }
    } else {
        if (pitch_abs >= 180) {
            if (pitch_enter_cnt < 255U) {
                pitch_enter_cnt++;
            }
        } else {
            pitch_enter_cnt = 0U;
        }
        pitch_exit_cnt = 0U;
        if (pitch_enter_cnt >= 4U) {
            pitch_enter_cnt = 0U;
            pitch_cmd_active = 1U;
        }
    }

    if (pitch_cmd_active != 0U) {
        if (pitch_ctrl_mode != AXIS_CTRL_SPEED) {
            ResetPitchPidRuntime();
        }
        pitch_ctrl_mode = AXIS_CTRL_SPEED;
        pitch_brake_stable_count = 0U;
        pitch_brake_start_ms = 0U;
    } else if (pitch_ctrl_mode == AXIS_CTRL_SPEED) {
        pitch_release_hold_ref = motor_pitch->measure.total_angle + GetPitchSpeedFeedback() * PITCH_RELEASE_SPEED_PREDICT_GAIN;
        pitch_hold_ref = pitch_release_hold_ref;
        pitch_ctrl_mode = AXIS_CTRL_BRAKE;
        pitch_brake_stable_count = 0U;
        pitch_brake_start_ms = now_ms;
        ResetPidState(&motor_pitch->motor_controller.speed_PID);
    } else if (pitch_ctrl_mode == AXIS_CTRL_BRAKE) {
        if (fabsf(GetPitchSpeedFeedback()) <= PITCH_BRAKE_SPEED_EPS) {
            if (pitch_brake_stable_count < 255U) {
                pitch_brake_stable_count++;
            }
        } else {
            pitch_brake_stable_count = 0U;
        }
        if (pitch_brake_stable_count >= PITCH_BRAKE_STABLE_COUNT ||
            (now_ms - pitch_brake_start_ms) >= PITCH_BRAKE_TIMEOUT_MS) {
            pitch_hold_ref = pitch_release_hold_ref;
            pitch_ctrl_mode = AXIS_CTRL_ANGLE;
            ResetPitchPidRuntime();
        }
    }

    if (pitch_ctrl_mode == AXIS_CTRL_SPEED) {
        DJIMotorOuterLoop(motor_pitch, SPEED_LOOP);
        DJIMotorSetRef(motor_pitch, pitch_speed_ref);
        if (fabsf(pitch_speed_ref) >= 300.0f && pitch_abs >= 220) {
            pitch_hold_ref = motor_pitch->measure.total_angle;
        }
    } else if (pitch_ctrl_mode == AXIS_CTRL_BRAKE) {
        DJIMotorChangeFeed(motor_pitch, SPEED_LOOP, (gimbal_imu != NULL) ? OTHER_FEED : MOTOR_FEED);
        DJIMotorOuterLoop(motor_pitch, SPEED_LOOP);
        DJIMotorSetRef(motor_pitch, 0.0f);
    } else {
        DJIMotorChangeFeed(motor_pitch, SPEED_LOOP, (gimbal_imu != NULL) ? OTHER_FEED : MOTOR_FEED);
        DJIMotorOuterLoop(motor_pitch, ANGLE_LOOP);
        DJIMotorSetRef(motor_pitch, pitch_hold_ref);
    }

    if (pitch_ctrl_mode != last_pitch_ctrl_mode) {
        TelemetryPrintf("[chs_gmb_et08] pitch loop -> %d\r\n", (int)pitch_ctrl_mode);
        last_pitch_ctrl_mode = pitch_ctrl_mode;
    }

    return true;
}

static float GetYawOffsetRawDeg(void)
{
    float angle_single_round;

    if (motor_yaw == NULL) {
        return 0.0f;
    }

    angle_single_round = motor_yaw->measure.angle_single_round;
    return WrapAngleDeg180(angle_single_round - YAW_ALIGN_ANGLE_DEG);
}

static float GetYawOffsetLogicDeg(void)
{
    return WrapAngleDeg180(GetYawOffsetRawDeg() - YAW_OFFSET_LOGIC_ZERO_DEG);
}

static bool BuildChassisCommandFromEt08(ChassisCmd_t *cmd)
{
    float cos_theta;
    float sin_theta;
    float theta_deg;
    float yaw_offset_deg;
    float yaw_offset_speed_deg = 0.0f;
    static float last_dt_s = MAIN_LOOP_PERIOD_MS / 1000.0f;

    if (cmd == NULL || et08_ctrl == NULL || motor_yaw == NULL) {
        return false;
    }

    if (!ET08_IsOnline() || et08_ctrl->failsafe || et08_ctrl->frame_lost) {
        /* 离线时清零积分 */
        follow_wz_integral = 0.0f;
        return false;
    }

    {
        const int16_t left_x = ApplyDeadzone(et08_ctrl->left.x, RC_INPUT_DEADZONE);
        const int16_t left_y = ApplyDeadzone(et08_ctrl->left.y, RC_INPUT_DEADZONE);
        const uint8_t sd_pos = ET08_MapUpperSwitchPos(et08_ctrl->switch_sd_sc_state);
        const uint8_t sa_pos = ET08_MapUpperSwitchPos(et08_ctrl->switch_sa_sb_state);

        /*
         * 与 infantry_omni_demo 对齐:
         * - 左摇杆 X -> 横移 vx (左为正)
         * - 左摇杆 Y -> 前后 vy (前为正)
         */
        cmd->vx_cmd = ClampFloat(-(float)left_x / RC_STICK_SCALE * CHASSIS_MAX_VX, -CHASSIS_MAX_VX, CHASSIS_MAX_VX);
        cmd->vy_cmd = ClampFloat((float)left_y / RC_STICK_SCALE * CHASSIS_MAX_VY, -CHASSIS_MAX_VY, CHASSIS_MAX_VY);
        chassis_mode = (sd_pos == ET08_SWITCH_POS_UP) ? CHASSIS_MODE_FOLLOW : CHASSIS_MODE_SEPARATE;
        
        /* 小陀螺模式: SA上开, SA下关 (仅在SEPARATE模式下生效) */
        spin_mode_enabled = (sa_pos == ET08_SWITCH_POS_UP) ? 1U : 0U;
    }

    yaw_offset_deg = GetYawOffsetLogicDeg();
    cmd->imu_yaw_logic_deg = GetImuYawLogicDeg();
    cmd->yaw_offset_raw_deg = GetYawOffsetRawDeg();
    cmd->yaw_offset_deg = yaw_offset_deg;
    /*
     * 将“云台坐标系下的平移指令”旋转到底盘坐标系.
     *
     * 这里统一采用与 omni demo 一致的轴定义:
     * - vx: 横移, 左为正
     * - vy: 前后, 前为正
     *
     * 若云台相对底盘逆时针偏转 yaw_offset_deg，则云台系到车体系的变换为:
     *   vx_body =  vx_cmd * cos(theta) + vy_cmd * sin(theta)
     *   vy_body = -vx_cmd * sin(theta) + vy_cmd * cos(theta)
     */
    theta_deg = -yaw_offset_deg;
    cos_theta = cosf(theta_deg * (float)M_PI / 180.0f);
    sin_theta = sinf(theta_deg * (float)M_PI / 180.0f);

    cmd->vx_body = cmd->vx_cmd * cos_theta + cmd->vy_cmd * sin_theta;
    cmd->vy_body = -cmd->vx_cmd * sin_theta + cmd->vy_cmd * cos_theta;

    if (fabsf(cmd->vx_body) < CHASSIS_DEADZONE_VX) {
        cmd->vx_body = 0.0f;
    }
    if (fabsf(cmd->vy_body) < CHASSIS_DEADZONE_VY) {
        cmd->vy_body = 0.0f;
    }

    if (chassis_mode == CHASSIS_MODE_FOLLOW) {
        yaw_offset_speed_deg = motor_yaw->measure.speed_aps;
        
        /* 积分项累积与限幅 */
        follow_wz_integral += yaw_offset_deg * last_dt_s;
        follow_wz_integral = ClampFloat(follow_wz_integral, 
                                        -CHASSIS_FOLLOW_WZ_I_MAX / CHASSIS_FOLLOW_WZ_KI,
                                        CHASSIS_FOLLOW_WZ_I_MAX / CHASSIS_FOLLOW_WZ_KI);
        
        /* PID控制律: wz = -(Kp*e + Ki*∫e + Kd*ė) */
        cmd->wz_cmd = -(CHASSIS_FOLLOW_WZ_KP * yaw_offset_deg + 
                        CHASSIS_FOLLOW_WZ_KI * follow_wz_integral + 
                        CHASSIS_FOLLOW_WZ_KD * yaw_offset_speed_deg) *
                      ((float)M_PI / 180.0f);
        cmd->wz_cmd = ClampFloat(cmd->wz_cmd, -CHASSIS_FOLLOW_WZ_MAX, CHASSIS_FOLLOW_WZ_MAX);
    } else {
        /* SEPARATE模式清零积分 */
        follow_wz_integral = 0.0f;
        
        /* 小陀螺模式: 叠加固定自旋速度 */
        if (spin_mode_enabled) {
            cmd->wz_cmd = DEMO_SPIN_ROTATE_SPEED_RAD_S;
        } else {
            cmd->wz_cmd = 0.0f;
        }
    }

    if (fabsf(cmd->wz_cmd) < CHASSIS_DEADZONE_WZ) {
        cmd->wz_cmd = 0.0f;
    }

    return true;
}

static void GimbalStop(void)
{
    yaw_speed_ref = 0.0f;
    pitch_speed_ref = 0.0f;
    pitch_ctrl_mode = AXIS_CTRL_ANGLE;
    gimbal_enabled = 0U;
    pitch_current_ff = 0.0f;
    pitch_imu_speed_fdb = 0.0f;
    ResetPitchPidRuntime();

    if (motor_yaw != NULL) {
        DJIMotorStop(motor_yaw);
    }
    if (motor_pitch != NULL) {
        DJIMotorStop(motor_pitch);
    }
}

static void ChassisStop(void)
{
    filtered_vx = 0.0f;
    filtered_vy = 0.0f;
    filtered_wz = 0.0f;
    memset(wheel_speed_ref, 0, sizeof(wheel_speed_ref));
    chassis_enabled = 0U;

    if (motor_fr != NULL) {
        DJIMotorSetRef(motor_fr, 0.0f);
        DJIMotorStop(motor_fr);
    }
    if (motor_fl != NULL) {
        DJIMotorSetRef(motor_fl, 0.0f);
        DJIMotorStop(motor_fl);
    }
    if (motor_br != NULL) {
        DJIMotorSetRef(motor_br, 0.0f);
        DJIMotorStop(motor_br);
    }
    if (motor_bl != NULL) {
        DJIMotorSetRef(motor_bl, 0.0f);
        DJIMotorStop(motor_bl);
    }
}

static void ChassisApplyCommand(const ChassisCmd_t *cmd)
{
    float wheel_speed_rad_s[CHASSIS_MOTOR_COUNT] = {0.0f};

    if (cmd == NULL) {
        return;
    }

    if (!chassis_enabled) {
        if (motor_fr != NULL)
            DJIMotorEnable(motor_fr);
        if (motor_fl != NULL)
            DJIMotorEnable(motor_fl);
        if (motor_br != NULL)
            DJIMotorEnable(motor_br);
        if (motor_bl != NULL)
            DJIMotorEnable(motor_bl);
        chassis_enabled = 1U;
    }

    last_cmd = *cmd;
    filtered_vx = filtered_vx * CHASSIS_SPEED_FILTER_COEF + cmd->vx_body * (1.0f - CHASSIS_SPEED_FILTER_COEF);
    filtered_vy = filtered_vy * CHASSIS_SPEED_FILTER_COEF + cmd->vy_body * (1.0f - CHASSIS_SPEED_FILTER_COEF);
    filtered_wz = filtered_wz * CHASSIS_SPEED_FILTER_COEF + cmd->wz_cmd * (1.0f - CHASSIS_SPEED_FILTER_COEF);

    OmniInverseKinematics(filtered_vx, filtered_vy, filtered_wz, wheel_speed_rad_s);

    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        float speed_dps = wheel_speed_rad_s[i] * 180.0f / (float)M_PI;
        float speed_deadzone = (chassis_mode == CHASSIS_MODE_FOLLOW) ? CHASSIS_FOLLOW_SPEED_DEADZONE : CHASSIS_SPEED_DEADZONE;
        speed_dps *= CHASSIS_SPEED_SCALE;
        if (fabsf(speed_dps) < speed_deadzone) {
            speed_dps = 0.0f;
        }
        wheel_speed_ref[i] = ClampFloat(speed_dps, -WHEEL_SPEED_LIMIT_DPS, WHEEL_SPEED_LIMIT_DPS);
    }

    if (motor_fr != NULL)
        DJIMotorSetRef(motor_fr, wheel_speed_ref[0]);
    if (motor_fl != NULL)
        DJIMotorSetRef(motor_fl, wheel_speed_ref[1]);
    if (motor_br != NULL)
        DJIMotorSetRef(motor_br, wheel_speed_ref[2]);
    if (motor_bl != NULL)
        DJIMotorSetRef(motor_bl, wheel_speed_ref[3]);
}

static void SendTelemetry(void)
{
    static uint32_t seq = 0U;
    seq++;

    TelemetryPrintf("[chs_gmb_et08] #%lu online=%u mode=%s raw(lx=%d ly=%d rx=%d ry=%d)\r\n",
                    (unsigned long)seq,
                    (unsigned int)ET08_IsOnline(),
                    (chassis_mode == CHASSIS_MODE_FOLLOW) ? "FOLLOW" : "SEPARATE",
                    (int)((et08_ctrl != NULL) ? et08_ctrl->left.x : 0),
                    (int)((et08_ctrl != NULL) ? et08_ctrl->left.y : 0),
                    (int)((et08_ctrl != NULL) ? et08_ctrl->right.x : 0),
                    (int)((et08_ctrl != NULL) ? et08_ctrl->right.y : 0));

    TelemetryPrintf("[chs_gmb_et08] yaw imu(ref_x10=%ld fdb_x10=%ld logic_x10=%ld gz_x10=%ld) enc(total_x10=%ld raw_off_x10=%ld logic_off_x10=%ld)\r\n",
                    (long)(yaw_hold_ref * 10.0f),
                    (long)((gimbal_imu != NULL) ? (gimbal_imu->YawTotalAngle * 10.0f) : 0.0f),
                    (long)(last_cmd.imu_yaw_logic_deg * 10.0f),
                    (long)((gimbal_imu != NULL) ? (gimbal_imu->Gyro[2] * 10.0f) : 0.0f),
                    (long)((motor_yaw != NULL) ? (motor_yaw->measure.total_angle * 10.0f) : 0.0f),
                    (long)(last_cmd.yaw_offset_raw_deg * 10.0f),
                    (long)(last_cmd.yaw_offset_deg * 10.0f));

    TelemetryPrintf("[chs_gmb_et08] pitch(ref_spd_x10=%ld hold_x10=%ld enc_x10=%ld imu_gx_x10=%ld ff_x10=%ld)\r\n",
                    (long)(pitch_speed_ref * 10.0f),
                    (long)(pitch_hold_ref * 10.0f),
                    (long)((motor_pitch != NULL) ? (motor_pitch->measure.total_angle * 10.0f) : 0.0f),
                    (long)((gimbal_imu != NULL) ? (gimbal_imu->Gyro[0] * 10.0f) : 0.0f),
                    (long)(pitch_current_ff * 10.0f));

    TelemetryPrintf("[chs_gmb_et08] chs cmd(gmb_vx=%.2f gmb_vy=%.2f off=%.2f body_vx=%.2f body_vy=%.2f wz=%.2f) filt(vx=%.2f vy=%.2f wz=%.2f)\r\n",
                    last_cmd.vx_cmd, last_cmd.vy_cmd, last_cmd.yaw_offset_deg,
                    last_cmd.vx_body, last_cmd.vy_body, last_cmd.wz_cmd,
                    filtered_vx, filtered_vy, filtered_wz);

    TelemetryPrintf("[chs_gmb_et08] wheel_ref(fr=%.1f fl=%.1f br=%.1f bl=%.1f) fdb(fr=%.1f fl=%.1f br=%.1f bl=%.1f)\r\n",
                    wheel_speed_ref[0], wheel_speed_ref[1], wheel_speed_ref[2], wheel_speed_ref[3],
                    (motor_fr != NULL) ? motor_fr->measure.speed_aps : 0.0f,
                    (motor_fl != NULL) ? motor_fl->measure.speed_aps : 0.0f,
                    (motor_br != NULL) ? motor_br->measure.speed_aps : 0.0f,
                    (motor_bl != NULL) ? motor_bl->measure.speed_aps : 0.0f);
}

static void SendVofaFrame(void)
{
    float ch[32];
    uint8_t txbuf[sizeof(ch) + 4U];
    static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

    ch[0] = (float)HAL_GetTick();
    ch[1] = (float)ET08_IsOnline();
    ch[2] = (float)chassis_mode;
    ch[3] = yaw_hold_ref;
    ch[4] = (gimbal_imu != NULL) ? gimbal_imu->YawTotalAngle : 0.0f;
    ch[5] = last_cmd.imu_yaw_logic_deg;
    ch[6] = (gimbal_imu != NULL) ? gimbal_imu->Gyro[2] : 0.0f;
    ch[7] = (motor_yaw != NULL) ? motor_yaw->measure.total_angle : 0.0f;
    ch[8] = (motor_yaw != NULL) ? motor_yaw->measure.angle_single_round : 0.0f;
    ch[9] = last_cmd.yaw_offset_raw_deg;
    ch[10] = last_cmd.vx_cmd;
    ch[11] = last_cmd.vy_cmd;
    ch[12] = last_cmd.vx_body;
    ch[13] = last_cmd.vy_body;
    ch[14] = last_cmd.wz_cmd;
    ch[15] = filtered_vx;
    ch[16] = filtered_vy;
    ch[17] = filtered_wz;
    ch[18] = wheel_speed_ref[0];
    ch[19] = (motor_fr != NULL) ? motor_fr->measure.speed_aps : 0.0f;
    ch[20] = wheel_speed_ref[1];
    ch[21] = (motor_fl != NULL) ? motor_fl->measure.speed_aps : 0.0f;
    ch[22] = wheel_speed_ref[2];
    ch[23] = (motor_br != NULL) ? motor_br->measure.speed_aps : 0.0f;
    ch[24] = wheel_speed_ref[3];
    ch[25] = (motor_bl != NULL) ? motor_bl->measure.speed_aps : 0.0f;
    ch[26] = pitch_speed_ref;
    ch[27] = pitch_hold_ref;
    ch[28] = (motor_pitch != NULL) ? motor_pitch->measure.total_angle : 0.0f;
    ch[29] = (gimbal_imu != NULL) ? gimbal_imu->Pitch : 0.0f;
    ch[30] = (gimbal_imu != NULL) ? gimbal_imu->Gyro[0] : 0.0f;
    ch[31] = pitch_current_ff;

    memcpy(txbuf, ch, sizeof(ch));
    memcpy(txbuf + sizeof(ch), tail, sizeof(tail));
    TelemetrySendBuffer(txbuf, (uint16_t)sizeof(txbuf));
}

/* Main ----------------------------------------------------------------------*/
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

    Debug_DisableWatchdogs();
    DWT_Init(168);
    BSPLogInit();
    InitTelemetryUsart();

    et08_ctrl = ET08_Init(&RC_UART);
    gimbal_imu = INS_Init();
    GimbalMotorsInit();
    ChassisMotorsInit();

    HAL_Delay(MOTOR_STABILIZE_TIME_MS);
    TelemetrySendString("[chs_gmb_et08] control loop start\r\n");

    {
        uint32_t last_control_tick = DWT_GetTimeline_ms();
        uint32_t last_telemetry_tick = last_control_tick;
        uint32_t last_vofa_tick = last_control_tick;
        uint32_t last_ins_tick = last_control_tick;
        uint32_t last_daemon_tick = last_control_tick;
        uint32_t last_motor_tick = last_control_tick;

        while (1) {
            uint32_t now = DWT_GetTimeline_ms();

            if ((now - last_ins_tick) >= INS_TASK_PERIOD_MS) {
                last_ins_tick = now;
                INS_Task();
            }
            if ((now - last_daemon_tick) >= DAEMON_TASK_PERIOD_MS) {
                last_daemon_tick = now;
                DaemonTask();
            }
            if ((now - last_motor_tick) >= MOTOR_CONTROL_PERIOD_MS) {
                last_motor_tick = now;
                DJIMotorControl();
            }

            if ((now - last_control_tick) >= MAIN_LOOP_PERIOD_MS) {
                float dt_s = (float)(now - last_control_tick) / 1000.0f;
                ChassisCmd_t cmd = {0.0f};
                bool gimbal_online;
                bool chassis_online;

                if (dt_s <= 0.0f) {
                    dt_s = MAIN_LOOP_PERIOD_MS / 1000.0f;
                }
                last_control_tick = now;

                gimbal_online = GimbalUpdateFromET08(dt_s);
                chassis_online = BuildChassisCommandFromEt08(&cmd);

                if (!gimbal_online || !chassis_online) {
                    GimbalStop();
                    ChassisStop();
                    if (last_online_state != 0U) {
                        TelemetrySendString("[chs_gmb_et08] remote offline/failsafe -> stop\r\n");
                        last_online_state = 0U;
                    }
                } else {
                    if (chassis_mode != last_chassis_mode) {
                        TelemetryPrintf("[chs_gmb_et08] chassis mode -> %s offset_x10=%ld\r\n",
                                        (chassis_mode == CHASSIS_MODE_FOLLOW) ? "FOLLOW" : "SEPARATE",
                                        (long)(cmd.yaw_offset_deg * 10.0f));
                        last_chassis_mode = chassis_mode;
                    }
                    ChassisApplyCommand(&cmd);
                    if (last_online_state == 0U) {
                        TelemetrySendString("[chs_gmb_et08] remote online -> run\r\n");
                        last_online_state = 1U;
                    }
                }
            }

            if ((now - last_telemetry_tick) >= TELEMETRY_PERIOD_MS) {
                last_telemetry_tick = now;
                SendTelemetry();
            }
            if ((now - last_vofa_tick) >= VOFA_PERIOD_MS) {
                last_vofa_tick = now;
                SendVofaFrame();
            }

            HAL_Delay(1);
        }
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

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
