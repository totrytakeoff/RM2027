/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Infantry ET08 minimal gimbal demo
 ******************************************************************************
 * @attention
 *
 * 仅保留云台测试链路：
 * ET08(SBUS) -> yaw/pitch指令 -> GM6020 速度/位置双模式控制
 *
 * - SBUS 输入: USART3
 * - 调试输出: USART6
 * - 云台电机: Yaw(CAN1 ID1), Pitch(CAN2 ID1)
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
#ifndef GIMBAL_SPIN_DEMO_ENABLE
#define GIMBAL_SPIN_DEMO_ENABLE 0
#endif

#define TELEMETRY_RX_DUMMY 32U
#define TELEMETRY_PERIOD_MS 100U
#define VOFA_PERIOD_MS 20U
#define INS_TASK_PERIOD_MS 1U
#define DAEMON_TASK_PERIOD_MS 1U
#define MOTOR_CONTROL_PERIOD_MS 2U
#define RC_INPUT_DEADZONE 50
#define RC_INPUT_ENTER_DEADZONE 180
#define RC_INPUT_EXIT_DEADZONE 90
#define RC_INPUT_ENTER_COUNT 4
#define RC_INPUT_EXIT_COUNT 10
#define HOLD_REF_UPDATE_MIN_SPEED 300.0f
#define HOLD_REF_UPDATE_MIN_RAW 220
#define PITCH_BRAKE_SPEED_EPS 20.0f
#define PITCH_BRAKE_STABLE_COUNT 3U
#define PITCH_BRAKE_TIMEOUT_MS 120U
#define PITCH_RELEASE_SPEED_PREDICT_GAIN 0.02f
#define DEMO_GIMBAL_SPEED_DEADZONE 30.0f
#define DEMO_ET08_GIMBAL_YAW_SPEED_SCALE 0.23f
#define DEMO_ET08_PITCH_SPEED_SCALE 0.15f
#define GM6020_SPEED_MIN (-GM6020_SPEED_MAX)
#define ET08_SWITCH_POS_UP 0U
#define ET08_SWITCH_POS_DOWN 2U

#if GIMBAL_SPIN_DEMO_ENABLE
#define CHASSIS_MOTOR_COUNT 4U
#define CHASSIS_SPIN_WZ_CMD 18.0f
#define CHASSIS_SPIN_WZ_RAMP_STEP 0.50f
#define WHEEL_SPEED_LIMIT_DPS 30000.0f
#endif

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    GIMBAL_MODE_FOLLOW = 0,
    GIMBAL_MODE_SEPARATE = 1,
} GimbalMode_t;

typedef enum {
    AXIS_CTRL_ANGLE = 0,
    AXIS_CTRL_SPEED = 1,
    AXIS_CTRL_BRAKE = 2,
} AxisCtrlMode_t;

#if GIMBAL_SPIN_DEMO_ENABLE
typedef struct {
    float vx;
    float vy;
    float wz;
} ChassisCmd_t;
#endif

/* Private variables ---------------------------------------------------------*/
static ET08_Ctrl_t *et08_ctrl = NULL;
static USARTInstance *telemetry_usart = NULL;
static attitude_t *gimbal_imu = NULL;

static DJIMotorInstance *motor_yaw = NULL;
static DJIMotorInstance *motor_pitch = NULL;

static uint8_t gimbal_enabled = 0U;
static uint8_t last_online_state = 0U;

static GimbalMode_t gimbal_mode = GIMBAL_MODE_FOLLOW;
static float yaw_speed_ref = 0.0f;
static float pitch_speed_ref = 0.0f;
static float yaw_hold_ref = 0.0f;
static float pitch_hold_ref = 0.0f;
static float pitch_release_hold_ref = 0.0f;
static float pitch_current_ff = 0.0f;
static float pitch_imu_speed_fdb = 0.0f;
static float yaw_separate_center = 0.0f;
static AxisCtrlMode_t yaw_ctrl_mode = AXIS_CTRL_ANGLE;
static AxisCtrlMode_t pitch_ctrl_mode = AXIS_CTRL_ANGLE;
static GimbalMode_t last_gimbal_mode = GIMBAL_MODE_FOLLOW;

#if GIMBAL_SPIN_DEMO_ENABLE
static DJIMotorInstance *motor_fr = NULL;
static DJIMotorInstance *motor_fl = NULL;
static DJIMotorInstance *motor_br = NULL;
static DJIMotorInstance *motor_bl = NULL;
static uint8_t chassis_enabled = 0U;
static uint8_t chassis_spin_enabled = 0U;
static uint8_t last_chassis_spin_enabled = 0U;
static float chassis_wz_target = 0.0f;
static float chassis_wz_cmd = 0.0f;
static float wheel_speed_ref[CHASSIS_MOTOR_COUNT] = {0.0f};
#endif

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
static float GetYawHoldFeedbackAngle(void);
static float LimitYawSeparateHoldRef(float ref);
static void ApplyYawPidProfile(GimbalMode_t mode);
static void UpdatePitchGravityFeedforward(void);
static float GetPitchSpeedFeedback(void);
static void ResetPidState(PIDInstance *pid);
static void ResetPitchPidRuntime(void);
static float PidSatRatio(const PIDInstance *pid);
static uint8_t PidNearlySaturated(const PIDInstance *pid);
static const char *AxisCtrlModeName(AxisCtrlMode_t mode);
static uint8_t ET08_MapUpperSwitchPos(uint8_t state);

static void GimbalMotorsInit(void);
static void GimbalStop(void);
static bool GimbalUpdateFromET08(void);
static void SendTelemetry(void);
static void SendVofaFrame(void);
static uint8_t UpdateStickActiveState(int16_t raw, uint8_t prev_state, uint8_t *enter_cnt, uint8_t *exit_cnt);

#if GIMBAL_SPIN_DEMO_ENABLE
static void OmniInverseKinematics(float vx, float vy, float wz, float out[4]);
static void ChassisMotorsInit(void);
static void ChassisStop(void);
static void ChassisApplyCommand(const ChassisCmd_t *cmd);
static void ChassisUpdateSpinFromEt08(void);
#endif

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
    TelemetrySendString("[gmb_et08] USART6 telemetry ready\r\n");
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

    uint16_t len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    TelemetrySendBuffer((const uint8_t *)msg, len);
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

static float GetYawHoldFeedbackAngle(void)
{
    if (gimbal_mode == GIMBAL_MODE_SEPARATE && gimbal_imu != NULL) {
        return gimbal_imu->YawTotalAngle;
    }
    return (motor_yaw != NULL) ? motor_yaw->measure.total_angle : 0.0f;
}

static float LimitYawSeparateHoldRef(float ref)
{
    if (gimbal_mode == GIMBAL_MODE_SEPARATE && gimbal_imu != NULL) {
        return ClampFloat(ref,
                          yaw_separate_center - GIMBAL_SEPARATE_YAW_MAX_ANGLE,
                          yaw_separate_center + GIMBAL_SEPARATE_YAW_MAX_ANGLE);
    }
    return ref;
}

static void ApplyYawPidProfile(GimbalMode_t mode)
{
    PID_Init_Config_s angle_cfg = {
        .Kp = YAW_FOLLOW_ANGLE_KP,
        .Ki = YAW_FOLLOW_ANGLE_KI,
        .Kd = YAW_FOLLOW_ANGLE_KD,
        .IntegralLimit = 500.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = YAW_FOLLOW_ANGLE_MAX_OUT,
    };
    PID_Init_Config_s speed_cfg = {
        .Kp = YAW_FOLLOW_SPEED_KP,
        .Ki = YAW_FOLLOW_SPEED_KI,
        .Kd = YAW_FOLLOW_SPEED_KD,
        .IntegralLimit = 1000.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = YAW_FOLLOW_SPEED_MAX_OUT,
    };

    if (motor_yaw == NULL) {
        return;
    }

    if (mode == GIMBAL_MODE_SEPARATE) {
        angle_cfg.Kp = YAW_SEPARATE_ANGLE_KP;
        angle_cfg.Ki = YAW_SEPARATE_ANGLE_KI;
        angle_cfg.Kd = YAW_SEPARATE_ANGLE_KD;
        angle_cfg.MaxOut = YAW_SEPARATE_ANGLE_MAX_OUT;

        speed_cfg.Kp = YAW_SEPARATE_SPEED_KP;
        speed_cfg.Ki = YAW_SEPARATE_SPEED_KI;
        speed_cfg.Kd = YAW_SEPARATE_SPEED_KD;
        speed_cfg.MaxOut = YAW_SEPARATE_SPEED_MAX_OUT;
    }

    PIDInit(&motor_yaw->motor_controller.angle_PID, &angle_cfg);
    PIDInit(&motor_yaw->motor_controller.speed_PID, &speed_cfg);
}

static void UpdatePitchGravityFeedforward(void)
{
    if (motor_pitch == NULL) {
        pitch_current_ff = 0.0f;
        return;
    }

    /* Positive FF means positive motor output (CCW by project convention). */
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
        return pitch_imu_speed_fdb;
    }
    pitch_imu_speed_fdb = (motor_pitch != NULL) ? motor_pitch->measure.speed_aps : 0.0f;
    return pitch_imu_speed_fdb;
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

static float PidSatRatio(const PIDInstance *pid)
{
    if (pid == NULL || pid->MaxOut <= 1e-6f) {
        return 0.0f;
    }
    return fabsf(pid->Output) / pid->MaxOut;
}

static uint8_t PidNearlySaturated(const PIDInstance *pid)
{
    return (PidSatRatio(pid) >= 0.98f) ? 1U : 0U;
}

static const char *AxisCtrlModeName(AxisCtrlMode_t mode)
{
    switch (mode) {
    case AXIS_CTRL_SPEED:
        return "SPEED";
    case AXIS_CTRL_BRAKE:
        return "BRAKE";
    case AXIS_CTRL_ANGLE:
    default:
        return "ANGLE";
    }
}

static uint8_t ET08_MapUpperSwitchPos(uint8_t state)
{
    if (state == 0xFFU || state > 5U) {
        return 0xFFU;
    }
    return (state <= 2U) ? ET08_SWITCH_POS_UP : ET08_SWITCH_POS_DOWN;
}

static uint8_t UpdateStickActiveState(int16_t raw, uint8_t prev_state, uint8_t *enter_cnt, uint8_t *exit_cnt)
{
    int16_t abs_raw = (raw >= 0) ? raw : (int16_t)(-raw);
    if (enter_cnt == NULL || exit_cnt == NULL) {
        return prev_state;
    }
    if (prev_state != 0U) {
        if (abs_raw <= RC_INPUT_EXIT_DEADZONE) {
            if (*exit_cnt < 255U) {
                (*exit_cnt)++;
            }
        } else {
            *exit_cnt = 0U;
        }
        *enter_cnt = 0U;
        if (*exit_cnt >= RC_INPUT_EXIT_COUNT) {
            *exit_cnt = 0U;
            return 0U;
        }
        return 1U;
    }
    if (abs_raw >= RC_INPUT_ENTER_DEADZONE) {
        if (*enter_cnt < 255U) {
            (*enter_cnt)++;
        }
    } else {
        *enter_cnt = 0U;
    }
    *exit_cnt = 0U;
    if (*enter_cnt >= RC_INPUT_ENTER_COUNT) {
        *enter_cnt = 0U;
        return 1U;
    }
    return 0U;
}

#if GIMBAL_SPIN_DEMO_ENABLE
static void OmniInverseKinematics(float vx, float vy, float wz, float out[4])
{
    const float l = CHASSIS_WHEEL_BASE * 0.5f;
    const float v_fr = vy - vx - l * wz;
    const float v_fl = vy + vx - l * wz;
    const float v_br = -vy + vx - l * wz;
    const float v_bl = -vy - vx - l * wz;

    out[0] = v_fr / CHASSIS_WHEEL_RADIUS;
    out[1] = v_fl / CHASSIS_WHEEL_RADIUS;
    out[2] = v_br / CHASSIS_WHEEL_RADIUS;
    out[3] = v_bl / CHASSIS_WHEEL_RADIUS;
}

static void ChassisMotorsInit(void)
{
    Motor_Init_Config_s config = {
        .motor_type = M3508,
        .can_init_config = {
            .can_handle = &CHASSIS_CAN,
            .tx_id = CHASSIS_MOTOR_FR_ID,
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

    config.can_init_config.tx_id = CHASSIS_MOTOR_FL_ID;
    motor_fl = DJIMotorInit(&config);
    if (motor_fl != NULL) {
        DJIMotorOuterLoop(motor_fl, SPEED_LOOP);
        DJIMotorStop(motor_fl);
    }

    config.can_init_config.tx_id = CHASSIS_MOTOR_BR_ID;
    motor_br = DJIMotorInit(&config);
    if (motor_br != NULL) {
        DJIMotorOuterLoop(motor_br, SPEED_LOOP);
        DJIMotorStop(motor_br);
    }

    config.can_init_config.tx_id = CHASSIS_MOTOR_BL_ID;
    motor_bl = DJIMotorInit(&config);
    if (motor_bl != NULL) {
        DJIMotorOuterLoop(motor_bl, SPEED_LOOP);
        DJIMotorStop(motor_bl);
    }

    TelemetrySendString("[gmb_et08] chassis motors initialized\r\n");
}

static void ChassisStop(void)
{
    chassis_wz_target = 0.0f;
    chassis_wz_cmd = 0.0f;
    chassis_enabled = 0U;
    chassis_spin_enabled = 0U;
    memset(wheel_speed_ref, 0, sizeof(wheel_speed_ref));

    if (motor_fr != NULL)
        DJIMotorStop(motor_fr);
    if (motor_fl != NULL)
        DJIMotorStop(motor_fl);
    if (motor_br != NULL)
        DJIMotorStop(motor_br);
    if (motor_bl != NULL)
        DJIMotorStop(motor_bl);
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

    OmniInverseKinematics(cmd->vx, cmd->vy, cmd->wz, wheel_speed_rad_s);

    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        float speed_dps = wheel_speed_rad_s[i] * 180.0f / (float)M_PI;
        speed_dps *= CHASSIS_SPEED_SCALE;
        if (fabsf(speed_dps) < CHASSIS_SPEED_DEADZONE) {
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

static void ChassisUpdateSpinFromEt08(void)
{
    ChassisCmd_t cmd = {0.0f};
    uint8_t sa_pos;

    if (et08_ctrl == NULL) {
        ChassisStop();
        return;
    }

    sa_pos = ET08_MapUpperSwitchPos(et08_ctrl->switch_sa_sb_state);
    chassis_spin_enabled = (sa_pos == ET08_SWITCH_POS_UP) ? 1U : 0U;
    chassis_wz_target = chassis_spin_enabled ? CHASSIS_SPIN_WZ_CMD : 0.0f;

    if (chassis_spin_enabled != last_chassis_spin_enabled) {
        TelemetryPrintf("[gmb_et08] chassis spin -> %s sa_raw=%u sa_state=%u target_wz_x100=%ld\r\n",
                        chassis_spin_enabled ? "ON" : "OFF",
                        (unsigned int)et08_ctrl->switch_sa_sb_raw,
                        (unsigned int)et08_ctrl->switch_sa_sb_state,
                        (long)(chassis_wz_target * 100.0f));
        last_chassis_spin_enabled = chassis_spin_enabled;
    }

    if (chassis_wz_cmd < chassis_wz_target) {
        chassis_wz_cmd += CHASSIS_SPIN_WZ_RAMP_STEP;
        if (chassis_wz_cmd > chassis_wz_target) {
            chassis_wz_cmd = chassis_wz_target;
        }
    } else if (chassis_wz_cmd > chassis_wz_target) {
        chassis_wz_cmd -= CHASSIS_SPIN_WZ_RAMP_STEP;
        if (chassis_wz_cmd < chassis_wz_target) {
            chassis_wz_cmd = chassis_wz_target;
        }
    }

    if (!chassis_spin_enabled && fabsf(chassis_wz_cmd) <= 1e-3f) {
        ChassisStop();
        return;
    }

    cmd.wz = chassis_wz_cmd;
    ChassisApplyCommand(&cmd);
}
#endif

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
                .Kp = YAW_FOLLOW_ANGLE_KP,
                .Ki = YAW_FOLLOW_ANGLE_KI,
                .Kd = YAW_FOLLOW_ANGLE_KD,
                .IntegralLimit = 500.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = YAW_FOLLOW_ANGLE_MAX_OUT,
            },
            .speed_PID = {
                .Kp = YAW_FOLLOW_SPEED_KP,
                .Ki = YAW_FOLLOW_SPEED_KI,
                .Kd = YAW_FOLLOW_SPEED_KD,
                .IntegralLimit = 1000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = YAW_FOLLOW_SPEED_MAX_OUT,
            },
            .other_angle_feedback_ptr = (gimbal_imu != NULL) ? &gimbal_imu->YawTotalAngle : NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
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

    if (motor_yaw) {
        DJIMotorOuterLoop(motor_yaw, SPEED_LOOP);
        DJIMotorStop(motor_yaw);
        ApplyYawPidProfile(GIMBAL_MODE_FOLLOW);
        yaw_hold_ref = GetYawHoldFeedbackAngle();
        yaw_separate_center = yaw_hold_ref;
    }
    if (motor_pitch) {
        DJIMotorOuterLoop(motor_pitch, SPEED_LOOP);
        DJIMotorStop(motor_pitch);
        pitch_hold_ref = motor_pitch->measure.total_angle;
        pitch_release_hold_ref = pitch_hold_ref;
    }

    TelemetrySendString("[gmb_et08] gimbal motors initialized\r\n");
}

static void GimbalStop(void)
{
    yaw_speed_ref = 0.0f;
    pitch_speed_ref = 0.0f;
    yaw_ctrl_mode = AXIS_CTRL_ANGLE;
    pitch_ctrl_mode = AXIS_CTRL_ANGLE;
    gimbal_enabled = 0U;
    pitch_release_hold_ref = pitch_hold_ref;
    pitch_current_ff = 0.0f;
    pitch_imu_speed_fdb = 0.0f;
    ResetPitchPidRuntime();

    if (motor_yaw) {
        DJIMotorStop(motor_yaw);
    }
    if (motor_pitch) {
        DJIMotorStop(motor_pitch);
    }
}

static bool GimbalUpdateFromET08(void)
{
    static AxisCtrlMode_t last_yaw_ctrl_mode = AXIS_CTRL_ANGLE;
    static AxisCtrlMode_t last_pitch_ctrl_mode = AXIS_CTRL_ANGLE;
    static uint8_t yaw_cmd_active = 0U;
    static uint8_t pitch_cmd_active = 0U;
    static uint8_t yaw_enter_cnt = 0U;
    static uint8_t yaw_exit_cnt = 0U;
    static uint8_t pitch_enter_cnt = 0U;
    static uint8_t pitch_exit_cnt = 0U;
    static uint8_t yaw_brake_stable_count = 0U;
    static uint32_t yaw_brake_start_ms = 0U;
    static uint8_t pitch_brake_stable_count = 0U;
    static uint32_t pitch_brake_start_ms = 0U;

    if (et08_ctrl == NULL || motor_yaw == NULL || motor_pitch == NULL) {
        return false;
    }

    if (!ET08_IsOnline() || et08_ctrl->failsafe || et08_ctrl->frame_lost) {
        return false;
    }

    if (!gimbal_enabled) {
        DJIMotorEnable(motor_yaw);
        DJIMotorEnable(motor_pitch);
        gimbal_enabled = 1U;
        TelemetryPrintf("[gmb_et08] motors re-enabled hold(y_x10=%ld p_x10=%ld)\r\n",
                        (long)(yaw_hold_ref * 10.0f),
                        (long)(pitch_hold_ref * 10.0f));
    }

    UpdatePitchGravityFeedforward();
    GetPitchSpeedFeedback();

    const int16_t yaw_raw = et08_ctrl->right.x;
    const int16_t pitch_raw = et08_ctrl->right.y;
    const int16_t yaw_in = ApplyDeadzone(yaw_raw, RC_INPUT_DEADZONE);
    const int16_t pitch_in = ApplyDeadzone(pitch_raw, RC_INPUT_DEADZONE);
    const int16_t yaw_abs = (yaw_in >= 0) ? yaw_in : (int16_t)(-yaw_in);
    const int16_t pitch_abs = (pitch_in >= 0) ? pitch_in : (int16_t)(-pitch_in);
    const uint32_t now_ms = DWT_GetTimeline_ms();

    uint8_t sd_pos = ET08_MapUpperSwitchPos(et08_ctrl->switch_sd_sc_state);
    if (sd_pos == ET08_SWITCH_POS_UP) {
        gimbal_mode = GIMBAL_MODE_FOLLOW;
    } else {
        gimbal_mode = GIMBAL_MODE_SEPARATE;
    }
    if (gimbal_mode != last_gimbal_mode) {
        ApplyYawPidProfile(gimbal_mode);
        if (gimbal_mode == GIMBAL_MODE_SEPARATE && gimbal_imu != NULL) {
            yaw_separate_center = gimbal_imu->YawTotalAngle;
        }
        yaw_hold_ref = LimitYawSeparateHoldRef(GetYawHoldFeedbackAngle());
        TelemetryPrintf("[gmb_et08] mode switch -> %s\r\n",
                        (gimbal_mode == GIMBAL_MODE_FOLLOW) ? "FOLLOW" : "SEPARATE");
        TelemetryPrintf("[gmb_et08] yaw pid profile -> %s angle(kp_x100=%ld ki_x100=%ld kd_x100=%ld) speed(kp_x100=%ld ki_x100=%ld kd_x100=%ld)\r\n",
                        (gimbal_mode == GIMBAL_MODE_FOLLOW) ? "FOLLOW" : "SEPARATE",
                        (long)(((gimbal_mode == GIMBAL_MODE_FOLLOW) ? YAW_FOLLOW_ANGLE_KP : YAW_SEPARATE_ANGLE_KP) * 100.0f),
                        (long)(((gimbal_mode == GIMBAL_MODE_FOLLOW) ? YAW_FOLLOW_ANGLE_KI : YAW_SEPARATE_ANGLE_KI) * 100.0f),
                        (long)(((gimbal_mode == GIMBAL_MODE_FOLLOW) ? YAW_FOLLOW_ANGLE_KD : YAW_SEPARATE_ANGLE_KD) * 100.0f),
                        (long)(((gimbal_mode == GIMBAL_MODE_FOLLOW) ? YAW_FOLLOW_SPEED_KP : YAW_SEPARATE_SPEED_KP) * 100.0f),
                        (long)(((gimbal_mode == GIMBAL_MODE_FOLLOW) ? YAW_FOLLOW_SPEED_KI : YAW_SEPARATE_SPEED_KI) * 100.0f),
                        (long)(((gimbal_mode == GIMBAL_MODE_FOLLOW) ? YAW_FOLLOW_SPEED_KD : YAW_SEPARATE_SPEED_KD) * 100.0f));
        TelemetryPrintf("[gmb_et08] yaw mode lock hold_x10=%ld enc_x10=%ld imu_yaw_total_x10=%ld\r\n",
                        (long)(yaw_hold_ref * 10.0f),
                        (long)((motor_yaw != NULL) ? (motor_yaw->measure.total_angle * 10.0f) : 0.0f),
                        (long)((gimbal_imu != NULL) ? (gimbal_imu->YawTotalAngle * 10.0f) : 0.0f));
        if (gimbal_mode == GIMBAL_MODE_SEPARATE && gimbal_imu != NULL) {
            TelemetryPrintf("[gmb_et08] separate center hold_x10=%ld limit_x10=%ld\r\n",
                            (long)(yaw_separate_center * 10.0f),
                            (long)(GIMBAL_SEPARATE_YAW_MAX_ANGLE * 10.0f));
        }
        last_gimbal_mode = gimbal_mode;
    }

    {
        float yaw_ratio = ClampFloat((float)yaw_in / RC_STICK_SCALE, -1.0f, 1.0f);
        yaw_speed_ref = yaw_ratio * GM6020_SPEED_MAX * DEMO_ET08_GIMBAL_YAW_SPEED_SCALE;
        yaw_speed_ref = ClampFloat(yaw_speed_ref, GM6020_SPEED_MIN, GM6020_SPEED_MAX);
    }
    if (fabsf(yaw_speed_ref) < DEMO_GIMBAL_SPEED_DEADZONE) {
        yaw_speed_ref = 0.0f;
    }

    {
        float pitch_ratio = ClampFloat((float)pitch_in / RC_STICK_SCALE, -1.0f, 1.0f);
        pitch_speed_ref = pitch_ratio * GM6020_SPEED_MAX * DEMO_ET08_PITCH_SPEED_SCALE;
        if (fabsf(pitch_speed_ref) < DEMO_GIMBAL_SPEED_DEADZONE) {
            pitch_speed_ref = 0.0f;
        }
    }
    float pitch_speed_cmd = ClampFloat(pitch_speed_ref, GM6020_SPEED_MIN, GM6020_SPEED_MAX);
    yaw_cmd_active = UpdateStickActiveState(yaw_in, yaw_cmd_active, &yaw_enter_cnt, &yaw_exit_cnt);
    pitch_cmd_active = UpdateStickActiveState(pitch_in, pitch_cmd_active, &pitch_enter_cnt, &pitch_exit_cnt);
    if (yaw_cmd_active) {
        yaw_ctrl_mode = AXIS_CTRL_SPEED;
        yaw_brake_stable_count = 0U;
        yaw_brake_start_ms = 0U;
    } else if (yaw_ctrl_mode == AXIS_CTRL_SPEED) {
        yaw_hold_ref = LimitYawSeparateHoldRef(GetYawHoldFeedbackAngle());
        yaw_ctrl_mode = AXIS_CTRL_BRAKE;
        yaw_brake_stable_count = 0U;
        yaw_brake_start_ms = now_ms;
        TelemetryPrintf("[gmb_et08] yaw speed -> brake lock_x10=%ld spd_x10=%ld\r\n",
                        (long)(yaw_hold_ref * 10.0f),
                        (long)(motor_yaw->measure.speed_aps * 10.0f));
    } else if (yaw_ctrl_mode == AXIS_CTRL_BRAKE) {
        yaw_hold_ref = LimitYawSeparateHoldRef(GetYawHoldFeedbackAngle());
        if (fabsf(motor_yaw->measure.speed_aps) <= YAW_BRAKE_SPEED_EPS) {
            if (yaw_brake_stable_count < 255U) {
                yaw_brake_stable_count++;
            }
        } else {
            yaw_brake_stable_count = 0U;
        }
        if (yaw_brake_stable_count >= YAW_BRAKE_STABLE_COUNT ||
            (now_ms - yaw_brake_start_ms) >= YAW_BRAKE_TIMEOUT_MS) {
            yaw_hold_ref = LimitYawSeparateHoldRef(GetYawHoldFeedbackAngle());
            yaw_ctrl_mode = AXIS_CTRL_ANGLE;
            TelemetryPrintf("[gmb_et08] yaw brake -> hold lock_x10=%ld spd_x10=%ld\r\n",
                            (long)(yaw_hold_ref * 10.0f),
                            (long)(motor_yaw->measure.speed_aps * 10.0f));
        }
    }

    if (yaw_ctrl_mode == AXIS_CTRL_SPEED) {
        DJIMotorOuterLoop(motor_yaw, SPEED_LOOP);
        DJIMotorSetRef(motor_yaw, yaw_speed_ref);
        yaw_hold_ref = LimitYawSeparateHoldRef(GetYawHoldFeedbackAngle());
    } else if (yaw_ctrl_mode == AXIS_CTRL_BRAKE) {
        DJIMotorOuterLoop(motor_yaw, SPEED_LOOP);
        DJIMotorSetRef(motor_yaw, 0.0f);
    } else {
        yaw_hold_ref = LimitYawSeparateHoldRef(yaw_hold_ref);
        if (last_yaw_ctrl_mode != AXIS_CTRL_ANGLE) {
            TelemetryPrintf("[gmb_et08] yaw -> hold lock_x10=%ld now_x10=%ld\r\n",
                            (long)(yaw_hold_ref * 10.0f),
                            (long)(motor_yaw->measure.total_angle * 10.0f));
        }
        DJIMotorChangeFeed(motor_yaw,
                           ANGLE_LOOP,
                           (gimbal_mode == GIMBAL_MODE_SEPARATE && gimbal_imu != NULL) ? OTHER_FEED : MOTOR_FEED);
        DJIMotorOuterLoop(motor_yaw, ANGLE_LOOP);
        DJIMotorSetRef(motor_yaw, yaw_hold_ref);
    }

    if (pitch_cmd_active) {
        if (pitch_ctrl_mode != AXIS_CTRL_SPEED) {
            ResetPitchPidRuntime();
        }
        pitch_ctrl_mode = AXIS_CTRL_SPEED;
        pitch_brake_stable_count = 0U;
        pitch_brake_start_ms = 0U;
    } else if (pitch_ctrl_mode == AXIS_CTRL_SPEED) {
        pitch_release_hold_ref = motor_pitch->measure.total_angle +
                                 GetPitchSpeedFeedback() * PITCH_RELEASE_SPEED_PREDICT_GAIN;
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
            TelemetryPrintf("[gmb_et08] pitch brake -> hold lock_x10=%ld spd_x10=%ld\r\n",
                            (long)(pitch_hold_ref * 10.0f),
                            (long)(GetPitchSpeedFeedback() * 10.0f));
        }
    }

    if (pitch_ctrl_mode == AXIS_CTRL_SPEED) {
        DJIMotorOuterLoop(motor_pitch, SPEED_LOOP);
        DJIMotorSetRef(motor_pitch, pitch_speed_cmd);
        if (fabsf(pitch_speed_cmd) >= HOLD_REF_UPDATE_MIN_SPEED && pitch_abs >= HOLD_REF_UPDATE_MIN_RAW) {
            pitch_hold_ref = motor_pitch->measure.total_angle;
        }
    } else if (pitch_ctrl_mode == AXIS_CTRL_BRAKE) {
        DJIMotorOuterLoop(motor_pitch, SPEED_LOOP);
        DJIMotorChangeFeed(motor_pitch, SPEED_LOOP, (gimbal_imu != NULL) ? OTHER_FEED : MOTOR_FEED);
        DJIMotorSetRef(motor_pitch, 0.0f);
    } else {
        if (last_pitch_ctrl_mode != AXIS_CTRL_ANGLE) {
            TelemetryPrintf("[gmb_et08] pitch -> hold lock_x10=%ld now_x10=%ld\r\n",
                            (long)(pitch_hold_ref * 10.0f),
                            (long)(motor_pitch->measure.total_angle * 10.0f));
        }
        DJIMotorChangeFeed(motor_pitch, SPEED_LOOP, (gimbal_imu != NULL) ? OTHER_FEED : MOTOR_FEED);
        DJIMotorOuterLoop(motor_pitch, ANGLE_LOOP);
        DJIMotorSetRef(motor_pitch, pitch_hold_ref);
    }

    if (yaw_ctrl_mode != last_yaw_ctrl_mode) {
        TelemetryPrintf("[gmb_et08] yaw loop -> %s\r\n", AxisCtrlModeName(yaw_ctrl_mode));
    }
    if (pitch_ctrl_mode != last_pitch_ctrl_mode) {
        TelemetryPrintf("[gmb_et08] pitch loop -> %s\r\n", AxisCtrlModeName(pitch_ctrl_mode));
    }
    last_yaw_ctrl_mode = yaw_ctrl_mode;
    last_pitch_ctrl_mode = pitch_ctrl_mode;

    return true;
}

static void SendTelemetry(void)
{
    static uint32_t seq = 0U;
    seq++;

    const int16_t rx = (et08_ctrl != NULL) ? et08_ctrl->right.x : 0;
    const int16_t ry = (et08_ctrl != NULL) ? et08_ctrl->right.y : 0;
    const long imu_yaw_total_x10 = (long)((gimbal_imu != NULL) ? (gimbal_imu->YawTotalAngle * 10.0f) : 0.0f);
    const long imu_yaw_x10 = (long)((gimbal_imu != NULL) ? (gimbal_imu->Yaw * 10.0f) : 0.0f);
    const long imu_pitch_x10 = (long)((gimbal_imu != NULL) ? (gimbal_imu->Pitch * 10.0f) : 0.0f);
    const long imu_gz_x10 = (long)((gimbal_imu != NULL) ? (gimbal_imu->Gyro[2] * 10.0f) : 0.0f);
    const long imu_gx_x10 = (long)((gimbal_imu != NULL) ? (gimbal_imu->Gyro[0] * 10.0f) : 0.0f);

    TelemetryPrintf("[gmb_et08] #%lu online=%u fs=%u lost=%u mode=%s raw(rx=%d ry=%d)\r\n",
                    (unsigned long)seq,
                    (unsigned int)ET08_IsOnline(),
                    (unsigned int)((et08_ctrl != NULL) ? et08_ctrl->failsafe : 0U),
                    (unsigned int)((et08_ctrl != NULL) ? et08_ctrl->frame_lost : 0U),
                    (gimbal_mode == GIMBAL_MODE_FOLLOW) ? "FOLLOW" : "SEPARATE",
                    rx,
                    ry);

    TelemetryPrintf("[gmb_et08] loop(y=%s p=%s) ref_spd(y_x10=%ld p_x10=%ld) fdb_spd(y_x10=%ld p_x10=%ld) hold_ang(y_x10=%ld p_x10=%ld)\r\n",
                    AxisCtrlModeName(yaw_ctrl_mode),
                    AxisCtrlModeName(pitch_ctrl_mode),
                    (long)(yaw_speed_ref * 10.0f),
                    (long)(pitch_speed_ref * 10.0f),
                    (long)((motor_yaw != NULL) ? (motor_yaw->measure.speed_aps * 10.0f) : 0.0f),
                    (long)((motor_pitch != NULL) ? (motor_pitch->measure.speed_aps * 10.0f) : 0.0f),
                    (long)(yaw_hold_ref * 10.0f),
                    (long)(pitch_hold_ref * 10.0f));

    TelemetryPrintf("[gmb_et08] yaw_state hold_x10=%ld enc_x10=%ld imu_yaw_total_x10=%ld imu_yaw_x10=%ld imu_gz_x10=%ld\r\n",
                    (long)(yaw_hold_ref * 10.0f),
                    (long)((motor_yaw != NULL) ? (motor_yaw->measure.total_angle * 10.0f) : 0.0f),
                    imu_yaw_total_x10,
                    imu_yaw_x10,
                    imu_gz_x10);

    TelemetryPrintf("[gmb_et08] imu yaw_total_x10=%ld yaw_x10=%ld pitch_x10=%ld gz_x10=%ld gx_x10=%ld\r\n",
                    imu_yaw_total_x10,
                    imu_yaw_x10,
                    imu_pitch_x10,
                    imu_gz_x10,
                    imu_gx_x10);

    TelemetryPrintf("[gmb_et08] yaw_pid a(ref_x10=%ld fdb_x10=%ld err_x10=%ld out_x10=%ld sat=%u) s(ref_x10=%ld fdb_x10=%ld err_x10=%ld out_x10=%ld sat=%u)\r\n",
                    (long)((motor_yaw != NULL) ? (motor_yaw->motor_controller.angle_PID.Ref * 10.0f) : 0.0f),
                    (long)((motor_yaw != NULL) ? (motor_yaw->motor_controller.angle_PID.Measure * 10.0f) : 0.0f),
                    (long)((motor_yaw != NULL) ? (motor_yaw->motor_controller.angle_PID.Err * 10.0f) : 0.0f),
                    (long)((motor_yaw != NULL) ? (motor_yaw->motor_controller.angle_PID.Output * 10.0f) : 0.0f),
                    (unsigned int)((motor_yaw != NULL) ? PidNearlySaturated(&motor_yaw->motor_controller.angle_PID) : 0U),
                    (long)((motor_yaw != NULL) ? (motor_yaw->motor_controller.speed_PID.Ref * 10.0f) : 0.0f),
                    (long)((motor_yaw != NULL) ? (motor_yaw->motor_controller.speed_PID.Measure * 10.0f) : 0.0f),
                    (long)((motor_yaw != NULL) ? (motor_yaw->motor_controller.speed_PID.Err * 10.0f) : 0.0f),
                    (long)((motor_yaw != NULL) ? (motor_yaw->motor_controller.speed_PID.Output * 10.0f) : 0.0f),
                    (unsigned int)((motor_yaw != NULL) ? PidNearlySaturated(&motor_yaw->motor_controller.speed_PID) : 0U));

    TelemetryPrintf("[gmb_et08] pitch_pid a(ref_x10=%ld fdb_x10=%ld err_x10=%ld out_x10=%ld sat=%u) s(ref_x10=%ld fdb_x10=%ld err_x10=%ld out_x10=%ld sat=%u)\r\n",
                    (long)((motor_pitch != NULL) ? (motor_pitch->motor_controller.angle_PID.Ref * 10.0f) : 0.0f),
                    (long)((motor_pitch != NULL) ? (motor_pitch->motor_controller.angle_PID.Measure * 10.0f) : 0.0f),
                    (long)((motor_pitch != NULL) ? (motor_pitch->motor_controller.angle_PID.Err * 10.0f) : 0.0f),
                    (long)((motor_pitch != NULL) ? (motor_pitch->motor_controller.angle_PID.Output * 10.0f) : 0.0f),
                    (unsigned int)((motor_pitch != NULL) ? PidNearlySaturated(&motor_pitch->motor_controller.angle_PID) : 0U),
                    (long)((motor_pitch != NULL) ? (motor_pitch->motor_controller.speed_PID.Ref * 10.0f) : 0.0f),
                    (long)((motor_pitch != NULL) ? (motor_pitch->motor_controller.speed_PID.Measure * 10.0f) : 0.0f),
                    (long)((motor_pitch != NULL) ? (motor_pitch->motor_controller.speed_PID.Err * 10.0f) : 0.0f),
                    (long)((motor_pitch != NULL) ? (motor_pitch->motor_controller.speed_PID.Output * 10.0f) : 0.0f),
                    (unsigned int)((motor_pitch != NULL) ? PidNearlySaturated(&motor_pitch->motor_controller.speed_PID) : 0U));

#if GIMBAL_SPIN_DEMO_ENABLE
    TelemetryPrintf("[gmb_et08] chassis spin=%u target_wz_x100=%ld cmd_wz_x100=%ld\r\n",
                    (unsigned int)chassis_spin_enabled,
                    (long)(chassis_wz_target * 100.0f),
                    (long)(chassis_wz_cmd * 100.0f));
    TelemetryPrintf("[gmb_et08] chassis wheel_ref(fr_x10=%ld fl_x10=%ld br_x10=%ld bl_x10=%ld) wheel_fdb(fr_x10=%ld fl_x10=%ld br_x10=%ld bl_x10=%ld)\r\n",
                    (long)(wheel_speed_ref[0] * 10.0f),
                    (long)(wheel_speed_ref[1] * 10.0f),
                    (long)(wheel_speed_ref[2] * 10.0f),
                    (long)(wheel_speed_ref[3] * 10.0f),
                    (long)((motor_fr != NULL) ? (motor_fr->measure.speed_aps * 10.0f) : 0.0f),
                    (long)((motor_fl != NULL) ? (motor_fl->measure.speed_aps * 10.0f) : 0.0f),
                    (long)((motor_br != NULL) ? (motor_br->measure.speed_aps * 10.0f) : 0.0f),
                    (long)((motor_bl != NULL) ? (motor_bl->measure.speed_aps * 10.0f) : 0.0f));
#endif
}

static void SendVofaFrame(void)
{
#if GIMBAL_SPIN_DEMO_ENABLE
    float ch[50];
#else
    float ch[39];
#endif
    uint8_t txbuf[sizeof(ch) + 4U];
    static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

    ch[0] = (float)HAL_GetTick();
    ch[1] = (float)ET08_IsOnline();
    ch[2] = (float)gimbal_mode;
    ch[3] = (float)yaw_ctrl_mode;
    ch[4] = (float)pitch_ctrl_mode;
    ch[5] = yaw_speed_ref;
    ch[6] = (motor_yaw != NULL) ? motor_yaw->measure.speed_aps : 0.0f;
    ch[7] = yaw_hold_ref;
    ch[8] = pitch_speed_ref;
    ch[9] = pitch_imu_speed_fdb;
    ch[10] = pitch_hold_ref;
    ch[11] = (motor_pitch != NULL) ? motor_pitch->measure.total_angle : 0.0f;
    ch[12] = (motor_pitch != NULL) ? motor_pitch->motor_controller.angle_PID.Ref : 0.0f;
    ch[13] = (motor_pitch != NULL) ? motor_pitch->motor_controller.angle_PID.Measure : 0.0f;
    ch[14] = (motor_pitch != NULL) ? motor_pitch->motor_controller.angle_PID.Err : 0.0f;
    ch[15] = (motor_pitch != NULL) ? motor_pitch->motor_controller.angle_PID.Output : 0.0f;
    ch[16] = (motor_pitch != NULL) ? motor_pitch->motor_controller.angle_PID.MaxOut : 0.0f;
    ch[17] = (motor_pitch != NULL) ? motor_pitch->motor_controller.speed_PID.Ref : 0.0f;
    ch[18] = (motor_pitch != NULL) ? motor_pitch->motor_controller.speed_PID.Measure : 0.0f;
    ch[19] = (motor_pitch != NULL) ? motor_pitch->motor_controller.speed_PID.Err : 0.0f;
    ch[20] = (motor_pitch != NULL) ? motor_pitch->motor_controller.speed_PID.Output : 0.0f;
    ch[21] = (motor_pitch != NULL) ? PidSatRatio(&motor_pitch->motor_controller.angle_PID) : 0.0f;
    ch[22] = (motor_yaw != NULL) ? motor_yaw->measure.total_angle : 0.0f;
    ch[23] = (motor_yaw != NULL) ? motor_yaw->motor_controller.angle_PID.Ref : 0.0f;
    ch[24] = (motor_yaw != NULL) ? motor_yaw->motor_controller.angle_PID.Measure : 0.0f;
    ch[25] = (motor_yaw != NULL) ? motor_yaw->motor_controller.angle_PID.Err : 0.0f;
    ch[26] = (motor_yaw != NULL) ? motor_yaw->motor_controller.angle_PID.Output : 0.0f;
    ch[27] = (motor_yaw != NULL) ? motor_yaw->motor_controller.angle_PID.MaxOut : 0.0f;
    ch[28] = (motor_yaw != NULL) ? motor_yaw->motor_controller.speed_PID.Ref : 0.0f;
    ch[29] = (motor_yaw != NULL) ? motor_yaw->motor_controller.speed_PID.Measure : 0.0f;
    ch[30] = (motor_yaw != NULL) ? motor_yaw->motor_controller.speed_PID.Err : 0.0f;
    ch[31] = (motor_yaw != NULL) ? motor_yaw->motor_controller.speed_PID.Output : 0.0f;
    ch[32] = (motor_yaw != NULL) ? PidSatRatio(&motor_yaw->motor_controller.angle_PID) : 0.0f;
    ch[33] = (gimbal_imu != NULL) ? gimbal_imu->YawTotalAngle : 0.0f;
    ch[34] = (gimbal_imu != NULL) ? gimbal_imu->Yaw : 0.0f;
    ch[35] = (gimbal_imu != NULL) ? gimbal_imu->Pitch : 0.0f;
    ch[36] = (gimbal_imu != NULL) ? gimbal_imu->Gyro[2] : 0.0f;
    ch[37] = (gimbal_imu != NULL) ? gimbal_imu->Gyro[0] : 0.0f;
    ch[38] = pitch_current_ff;
#if GIMBAL_SPIN_DEMO_ENABLE
    ch[39] = (float)chassis_spin_enabled;
    ch[40] = chassis_wz_target;
    ch[41] = chassis_wz_cmd;
    ch[42] = wheel_speed_ref[0];
    ch[43] = (motor_fr != NULL) ? motor_fr->measure.speed_aps : 0.0f;
    ch[44] = wheel_speed_ref[1];
    ch[45] = (motor_fl != NULL) ? motor_fl->measure.speed_aps : 0.0f;
    ch[46] = wheel_speed_ref[2];
    ch[47] = (motor_br != NULL) ? motor_br->measure.speed_aps : 0.0f;
    ch[48] = wheel_speed_ref[3];
    ch[49] = (motor_bl != NULL) ? motor_bl->measure.speed_aps : 0.0f;
#endif

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
#if GIMBAL_SPIN_DEMO_ENABLE
    ChassisMotorsInit();
#endif

    HAL_Delay(MOTOR_STABILIZE_TIME_MS);
    TelemetrySendString("[gmb_et08] control loop start\r\n");

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
            last_control_tick = now;

            bool online = GimbalUpdateFromET08();
            if (!online) {
                GimbalStop();
#if GIMBAL_SPIN_DEMO_ENABLE
                ChassisStop();
#endif
                if (last_online_state != 0U) {
                    TelemetryPrintf("[gmb_et08] remote offline/failsafe -> stop online=%u fs=%u lost=%u raw(rx=%d ry=%d)\r\n",
                                    (unsigned int)ET08_IsOnline(),
                                    (unsigned int)((et08_ctrl != NULL) ? et08_ctrl->failsafe : 0U),
                                    (unsigned int)((et08_ctrl != NULL) ? et08_ctrl->frame_lost : 0U),
                                    (int)((et08_ctrl != NULL) ? et08_ctrl->right.x : 0),
                                    (int)((et08_ctrl != NULL) ? et08_ctrl->right.y : 0));
                    last_online_state = 0U;
                }
            } else {
#if GIMBAL_SPIN_DEMO_ENABLE
                ChassisUpdateSpinFromEt08();
#endif
                if (last_online_state == 0U) {
                    TelemetrySendString("[gmb_et08] remote online -> run\r\n");
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
