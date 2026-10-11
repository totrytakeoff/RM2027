/**
 * @file minimal_gimbal.c
 * @brief 云台控制模块实现
 *
 * 迁移基准:
 * - Yaw 使用“IMU速度控制 -> 主动制动 -> IMU角度保持”三态
 * - Pitch 始终使用 IMU 角度串级，速度意图按时间推进有限角度目标
 * - 底盘通过公开接口读取云台相对底盘夹角，不再直接耦合电机对象
 */

#include "gimbal.h"

#include <math.h>
#include <string.h>

#include "can.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "robot_config.h"
#include "debug.h"
#include "gimbal_axis_state.h"
#include "gimbal_limits.h"
#include "gimbal_units.h"
#include "robot_types.h"
#include "rm_critical.h"
#include "rm_time.h"
#include "user_lib.h"
#include "vision_link.h"

static DJIMotorInstance *motor_yaw = NULL;
static DJIMotorInstance *motor_pitch = NULL;
static uint8_t gimbal_imu_initialized = 0U;

static RobotControlMode_e current_mode = ROBOT_CONTROL_FOLLOW;
static RobotControlMode_e last_mode_logged = ROBOT_CONTROL_FOLLOW;
static uint8_t gimbal_enabled = 0U;
static uint32_t gimbal_debug_last_tick = 0U;

static float yaw_speed_ref_last = 0.0f;
static float pitch_speed_ref_last = 0.0f;
static float yaw_hold_ref = 0.0f;
static float pitch_hold_ref = 0.0f;
static float yaw_current_ff = 0.0f;
static float yaw_base_rate_estimate_rad_s = 0.0f;
static uint8_t yaw_base_rate_initialized = 0U;
static float pitch_current_ff = 0.0f;
static uint32_t pitch_target_last_update_ms = 0U;

/*
 * 自瞄接管：视觉在线且 TRACKING=1 时，角度目标改由视觉下发。
 * 链路本身做自瞄开关门控（VISION_ENABLE=0 时 RmVisionLink_IsOnline()
 * 恒为 false），因此这里再并联一个编译期判断，关闭视觉时整段逻辑
 * 不参与优化，也不影响现有手动路径。
 */
/* 本周期是否采纳了视觉角度目标。VISION_ENABLE=0 时恒为 0。 */
static uint8_t vision_held = 0U;
/* 上一周期是否仍由视觉拥有角度参考，用于检测降级边沿。 */
static uint8_t vision_owner_last = 0U;
/* These three values are written and consumed only by the 5 ms motor task. */
static float motor_yaw_imu_angle_fdb = 0.0f;
static float motor_yaw_imu_speed_fdb = 0.0f;
static float motor_pitch_imu_angle_fdb = 0.0f;
static float motor_pitch_imu_speed_fdb = 0.0f;
static AxisCtrlMode_e last_yaw_ctrl_mode = AXIS_CTRL_ANGLE;
static GimbalAxisState yaw_control_state = {
    .mode = AXIS_CTRL_ANGLE,
};
static uint8_t yaw_pid_reset_request = DJI_MOTOR_PID_RESET_NONE;
static uint8_t pitch_pid_reset_request = DJI_MOTOR_PID_RESET_NONE;

static const GimbalPitchLimitConfig pitch_limit_config = {
    .min_angle_deg = GIMBAL_PITCH_MIN_DEG,
    .max_angle_deg = GIMBAL_PITCH_MAX_DEG,
};

static const GimbalAxisStateConfig yaw_control_config = {
    .brake_speed_epsilon = YAW_BRAKE_SPEED_EPS,
    .brake_stable_count_required = YAW_BRAKE_STABLE_COUNT,
    .brake_timeout_ms = YAW_BRAKE_TIMEOUT_MS,
};

static float MotorTotalAngle(const DJIMotorInstance *motor)
{
    DJI_Motor_Measure_s measure;

    return (motor != NULL && DJIMotorGetMeasure(motor, &measure))
               ? measure.total_angle
               : 0.0f;
}

static float MotorSingleRoundAngle(const DJIMotorInstance *motor)
{
    DJI_Motor_Measure_s measure;

    return (motor != NULL && DJIMotorGetMeasure(motor, &measure))
               ? measure.angle_single_round
               : 0.0f;
}

static float MotorSpeed(const DJIMotorInstance *motor)
{
    DJI_Motor_Measure_s measure;

    return (motor != NULL && DJIMotorGetMeasure(motor, &measure))
               ? measure.speed_aps
               : 0.0f;
}

static float MotorSpeedRadS(const DJIMotorInstance *motor)
{
    DJI_Motor_Measure_s measure;

    return (motor != NULL && DJIMotorGetMeasure(motor, &measure))
               ? measure.speed_rad_s
               : 0.0f;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (!Gimbal_FloatIsFinite(value) ||
        !Gimbal_FloatIsFinite(min_value) ||
        !Gimbal_FloatIsFinite(max_value) ||
        min_value > max_value) {
        return 0.0f;
    }
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float OutputLimitRatio(float output, float max_abs_output)
{
    if (!Gimbal_FloatIsFinite(output) ||
        !Gimbal_FloatIsFinite(max_abs_output) ||
        max_abs_output <= 0.0f) {
        return 0.0f;
    }
    return ClampFloat(output / max_abs_output, -1.0f, 1.0f);
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

static bool ReadAttitude(attitude_t *attitude)
{
    return (gimbal_imu_initialized != 0U) && INS_Read(attitude);
}

static void RequestYawPidReset(uint8_t mask)
{
    yaw_pid_reset_request |= mask & DJI_MOTOR_PID_RESET_ALL;
}

static void RequestPitchPidReset(uint8_t mask)
{
    pitch_pid_reset_request |= mask & DJI_MOTOR_PID_RESET_ALL;
}

static bool PublishYawCommand(Closeloop_Type_e outer_loop,
                              float reference,
                              Motor_Working_Type_e working_state,
                              bool imu_available)
{
    DJIMotorCommand command;

    if (!DJIMotorGetCommand(motor_yaw, &command)) {
        return false;
    }
    command.settings.angle_feedback_source =
        imu_available ? OTHER_FEED : MOTOR_FEED;
    command.settings.speed_feedback_source =
        imu_available ? OTHER_FEED : MOTOR_FEED;
    command.settings.outer_loop_type = outer_loop;
    command.reference = reference;
    command.current_feedforward = yaw_current_ff;
    command.external_input_mask |= DJI_MOTOR_EXTERNAL_CURRENT_FF;
    command.working_state = working_state;
    command.pid_reset_mask = yaw_pid_reset_request;
    if (!DJIMotorPublishCommand(motor_yaw, &command)) {
        return false;
    }

    yaw_pid_reset_request = DJI_MOTOR_PID_RESET_NONE;
    return true;
}

static bool PublishPitchCommand(Closeloop_Type_e outer_loop,
                                float reference,
                                Motor_Working_Type_e working_state,
                                bool imu_available)
{
    DJIMotorCommand command;

    if (!DJIMotorGetCommand(motor_pitch, &command)) {
        return false;
    }
    command.settings.angle_feedback_source =
        imu_available ? OTHER_FEED : MOTOR_FEED;
    command.settings.speed_feedback_source =
        imu_available ? OTHER_FEED : MOTOR_FEED;
    command.settings.outer_loop_type = outer_loop;
    command.reference = reference;
    command.current_feedforward = pitch_current_ff;
    command.external_input_mask |= DJI_MOTOR_EXTERNAL_CURRENT_FF;
    command.working_state = working_state;
    command.pid_reset_mask = pitch_pid_reset_request;
    if (!DJIMotorPublishCommand(motor_pitch, &command)) {
        return false;
    }

    pitch_pid_reset_request = DJI_MOTOR_PID_RESET_NONE;
    return true;
}

static void ResetYawState(void)
{
    GimbalAxisState_Init(&yaw_control_state);
    last_yaw_ctrl_mode = AXIS_CTRL_ANGLE;
}

static float GetYawSpeedFeedback(void)
{
    attitude_t attitude = {0};

    if (ReadAttitude(&attitude)) {
        return Gimbal_RadPerSecToDegPerSec(attitude.Gyro[2]);
    }
    return (motor_yaw != NULL) ? MotorSpeed(motor_yaw) : 0.0f;
}

static float GetYawAngleFeedback(bool imu_available,
                                 const attitude_t *attitude)
{
    if (imu_available && attitude != NULL &&
        Gimbal_FloatIsFinite(attitude->YawTotalAngle)) {
        return attitude->YawTotalAngle;
    }
    return MotorTotalAngle(motor_yaw);
}

static void UpdateYawControlMode(const Input_Data_t *input,
                                 uint32_t now_ms,
                                 bool imu_available,
                                 const attitude_t *attitude)
{
    float angle_feedback;
    float speed_feedback;
    GimbalAxisTransition transition;

    if (input == NULL) {
        return;
    }

    angle_feedback = GetYawAngleFeedback(imu_available, attitude);
    speed_feedback = imu_available && attitude != NULL
                         ? Gimbal_RadPerSecToDegPerSec(
                               attitude->Gyro[2])
                         : MotorSpeed(motor_yaw);

    if (!GimbalAxisState_Update(
            &yaw_control_state,
            &yaw_control_config,
            input->yaw_control_active != 0U,
            speed_feedback,
            now_ms,
            &transition)) {
        GimbalAxisState_Init(&yaw_control_state);
        yaw_hold_ref = angle_feedback;
        RequestYawPidReset(DJI_MOTOR_PID_RESET_ALL);
        return;
    }

    /* 速度/制动阶段持续跟随当前角，进入角度保持时不会产生位置阶跃。 */
    if (yaw_control_state.mode != AXIS_CTRL_ANGLE || transition.changed) {
        yaw_hold_ref = angle_feedback;
    }

    if (transition.changed) {
        if (transition.current_mode == AXIS_CTRL_BRAKE) {
            RequestYawPidReset(DJI_MOTOR_PID_RESET_SPEED);
        } else {
            RequestYawPidReset(DJI_MOTOR_PID_RESET_ALL);
        }
    }
}

static void UpdatePitchGravityFeedforward(bool imu_available,
                                          float pitch_imu_deg)
{
    float pitch_ff_raw;

    if (motor_pitch == NULL || !imu_available ||
        !Gimbal_FloatIsFinite(pitch_imu_deg)) {
        pitch_current_ff = 0.0f;
        return;
    }

    pitch_ff_raw = Gimbal_GravityFeedforward(
        pitch_imu_deg, PITCH_GRAVITY_HORIZONTAL_DEG,
        PITCH_GRAVITY_FF_K, PITCH_GRAVITY_FF_MAX);
    pitch_current_ff = pitch_current_ff * PITCH_FF_LPF +
                       pitch_ff_raw * (1.0f - PITCH_FF_LPF);
}

static void UpdateYawBaseRateFeedforward(bool imu_available,
                                         float imu_yaw_rate_rad_s,
                                         bool spin_mode)
{
    float motor_relative_rate_rad_s;
    float raw_base_rate_rad_s;

    if (!spin_mode || !imu_available || motor_yaw == NULL ||
        !Gimbal_FloatIsFinite(imu_yaw_rate_rad_s)) {
        yaw_current_ff = 0.0f;
        yaw_base_rate_estimate_rad_s = 0.0f;
        yaw_base_rate_initialized = 0U;
        return;
    }

    motor_relative_rate_rad_s = MotorSpeedRadS(motor_yaw);
    raw_base_rate_rad_s = Gimbal_EstimateBaseRateRadS(
        imu_yaw_rate_rad_s, motor_relative_rate_rad_s);
    if (!Gimbal_FloatIsFinite(raw_base_rate_rad_s)) {
        yaw_current_ff = 0.0f;
        yaw_base_rate_estimate_rad_s = 0.0f;
        yaw_base_rate_initialized = 0U;
        return;
    }

    if (yaw_base_rate_initialized == 0U) {
        yaw_base_rate_estimate_rad_s = raw_base_rate_rad_s;
        yaw_base_rate_initialized = 1U;
    } else {
        yaw_base_rate_estimate_rad_s =
            yaw_base_rate_estimate_rad_s * YAW_BASE_RATE_FF_LPF +
            raw_base_rate_rad_s * (1.0f - YAW_BASE_RATE_FF_LPF);
    }
    yaw_current_ff = Gimbal_BaseRateCurrentFeedforward(
        yaw_base_rate_estimate_rad_s,
        YAW_BASE_RATE_CURRENT_FF_K,
        YAW_BASE_RATE_FF_DEADBAND_RAD_S,
        YAW_BASE_RATE_CURRENT_FF_MAX);
}

static float GetPitchSpeedFeedback(void)
{
    attitude_t attitude = {0};

    if (ReadAttitude(&attitude)) {
        return Gimbal_RadPerSecToDegPerSec(attitude.Gyro[0]);
    }
    /* Pitch 控制坐标固定为 IMU，禁止在运行时静默切换到反向电机坐标。 */
    return 0.0f;
}

bool Gimbal_Init(void)
{
    Motor_Init_Config_s yaw_config;
    Motor_Init_Config_s pitch_config;
    attitude_t attitude;

    if (!GimbalPitchLimit_IsConfigValid(&pitch_limit_config)) {
        MDBG_GMB("invalid pitch limit config");
        return false;
    }
    if (!Gimbal_FloatIsFinite(YAW_BASE_RATE_CURRENT_FF_K) ||
        YAW_BASE_RATE_CURRENT_FF_K < 0.0f ||
        !Gimbal_FloatIsFinite(YAW_BASE_RATE_CURRENT_FF_MAX) ||
        YAW_BASE_RATE_CURRENT_FF_MAX < 0.0f ||
        !Gimbal_FloatIsFinite(YAW_BASE_RATE_FF_LPF) ||
        YAW_BASE_RATE_FF_LPF < 0.0f ||
        YAW_BASE_RATE_FF_LPF >= 1.0f ||
        !Gimbal_FloatIsFinite(YAW_BASE_RATE_FF_DEADBAND_RAD_S) ||
        YAW_BASE_RATE_FF_DEADBAND_RAD_S < 0.0f ||
        !Gimbal_FloatIsFinite(YAW_SPEED_I_MAX) || YAW_SPEED_I_MAX < 0.0f ||
        YAW_SPEED_I_MAX > YAW_SPEED_MAX_OUT ||
        !Gimbal_FloatIsFinite(PITCH_SPEED_I_MAX) ||
        PITCH_SPEED_I_MAX < 0.0f ||
        PITCH_SPEED_I_MAX > PITCH_SPEED_MAX_OUT ||
        !Gimbal_FloatIsFinite(PITCH_GRAVITY_FF_K) ||
        !Gimbal_FloatIsFinite(PITCH_GRAVITY_FF_MAX) ||
        PITCH_GRAVITY_FF_MAX < 0.0f ||
        !Gimbal_FloatIsFinite(PITCH_FF_LPF) || PITCH_FF_LPF < 0.0f ||
        PITCH_FF_LPF >= 1.0f ||
        PITCH_MOTOR_OUTPUT_REVERSED > 1U ||
        (YAW_SPEED_MAX_OUT + YAW_BASE_RATE_CURRENT_FF_MAX) >
            GM6020_COMMAND_LIMIT ||
        (PITCH_SPEED_MAX_OUT + PITCH_GRAVITY_FF_MAX) >
            GM6020_COMMAND_LIMIT) {
        MDBG_GMB("invalid gimbal control config");
        return false;
    }

    gimbal_imu_initialized =
        (INS_InitWithTimeout(ROBOT_IMU_INIT_TIMEOUT_MS) != NULL) ? 1U : 0U;
    motor_yaw_imu_angle_fdb = 0.0f;
    motor_yaw_imu_speed_fdb = 0.0f;
    motor_pitch_imu_angle_fdb = 0.0f;
    motor_pitch_imu_speed_fdb = 0.0f;

    if (gimbal_imu_initialized == 0U) {
        MDBG_SYS("INS init failed status=%u sensor_error=0x%02x",
                 (unsigned)INS_GetInitStatus(),
                 (unsigned)INS_GetSensorInitError());
        return false;
    }

    yaw_config = (Motor_Init_Config_s){
        .motor_type = GM6020,
        .can_init_config = {
            .can_handle = &YAW_CAN,
            .tx_id = YAW_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = YAW_ANGLE_KP,
                .Ki = YAW_ANGLE_KI,
                .Kd = YAW_ANGLE_KD,
                .IntegralLimit = 500.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = YAW_ANGLE_MAX_OUT,
            },
            .speed_PID = {
                .Kp = YAW_SPEED_KP,
                .Ki = YAW_SPEED_KI,
                .Kd = YAW_SPEED_KD,
                .IntegralLimit = YAW_SPEED_I_MAX,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = YAW_SPEED_MAX_OUT,
            },
            .other_angle_feedback_ptr = (gimbal_imu_initialized != 0U)
                                            ? &motor_yaw_imu_angle_fdb
                                            : NULL,
            .other_speed_feedback_ptr = (gimbal_imu_initialized != 0U)
                                            ? &motor_yaw_imu_speed_fdb
                                            : NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .speed_unit = MOTOR_SPEED_DEG_PER_SEC,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    pitch_config = (Motor_Init_Config_s){
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
                .IntegralLimit = PITCH_SPEED_I_MAX,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = PITCH_SPEED_MAX_OUT,
            },
            .other_angle_feedback_ptr = (gimbal_imu_initialized != 0U)
                                            ? &motor_pitch_imu_angle_fdb
                                            : NULL,
            .other_speed_feedback_ptr = (gimbal_imu_initialized != 0U)
                                            ? &motor_pitch_imu_speed_fdb
                                            : NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .speed_unit = MOTOR_SPEED_DEG_PER_SEC,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = PITCH_MOTOR_OUTPUT_REVERSED
                                         ? FEEDBACK_DIRECTION_REVERSE
                                         : FEEDBACK_DIRECTION_NORMAL,
        },
    };

    motor_yaw = DJIMotorInit(&yaw_config);
    motor_pitch = DJIMotorInit(&pitch_config);

    if ((motor_yaw == NULL) || (motor_pitch == NULL)) {
        MDBG_SYS("gimbal motor registration failed yaw=%u pitch=%u",
                 motor_yaw != NULL ? 1U : 0U,
                 motor_pitch != NULL ? 1U : 0U);
    }

    ResetYawState();
    vision_held = 0U;
    vision_owner_last = 0U;
    if (motor_yaw != NULL) {
        bool imu_available = ReadAttitude(&attitude);

        yaw_hold_ref = imu_available ? attitude.YawTotalAngle
                                     : MotorTotalAngle(motor_yaw);
        RequestYawPidReset(DJI_MOTOR_PID_RESET_ALL);
        (void)PublishYawCommand(ANGLE_LOOP, yaw_hold_ref, MOTOR_STOP,
                                imu_available);
    }

    if (motor_pitch != NULL) {
        bool imu_available = ReadAttitude(&attitude);

        pitch_hold_ref = GimbalPitchLimit_ClampAngleReference(
            PITCH_STARTUP_CENTER_DEG, imu_available,
            imu_available ? attitude.Pitch : 0.0f,
            &pitch_limit_config);
        RequestPitchPidReset(DJI_MOTOR_PID_RESET_ALL);
        (void)PublishPitchCommand(ANGLE_LOOP, pitch_hold_ref, MOTOR_STOP,
                                  imu_available);
    }

    current_mode = ROBOT_CONTROL_FOLLOW;
    last_mode_logged = current_mode;
    MDBG_GMB("limits yaw_dps=%ld pitch_dps=%ld gravity_k=%ld gravity_max=%ld horizontal_x10=%ld",
             (long)GIMBAL_YAW_MAX_SPEED_DEG_S,
             (long)GIMBAL_PITCH_MAX_SPEED_DEG_S,
             (long)PITCH_GRAVITY_FF_K,
             (long)PITCH_GRAVITY_FF_MAX,
             (long)(PITCH_GRAVITY_HORIZONTAL_DEG * 10.0f));
    return (gimbal_imu_initialized != 0U) && (motor_yaw != NULL) &&
           (motor_pitch != NULL);
}

void Gimbal_MotorStep(void)
{
    attitude_t attitude;

    if (!ReadAttitude(&attitude)) {
        return;
    }

    motor_yaw_imu_angle_fdb = attitude.YawTotalAngle;
    motor_yaw_imu_speed_fdb =
        Gimbal_RadPerSecToDegPerSec(attitude.Gyro[2]);
    motor_pitch_imu_angle_fdb = attitude.Pitch;
    motor_pitch_imu_speed_fdb =
        Gimbal_RadPerSecToDegPerSec(attitude.Gyro[0]);
}

#if VISION_ENABLE
/*
 * 自瞄状态打印，用来回答实板联调时唯一真正要紧的问题：
 * "为什么没接管"。计数按**每秒增量**打印而不是累计值——累计值要靠人工
 * 做差才知道当前还在不在收帧，而这恰恰是最常需要看的一条。
 * 各字段的含义（均为最近一个统计周期内的增量）：
 *   rx   = 0          → 一个字节都没收到：接线 / 波特率 / 上位机没在发
 *   crc/fmt > 0       → 收到了但帧不同步：协议版本或长度字段不一致
 *   tgt > 0 但 idle   → 目标帧正常，是 TRACKING=0 或云台不在角度保持态
 *   rej > 0           → 目标被判为非有限值（NaN/Inf），算法侧发散了
 * 非自瞄模式下不打印，但基线仍跟随更新，避免刚切进自瞄时把历史计数
 * 当成"这一秒收到的帧"打出来。
 */
static void LogVisionStatus(uint32_t now_ms, bool target_ready)
{
    static uint32_t last_log_ms = 0U;
    static RmVisionLinkStats last = {0};
    RmVisionLinkStats stats;
    bool report;

    if (!RmVisionLink_GetStats(&stats)) {
        return;
    }

    report = (current_mode == ROBOT_CONTROL_AUTO_AIM) &&
             ((now_ms - last_log_ms) >= VISION_STATUS_LOG_PERIOD_MS);
    if (report) {
        uint32_t reject_now =
            stats.rejected_not_armed + stats.rejected_nonfinite;
        uint32_t reject_last =
            last.rejected_not_armed + last.rejected_nonfinite;

        last_log_ms = now_ms;
        MDBG_GMB("aim %s online=%u rx=%u crc=%u fmt=%u tgt=%u rej=%u id=%u n=%u",
                 target_ready ? "ACTIVE" : "idle",
                 RmVisionLink_IsOnline() ? 1U : 0U,
                 (unsigned)(stats.rx_frames - last.rx_frames),
                 (unsigned)(stats.crc_errors - last.crc_errors),
                 (unsigned)(stats.format_errors - last.format_errors),
                 (unsigned)(stats.target_frames - last.target_frames),
                 (unsigned)(reject_now - reject_last),
                 (unsigned)stats.last_target_id,
                 (unsigned)stats.last_armors_num);
        last = stats;
    } else if (current_mode != ROBOT_CONTROL_AUTO_AIM) {
        last = stats;
        last_log_ms = now_ms;
    }
}
#endif

void Gimbal_Update(Input_Data_t *input)
{
    uint32_t now_ms;
    uint32_t pitch_target_elapsed_ms;
    float yaw_speed_cmd;
    float yaw_reference;
    float pitch_target_rate_cmd;
    float pitch_intent;
    float pitch_reference;
    Closeloop_Type_e yaw_outer_loop;
    attitude_t attitude = {0};
    bool imu_available;
    Gimbal_Cmd_t cmd = {0};

    if (input == NULL || !input->online || input->emergency_stop) {
        MDBG_GMB("stop by input offline/estop");
        Gimbal_Stop();
        return;
    }

    if (motor_yaw == NULL || motor_pitch == NULL) {
        return;
    }

    now_ms = RmTime_NowMs();
    imu_available = ReadAttitude(&attitude);
    if (!imu_available || !Gimbal_FloatIsFinite(attitude.Pitch) ||
        !Gimbal_FloatIsFinite(attitude.YawTotalAngle) ||
        !Gimbal_FloatIsFinite(attitude.Gyro[0]) ||
        !Gimbal_FloatIsFinite(attitude.Gyro[2])) {
        MDBG_GMB("stop by invalid IMU feedback");
        Gimbal_Stop();
        return;
    }

    if (!gimbal_enabled) {
        gimbal_enabled = 1U;
        yaw_hold_ref = attitude.YawTotalAngle;
        ResetYawState();
        RequestYawPidReset(DJI_MOTOR_PID_RESET_ALL);
        pitch_hold_ref = GimbalPitchLimit_ClampAngleReference(
            PITCH_STARTUP_CENTER_DEG, true, attitude.Pitch,
            &pitch_limit_config);
        pitch_target_last_update_ms = now_ms;
        RequestPitchPidReset(DJI_MOTOR_PID_RESET_ALL);
    }

    current_mode = input->control_mode;
    if (current_mode != last_mode_logged) {
        if (current_mode == ROBOT_CONTROL_SPIN ||
            last_mode_logged == ROBOT_CONTROL_SPIN) {
            RequestYawPidReset(DJI_MOTOR_PID_RESET_SPEED);
        }
        MDBG_GMB("mode switch %u -> %u", (unsigned)last_mode_logged, (unsigned)current_mode);
        last_mode_logged = current_mode;
    }

    UpdateYawBaseRateFeedforward(
        imu_available,
        imu_available ? attitude.Gyro[2] : 0.0f,
        current_mode == ROBOT_CONTROL_SPIN);
    UpdatePitchGravityFeedforward(imu_available, attitude.Pitch);

    yaw_speed_cmd = ClampFloat(input->gimbal_yaw_intent, -1.0f, 1.0f) *
                    GIMBAL_YAW_MAX_SPEED_DEG_S;
    if (input->yaw_control_active == 0U) {
        yaw_speed_cmd = 0.0f;
    }
    UpdateYawControlMode(input, now_ms, imu_available, &attitude);

    /* 自瞄接管只在角度保持态发生：速度态仍由操作手主导（见 §1.2）。 */
    vision_held = 0U;
#if VISION_ENABLE
    {
        RmVisionTarget target = {0};
        bool target_ready;

        /*
         * RmVisionLink_HasTarget() 已包含全部安全门：链路在线、自瞄已开启、
         * TRACKING=1、浮点状态有限。这里只需再确认云台处于角度保持态。
         */
        target_ready = (current_mode == ROBOT_CONTROL_AUTO_AIM) &&
                       (yaw_control_state.mode == AXIS_CTRL_ANGLE) &&
                       RmVisionLink_HasTarget() &&
                       RmVisionLink_GetTarget(&target) &&
                       RmVisionTarget_HasFiniteState(&target);

        if (target_ready) {
            /*
             * 解算复用 gimbal_units.h 中已由主机测试覆盖的两个函数，
             * 避免同一套符号换算在 gimbal.c 里再抄一遍。
             * 退化输入（水平距离过小、非有限值）返回 NaN，此处据此放弃本帧。
             *
             * 方向开关只作用于 Yaw 推进量，不作用于解算本身：关掉 Yaw 跟随
             * 仍需要 target_ready 为真，Pitch 跟随与日志行为都不受影响。
             */
            float yaw_target_deg = Gimbal_TargetYawReferenceDeg(
                yaw_hold_ref, target.x, target.y);
            float pitch_delta_deg = Gimbal_TargetPitchReferenceDeg(
                target.x, target.y, target.z, VISION_MIN_HORIZONTAL_DIST_M);

            if (Gimbal_FloatIsFinite(yaw_target_deg) &&
                Gimbal_FloatIsFinite(pitch_delta_deg)) {
                float yaw_delta_deg = yaw_target_deg - yaw_hold_ref;
                float yaw_limit = VISION_MAX_RATE_DEG_S *
                                  (float)MAIN_LOOP_PERIOD_MS * 0.001f;
                float pitch_limit = VISION_MAX_RATE_PITCH_DEG_S *
                                    (float)MAIN_LOOP_PERIOD_MS * 0.001f;

                if (VISION_YAW_FOLLOW_ENABLE == 0U) {
                    yaw_delta_deg = 0.0f;
                }

                /*
                 * 单周期限幅：避免目标跳变或跟踪器重收敛时把角度参考甩到
                 * 极限。按标称控制周期计算，任务延迟时步长偏小，只慢不猛。
                 */
                if (yaw_limit > GIMBAL_YAW_MAX_SPEED_DEG_S) {
                    yaw_limit = GIMBAL_YAW_MAX_SPEED_DEG_S;
                }
                if (pitch_limit > GIMBAL_PITCH_MAX_SPEED_DEG_S) {
                    pitch_limit = GIMBAL_PITCH_MAX_SPEED_DEG_S;
                }

                /*
                 * 关键：推进的是**相对增量**，不是把 atan2 的绝对值直接赋给
                 * 参考。上位机 tracker 工作在云台系(target_frame = gimbal_link)，
                 * 输出的是相对当前指向的方位/仰角；Pitch 的解算结果已经是
                 * 相对修正量，不能再减去当前 IMU Pitch，否则会重复计算姿态。
                 * 直接取绝对值会在云台转动后把目标拽回旧方向，形成正反馈持续
                 * 打偏。
                 *
                 * 这里**不能**用 WrapAngleDeg180：yaw_hold_ref 与
                 * attitude.YawTotalAngle 同为多圈累计角，折到 [-180,180] 会在
                 * 跨越边界时产生 360 deg 阶跃。
                 */
                yaw_hold_ref = Gimbal_AdvanceAngleRefDeg(
                    yaw_hold_ref, yaw_delta_deg, yaw_limit);
                pitch_hold_ref = GimbalPitchLimit_ClampAngleReference(
                    Gimbal_AdvanceAngleRefDeg(pitch_hold_ref, pitch_delta_deg,
                                              pitch_limit),
                    true, attitude.Pitch, &pitch_limit_config);
                vision_held = 1U;
            }
        }

        /*
         * 视觉接管结束后必须把角度参考对齐到当前姿态。否则 target_usable
         * 虽然已经清零，角度环仍会继续追上一帧视觉目标，无法真正降级到
         * 当前角度保持。该边沿同时覆盖 TRACKING 清零、链路超时和离开自瞄。
         */
        if (vision_owner_last != 0U && vision_held == 0U) {
            yaw_hold_ref = GetYawAngleFeedback(imu_available, &attitude);
            pitch_hold_ref = GimbalPitchLimit_ClampAngleReference(
                attitude.Pitch, true, attitude.Pitch, &pitch_limit_config);
            RequestYawPidReset(DJI_MOTOR_PID_RESET_ALL);
            RequestPitchPidReset(DJI_MOTOR_PID_RESET_ALL);
        }
        vision_owner_last = vision_held;

        LogVisionStatus(now_ms, target_ready);
    }
#endif

    if (yaw_control_state.mode == AXIS_CTRL_SPEED) {
        yaw_outer_loop = SPEED_LOOP;
        yaw_reference = yaw_speed_cmd;
        yaw_hold_ref = GetYawAngleFeedback(imu_available, &attitude);
    } else if (yaw_control_state.mode == AXIS_CTRL_BRAKE) {
        yaw_outer_loop = SPEED_LOOP;
        yaw_reference = 0.0f;
    } else {
        yaw_outer_loop = ANGLE_LOOP;
        yaw_reference = yaw_hold_ref;
    }
    (void)PublishYawCommand(yaw_outer_loop, yaw_reference,
                            MOTOR_ENALBED, imu_available);

    pitch_target_elapsed_ms = now_ms - pitch_target_last_update_ms;
    pitch_target_last_update_ms = now_ms;
    if (pitch_target_elapsed_ms > PITCH_TARGET_INTEGRATION_MAX_DT_MS) {
        pitch_target_elapsed_ms = PITCH_TARGET_INTEGRATION_MAX_DT_MS;
    }
    pitch_intent = input->pitch_control_active != 0U
                       ? ClampFloat(input->gimbal_pitch_intent, -1.0f, 1.0f)
                       : 0.0f;
    pitch_target_rate_cmd = pitch_intent * GIMBAL_PITCH_MAX_SPEED_DEG_S;
    /*
     * 自瞄采纳时 pitch_hold_ref 已由视觉推进，这里不得再按手动意图积分，
     * 否则操作手残留的摇杆量会与视觉目标叠加。
     */
    if (vision_held == 0U) {
        pitch_hold_ref = GimbalPitchLimit_AdvanceAngleReference(
            pitch_hold_ref, pitch_intent, GIMBAL_PITCH_MAX_SPEED_DEG_S,
            (float)pitch_target_elapsed_ms * 0.001f, &pitch_limit_config);
    }
    pitch_reference = GimbalPitchLimit_ClampAngleReference(
        pitch_hold_ref, true, attitude.Pitch, &pitch_limit_config);
    pitch_hold_ref = pitch_reference;
    (void)PublishPitchCommand(ANGLE_LOOP, pitch_reference,
                              MOTOR_ENALBED, imu_available);

    if (yaw_control_state.mode != last_yaw_ctrl_mode) {
        MDBG_GMB("yaw loop -> %u", (unsigned)yaw_control_state.mode);
        last_yaw_ctrl_mode = yaw_control_state.mode;
    }
    yaw_speed_ref_last = yaw_speed_cmd;
    pitch_speed_ref_last = pitch_target_rate_cmd;

    cmd.yaw_speed = yaw_speed_cmd;
    cmd.pitch_speed = pitch_target_rate_cmd;
    cmd.yaw_angle = yaw_hold_ref;
    cmd.pitch_angle = pitch_hold_ref;
    cmd.mode = current_mode;
    cmd.yaw_ctrl_mode = yaw_control_state.mode;
    cmd.manual_pitch = input->pitch_control_active != 0U ? 1U : 0U;
    cmd.pitch_ctrl_mode = AXIS_CTRL_ANGLE;
    cmd.control_mode = CTRL_ENABLE;
    cmd.ref_type = yaw_control_state.mode != AXIS_CTRL_ANGLE
                       ? REF_SPEED
                       : REF_ANGLE;
    g_robot.gimbal = cmd;

    if ((now_ms - gimbal_debug_last_tick) >= GIMBAL_DEBUG_DETAIL_PERIOD_MS) {
        gimbal_debug_last_tick = now_ms;
        MDBG_GMB("mode=%u yaw_loop=%u pitch_loop=%u yaw_ref=%ld yaw_imu=%ld off=%ld pitch_ref=%ld pitch_enc=%ld pitch_imu=%ld pitch_spd=%ld ff=%ld",
                 (unsigned)current_mode,
                 (unsigned)yaw_control_state.mode,
                 (unsigned)AXIS_CTRL_ANGLE,
                 (long)(yaw_hold_ref * 10.0f),
                 (long)(Gimbal_GetYawIMUAngle() * 10.0f),
                 (long)(Gimbal_GetYawOffsetLogicDeg() * 10.0f),
                 (long)(pitch_hold_ref * 10.0f),
                 (long)(Gimbal_GetPitchEncoderAngle() * 10.0f),
                 (long)(attitude.Pitch * 10.0f),
                 (long)(GetPitchSpeedFeedback() * 10.0f),
                 (long)pitch_current_ff);
    }
}

void Gimbal_Stop(void)
{
    gimbal_enabled = 0U;
    vision_held = 0U;
    vision_owner_last = 0U;
    gimbal_debug_last_tick = 0U;
    yaw_speed_ref_last = 0.0f;
    pitch_speed_ref_last = 0.0f;
    pitch_target_last_update_ms = 0U;
    yaw_current_ff = 0.0f;
    yaw_base_rate_estimate_rad_s = 0.0f;
    yaw_base_rate_initialized = 0U;
    pitch_current_ff = 0.0f;
    ResetYawState();
    RequestYawPidReset(DJI_MOTOR_PID_RESET_ALL);
    RequestPitchPidReset(DJI_MOTOR_PID_RESET_ALL);

    g_robot.gimbal = (Gimbal_Cmd_t){
        .mode = current_mode,
        .yaw_ctrl_mode = AXIS_CTRL_ANGLE,
        .pitch_ctrl_mode = AXIS_CTRL_ANGLE,
        .control_mode = CTRL_ZERO_FORCE,
        .ref_type = REF_ANGLE,
    };

    if (motor_yaw != NULL) {
        (void)PublishYawCommand(ANGLE_LOOP, yaw_hold_ref, MOTOR_STOP,
                                INS_IsReady());
    }
    if (motor_pitch != NULL) {
        (void)PublishPitchCommand(ANGLE_LOOP, pitch_hold_ref, MOTOR_STOP,
                                  INS_IsReady());
    }
}

RobotControlMode_e Gimbal_GetMode(void)
{
    return current_mode;
}

float Gimbal_GetYawSpeedRef(void)
{
    return yaw_speed_ref_last;
}

float Gimbal_GetYawSpeedFdb(void)
{
    return GetYawSpeedFeedback();
}

float Gimbal_GetPitchSpeedRef(void)
{
    return pitch_speed_ref_last;
}

float Gimbal_GetPitchSpeedFdb(void)
{
    return GetPitchSpeedFeedback();
}

float Gimbal_GetPitchTargetAngle(void)
{
    return pitch_hold_ref;
}

float Gimbal_GetYawTargetAngle(void)
{
    return yaw_hold_ref;
}

float Gimbal_GetYawEncoderAngle(void)
{
    if (motor_yaw == NULL) {
        return 0.0f;
    }
    return MotorTotalAngle(motor_yaw);
}

float Gimbal_GetPitchEncoderAngle(void)
{
    if (motor_pitch == NULL) {
        return 0.0f;
    }
    return MotorTotalAngle(motor_pitch);
}

float Gimbal_GetYawIMUAngle(void)
{
    attitude_t attitude;

    return ReadAttitude(&attitude) ? attitude.YawTotalAngle : 0.0f;
}

float Gimbal_GetPitchIMUAngle(void)
{
    attitude_t attitude;

    return ReadAttitude(&attitude) ? attitude.Pitch : 0.0f;
}

float Gimbal_GetPitchGravityFeedforward(void)
{
    return pitch_current_ff;
}

float Gimbal_GetYawOffsetRawDeg(void)
{
    if (motor_yaw == NULL) {
        return 0.0f;
    }
    return WrapAngleDeg180(MotorSingleRoundAngle(motor_yaw) -
                           YAW_ALIGN_ANGLE_DEG);
}

float Gimbal_GetYawOffsetLogicDeg(void)
{
    return WrapAngleDeg180(Gimbal_GetYawOffsetRawDeg() - YAW_OFFSET_LOGIC_ZERO_DEG);
}

float Gimbal_GetYawOffsetLogicRad(void)
{
    return Gimbal_GetYawOffsetLogicDeg() * ((float)M_PI / 180.0f);
}

float Gimbal_GetYawRelativeSpeedRadS(void)
{
    DJI_Motor_Measure_s measure;

    if (motor_yaw == NULL) {
        return 0.0f;
    }
    return DJIMotorGetMeasure(motor_yaw, &measure) ? measure.speed_rad_s
                                                   : 0.0f;
}

float Gimbal_GetYawLogicAngle(void)
{
    attitude_t attitude;

    if (!ReadAttitude(&attitude)) {
        return 0.0f;
    }
    return WrapAngleDeg180(attitude.YawTotalAngle -
                           IMU_YAW_LOGIC_ZERO_TOTAL_DEG);
}

bool Gimbal_AreMotorsHealthy(void)
{
    return DJIMotorIsOnline(motor_yaw) && DJIMotorIsOnline(motor_pitch);
}

bool Gimbal_IsHealthy(void)
{
    return (gimbal_imu_initialized != 0U) && INS_IsReady() &&
           Gimbal_AreMotorsHealthy();
}

AxisCtrlMode_e Gimbal_GetPitchCtrlMode(void)
{
    return AXIS_CTRL_ANGLE;
}

AxisCtrlMode_e Gimbal_GetYawCtrlMode(void)
{
    return yaw_control_state.mode;
}

static void FillAxisTuningSnapshot(
    GimbalAxisTuningSnapshot *axis,
    const DJI_Motor_Measure_s *measure,
    const DJIMotorControlSnapshot *control)
{
    if (axis == NULL || measure == NULL || control == NULL) {
        return;
    }

    axis->encoder_ecd = measure->ecd;
    axis->motor_current_feedback = measure->real_current;
    axis->encoder_single_deg = measure->angle_single_round;
    axis->encoder_total_deg = measure->total_angle;
    axis->encoder_speed_deg_s = measure->speed_aps;
    axis->angle_reference_deg = control->angle.reference;
    axis->angle_feedback_deg = control->angle.measure;
    axis->angle_error_deg = control->angle.error;
    axis->angle_p_deg_s = control->angle.p_output;
    axis->angle_i_deg_s = control->angle.i_output;
    axis->angle_d_deg_s = control->angle.d_output;
    axis->angle_output_deg_s = control->angle.output;
    axis->angle_output_limit_ratio = OutputLimitRatio(
        control->angle.output, control->angle.max_output);
    axis->speed_reference_deg_s = control->speed.reference;
    axis->speed_feedback_deg_s = control->speed.measure;
    axis->speed_error_deg_s = control->speed.error;
    axis->speed_p_current = control->speed.p_output;
    axis->speed_i_current = control->speed.i_output;
    axis->speed_d_current = control->speed.d_output;
    axis->speed_output_current = control->speed.output;
    axis->speed_output_limit_ratio = OutputLimitRatio(
        control->speed.output, control->speed.max_output);
}

bool Gimbal_GetTuningSnapshot(GimbalTuningSnapshot *snapshot)
{
    DJI_Motor_Measure_s yaw_measure = {0};
    DJI_Motor_Measure_s pitch_measure = {0};
    DJIMotorControlSnapshot yaw_control = {0};
    DJIMotorControlSnapshot pitch_control = {0};
    attitude_t attitude = {0};
    RmCriticalState critical_state;
    bool yaw_valid;
    bool pitch_valid;
    bool imu_valid;

    if (snapshot == NULL) {
        return false;
    }
    memset(snapshot, 0, sizeof(*snapshot));

    yaw_valid = motor_yaw != NULL &&
                DJIMotorGetMeasure(motor_yaw, &yaw_measure) &&
                DJIMotorGetControlSnapshot(motor_yaw, &yaw_control);
    pitch_valid = motor_pitch != NULL &&
                  DJIMotorGetMeasure(motor_pitch, &pitch_measure) &&
                  DJIMotorGetControlSnapshot(motor_pitch, &pitch_control);
    imu_valid = ReadAttitude(&attitude);

    critical_state = RmCritical_Enter();
    snapshot->yaw.control_mode = yaw_control_state.mode;
    snapshot->yaw.operator_speed_command_deg_s = yaw_speed_ref_last;
    snapshot->yaw.hold_target_deg = yaw_hold_ref;
    snapshot->yaw.current_feedforward = yaw_current_ff;
    snapshot->pitch.control_mode = AXIS_CTRL_ANGLE;
    snapshot->pitch.operator_speed_command_deg_s = pitch_speed_ref_last;
    snapshot->pitch.hold_target_deg = pitch_hold_ref;
    snapshot->pitch.current_feedforward = pitch_current_ff;
    snapshot->yaw_base_rate_estimate_rad_s =
        yaw_base_rate_estimate_rad_s;
    RmCritical_Exit(critical_state);

    if (yaw_valid) {
        FillAxisTuningSnapshot(&snapshot->yaw, &yaw_measure, &yaw_control);
        snapshot->yaw_offset_raw_deg = WrapAngleDeg180(
            yaw_measure.angle_single_round - YAW_ALIGN_ANGLE_DEG);
        snapshot->yaw_offset_logic_deg = WrapAngleDeg180(
            snapshot->yaw_offset_raw_deg - YAW_OFFSET_LOGIC_ZERO_DEG);
        snapshot->yaw_relative_speed_rad_s = yaw_measure.speed_rad_s;
    }
    if (pitch_valid) {
        FillAxisTuningSnapshot(&snapshot->pitch, &pitch_measure,
                               &pitch_control);
    }
    if (imu_valid) {
        snapshot->yaw.imu_angle_deg = attitude.YawTotalAngle;
        snapshot->yaw.imu_gyro_rad_s = attitude.Gyro[2];
        snapshot->yaw.imu_gyro_deg_s =
            Gimbal_RadPerSecToDegPerSec(attitude.Gyro[2]);
        snapshot->pitch.imu_angle_deg = attitude.Pitch;
        snapshot->pitch.imu_gyro_rad_s = attitude.Gyro[0];
        snapshot->pitch.imu_gyro_deg_s =
            Gimbal_RadPerSecToDegPerSec(attitude.Gyro[0]);
    }

    return yaw_valid && pitch_valid && imu_valid;
}
