/**
 * @file minimal_chassis.c
 * @brief 底盘控制模块实现
 *
 * 迁移基准:
 * - 输入平移指令始终定义在云台坐标系
 * - 底盘使用 yaw 编码器相对角完成坐标变换
 * - FOLLOW 模式通过相对夹角闭环追随云台
 * - SPIN 模式保留旋转轮速，平移按四轮剩余余量缩放
 */

#include "chassis.h"

#include <math.h>
#include <string.h>

#include "can.h"
#include "dji_motor.h"
#include "robot_config.h"
#include "chassis_follow.h"
#include "chassis_kinematics.h"
#include "debug.h"
#include "gimbal.h"
#include "referee.h"
#include "robot_types.h"
#include "rm_critical.h"
#include "rm_time.h"
#include "user_lib.h"

static DJIMotorInstance *motor_fr = NULL;
static DJIMotorInstance *motor_fl = NULL;
static DJIMotorInstance *motor_br = NULL;
static DJIMotorInstance *motor_bl = NULL;

static float last_wz = 0.0f;
static float last_wheel_ref[4] = {0.0f};
static float last_power_scale = 1.0f;
static float filtered_vx = 0.0f;
static float filtered_vy = 0.0f;
static float filtered_wz = 0.0f;
static float follow_wz_integral = 0.0f;
static uint32_t chassis_last_tick = 0U;
static RobotControlMode_e last_control_mode = ROBOT_CONTROL_FOLLOW;
/*
 * 上电/安全停机恢复以及小陀螺退出后，首个 FOLLOW 周期必须立即开始
 * 消除云台相对底盘的标定零位误差，不能再被角速度低通削弱一拍。
 */
static uint8_t follow_recovery_pending = 1U;
static ChassisTuningSnapshot last_tuning_snapshot;

static float ReadMotorSpeedRadS(const DJIMotorInstance *motor)
{
    DJI_Motor_Measure_s measure;

    if (motor == NULL) {
        return 0.0f;
    }
    return DJIMotorGetMeasure(motor, &measure) ? measure.speed_rad_s : 0.0f;
}

static bool PublishMotorCommand(DJIMotorInstance *motor,
                                Closeloop_Type_e outer_loop,
                                float reference,
                                Motor_Working_Type_e working_state)
{
    DJIMotorCommand command;

    if (!DJIMotorGetCommand(motor, &command)) {
        return false;
    }
    command.settings.outer_loop_type = outer_loop;
    command.reference = reference;
    command.working_state = working_state;
    return DJIMotorPublishCommand(motor, &command);
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (!isfinite(value) || !isfinite(min_value) || !isfinite(max_value) ||
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

bool Chassis_Init(void)
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
                .MaxOut = 15000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .speed_unit = MOTOR_SPEED_RAD_PER_SEC,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    if (!isfinite(CHASSIS_SPIN_SPEED_RATIO) ||
        CHASSIS_SPIN_SPEED_RATIO <= 0.0f ||
        CHASSIS_SPIN_SPEED_RATIO >= 1.0f) {
        MDBG_CHS("invalid spin speed ratio");
        return false;
    }

    motor_fr = DJIMotorInit(&config);
    if (motor_fr != NULL) {
        (void)PublishMotorCommand(motor_fr, CHASSIS_INIT_LOOP, 0.0f,
                                  MOTOR_STOP);
    }

    config.can_init_config.tx_id = CHASSIS_MOTOR_FL_ID;
    motor_fl = DJIMotorInit(&config);
    if (motor_fl != NULL) {
        (void)PublishMotorCommand(motor_fl, CHASSIS_INIT_LOOP, 0.0f,
                                  MOTOR_STOP);
    }

    config.can_init_config.tx_id = CHASSIS_MOTOR_BR_ID;
    motor_br = DJIMotorInit(&config);
    if (motor_br != NULL) {
        (void)PublishMotorCommand(motor_br, CHASSIS_INIT_LOOP, 0.0f,
                                  MOTOR_STOP);
    }

    config.can_init_config.tx_id = CHASSIS_MOTOR_BL_ID;
    motor_bl = DJIMotorInit(&config);
    if (motor_bl != NULL) {
        (void)PublishMotorCommand(motor_bl, CHASSIS_INIT_LOOP, 0.0f,
                                  MOTOR_STOP);
    }

    MDBG_CHS("limits vmax_mm_s=%ld wmax_mrad_s=%ld spin_mrad_s=%ld motor_max_mrad_s=%ld ratio_x1000=%ld",
             (long)(CHASSIS_MAX_TRANSLATION_SPEED * 1000.0f),
             (long)(CHASSIS_MAX_ROTATION_SPEED_RAD_S * 1000.0f),
             (long)(CHASSIS_SPIN_SPEED_RAD_S * 1000.0f),
             (long)(M3508_ROTOR_SPEED_LIMIT_RAD_S * 1000.0f),
             (long)(CHASSIS_MOTOR_REDUCTION_RATIO * 1000.0f));

    return (motor_fr != NULL) && (motor_fl != NULL) &&
           (motor_br != NULL) && (motor_bl != NULL);
}

void Chassis_Update(Input_Data_t *input)
{
    uint32_t now_ms;
    float dt_s = MAIN_LOOP_PERIOD_MS / 1000.0f;
    float yaw_offset_rad;
    float yaw_offset_rate_rad_s;
    float manual_wz;
    float follow_wz = 0.0f;
    float translation_wheel_speed_rad_s[4] = {0.0f};
    float rotation_wheel_speed_rad_s[4] = {0.0f};
    float translation_motor_speed_rad_s[4] = {0.0f};
    float rotation_motor_speed_rad_s[4] = {0.0f};
    float spin_translation_scale = 1.0f;
    float power_scale;
    uint8_t follow_mode;
    uint8_t follow_recovery_entry;
    Chassis_Cmd_t cmd = {0};
    ChassisFollowOutput follow_output = {0};
    ChassisTuningSnapshot tuning_snapshot = {0};

    if (input == NULL || !input->online || input->emergency_stop) {
        MDBG_CHS("stop by input offline/estop");
        Chassis_Stop();
        return;
    }
    if (!MinimalReferee_AllowChassis()) {
        MDBG_CHS("stop by referee lock");
        Chassis_Stop();
        return;
    }

    now_ms = RmTime_NowMs();
    if (chassis_last_tick != 0U) {
        dt_s = ClampFloat((float)(now_ms - chassis_last_tick) / 1000.0f, 0.001f, 0.05f);
    }
    chassis_last_tick = now_ms;

    yaw_offset_rad = Gimbal_GetYawOffsetLogicRad();
    if (!isfinite(yaw_offset_rad)) {
        yaw_offset_rad = 0.0f;
        follow_wz_integral = 0.0f;
    }
    yaw_offset_rate_rad_s = Gimbal_GetYawRelativeSpeedRadS();
    if (!isfinite(yaw_offset_rate_rad_s)) {
        yaw_offset_rate_rad_s = 0.0f;
    }
    if (input->control_mode != last_control_mode) {
        if (last_control_mode == ROBOT_CONTROL_SPIN &&
            input->control_mode != ROBOT_CONTROL_SPIN) {
            follow_recovery_pending = 1U;
        }
        follow_wz_integral = 0.0f;
        filtered_wz = 0.0f;
        last_control_mode = input->control_mode;
    }

    follow_mode = (input->control_mode != ROBOT_CONTROL_SPIN) ? 1U : 0U;
    follow_recovery_entry =
        (follow_mode != 0U && follow_recovery_pending != 0U) ? 1U : 0U;
    cmd.vx_cmd = ClampFloat(input->chassis_x_intent, -1.0f, 1.0f) *
                 CHASSIS_MAX_TRANSLATION_SPEED;
    cmd.vy_cmd = ClampFloat(input->chassis_y_intent, -1.0f, 1.0f) *
                 CHASSIS_MAX_TRANSLATION_SPEED;
    Chassis_LimitTranslation(&cmd.vx_cmd, &cmd.vy_cmd,
                                     CHASSIS_MAX_TRANSLATION_SPEED);
    Chassis_RotateToBody(cmd.vx_cmd, cmd.vy_cmd, yaw_offset_rad,
                                 &cmd.vx, &cmd.vy);
    cmd.yaw_offset_rad = yaw_offset_rad;
    cmd.mode = follow_mode ? CHASSIS_FOLLOW : CHASSIS_NO_FOLLOW;
    cmd.spin_enable = (input->control_mode == ROBOT_CONTROL_SPIN) ? 1U : 0U;
    cmd.control_mode = CTRL_ENABLE;
    cmd.ref_type = REF_SPEED;

    if (fabsf(cmd.vx) < CHASSIS_DEADZONE_VX) {
        cmd.vx = 0.0f;
    }
    if (fabsf(cmd.vy) < CHASSIS_DEADZONE_VY) {
        cmd.vy = 0.0f;
    }

    manual_wz = ClampFloat(input->chassis_rotate_intent, -1.0f, 1.0f) *
                CHASSIS_MAX_ROTATION_SPEED_RAD_S;
    if (follow_mode != 0U) {
        if (fabsf(CHASSIS_FOLLOW_WZ_KI) > 1e-6f) {
            float integral_limit =
                CHASSIS_FOLLOW_WZ_I_MAX_RAD_S /
                fabsf(CHASSIS_FOLLOW_WZ_KI);
            follow_wz_integral += yaw_offset_rad * dt_s;
            follow_wz_integral = ClampFloat(follow_wz_integral, -integral_limit, integral_limit);
        } else {
            follow_wz_integral = 0.0f;
        }

        if (Chassis_CalculateFollowOutput(
                yaw_offset_rad,
                yaw_offset_rate_rad_s,
                follow_wz_integral,
                CHASSIS_FOLLOW_WZ_KP,
                CHASSIS_FOLLOW_WZ_KI,
                CHASSIS_FOLLOW_WZ_KD,
                CHASSIS_FOLLOW_WZ_MAX,
                &follow_output)) {
            follow_wz = follow_output.limited_wz_rad_s;
        }
        cmd.wz = follow_wz + manual_wz;
    } else {
        follow_wz_integral = 0.0f;
        cmd.wz = manual_wz;
        if (input->control_mode == ROBOT_CONTROL_SPIN) {
            cmd.wz += CHASSIS_SPIN_SPEED_RAD_S;
        }
    }

    if (fabsf(cmd.wz) < CHASSIS_DEADZONE_WZ) {
        cmd.wz = 0.0f;
    }
    cmd.wz = ClampFloat(cmd.wz, -CHASSIS_MAX_ROTATION_SPEED_RAD_S,
                        CHASSIS_MAX_ROTATION_SPEED_RAD_S);

    g_robot.chassis = cmd;
    last_wz = cmd.wz;

    power_scale = MinimalReferee_ChassisScale();
    last_power_scale = power_scale;

    if (fabsf(cmd.vx) < CHASSIS_DEADZONE_VX && fabsf(cmd.vy) < CHASSIS_DEADZONE_VY &&
        fabsf(cmd.wz) < CHASSIS_DEADZONE_WZ) {
        filtered_vx = 0.0f;
        filtered_vy = 0.0f;
        filtered_wz = 0.0f;
    } else {
        filtered_vx = filtered_vx * CHASSIS_SPEED_FILTER_COEF + cmd.vx * (1.0f - CHASSIS_SPEED_FILTER_COEF);
        filtered_vy = filtered_vy * CHASSIS_SPEED_FILTER_COEF + cmd.vy * (1.0f - CHASSIS_SPEED_FILTER_COEF);
        if (follow_recovery_entry != 0U) {
            /* 标定正姿态恢复首拍直接生效；平移仍保留原有滤波。 */
            filtered_wz = cmd.wz;
        } else {
            filtered_wz = filtered_wz * CHASSIS_SPEED_FILTER_COEF +
                          cmd.wz * (1.0f - CHASSIS_SPEED_FILTER_COEF);
        }
    }
    if (follow_recovery_entry != 0U) {
        follow_recovery_pending = 0U;
    }

    Chassis_OmniInverse(filtered_vx, filtered_vy, 0.0f,
                                CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_RADIUS,
                                translation_wheel_speed_rad_s);
    Chassis_OmniInverse(0.0f, 0.0f, filtered_wz,
                                CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_RADIUS,
                                rotation_wheel_speed_rad_s);

    for (uint8_t i = 0; i < 4U; i++) {
        translation_motor_speed_rad_s[i] =
            Chassis_WheelToMotorSpeedRadS(
                translation_wheel_speed_rad_s[i],
                CHASSIS_MOTOR_REDUCTION_RATIO) * power_scale;
        rotation_motor_speed_rad_s[i] =
            Chassis_WheelToMotorSpeedRadS(
                rotation_wheel_speed_rad_s[i],
                CHASSIS_MOTOR_REDUCTION_RATIO) * power_scale;
    }

    if (cmd.spin_enable != 0U) {
        (void)Chassis_CombineWheelSpeedsPreserveRotation(
            translation_motor_speed_rad_s,
            rotation_motor_speed_rad_s,
            M3508_ROTOR_SPEED_LIMIT_RAD_S,
            last_wheel_ref,
            &spin_translation_scale);
    } else {
        for (uint8_t i = 0U; i < 4U; ++i) {
            last_wheel_ref[i] = translation_motor_speed_rad_s[i] +
                                rotation_motor_speed_rad_s[i];
        }
        Chassis_NormalizeWheelSpeeds(
            last_wheel_ref, M3508_ROTOR_SPEED_LIMIT_RAD_S);
    }

    for (uint8_t i = 0U; i < 4U; ++i) {
        float speed_deadzone =
            follow_mode ? CHASSIS_FOLLOW_MOTOR_SPEED_DEADZONE_RAD_S
                        : CHASSIS_MOTOR_SPEED_DEADZONE_RAD_S;
        if (fabsf(last_wheel_ref[i]) < speed_deadzone) {
            last_wheel_ref[i] = 0.0f;
        }
    }

    tuning_snapshot.input_x_intent = input->chassis_x_intent;
    tuning_snapshot.input_y_intent = input->chassis_y_intent;
    tuning_snapshot.yaw_error_rad = yaw_offset_rad;
    tuning_snapshot.yaw_error_rate_rad_s = yaw_offset_rate_rad_s;
    tuning_snapshot.follow_p_rad_s = follow_output.p_rad_s;
    tuning_snapshot.follow_i_rad_s = follow_output.i_rad_s;
    tuning_snapshot.follow_d_rad_s = follow_output.d_rad_s;
    tuning_snapshot.follow_raw_wz_rad_s = follow_output.raw_wz_rad_s;
    tuning_snapshot.follow_limited_wz_rad_s =
        follow_output.limited_wz_rad_s;
    tuning_snapshot.command_vx_m_s = cmd.vx;
    tuning_snapshot.command_vy_m_s = cmd.vy;
    tuning_snapshot.command_wz_rad_s = cmd.wz;
    tuning_snapshot.filtered_vx_m_s = filtered_vx;
    tuning_snapshot.filtered_vy_m_s = filtered_vy;
    tuning_snapshot.filtered_wz_rad_s = filtered_wz;
    tuning_snapshot.spin_translation_scale = spin_translation_scale;
    memcpy(tuning_snapshot.wheel_ref_rad_s,
           last_wheel_ref,
           sizeof(last_wheel_ref));
    {
        RmCriticalState critical_state = RmCritical_Enter();
        last_tuning_snapshot = tuning_snapshot;
        RmCritical_Exit(critical_state);
    }

    if (motor_fr != NULL) {
        (void)PublishMotorCommand(motor_fr, CHASSIS_RUN_LOOP_NORMAL,
                                  last_wheel_ref[0], MOTOR_ENALBED);
    }
    if (motor_fl != NULL) {
        (void)PublishMotorCommand(motor_fl, CHASSIS_RUN_LOOP_NORMAL,
                                  last_wheel_ref[1], MOTOR_ENALBED);
    }
    if (motor_br != NULL) {
        (void)PublishMotorCommand(motor_br, CHASSIS_RUN_LOOP_NORMAL,
                                  last_wheel_ref[2], MOTOR_ENALBED);
    }
    if (motor_bl != NULL) {
        (void)PublishMotorCommand(motor_bl, CHASSIS_RUN_LOOP_NORMAL,
                                  last_wheel_ref[3], MOTOR_ENALBED);
    }

    MDBG_CHS("mode=%u spin=%u off_mrad=%ld cmd(vx=%ld vy=%ld wz=%ld) filt(vx=%ld vy=%ld wz=%ld)",
             (unsigned)cmd.mode,
             (unsigned)cmd.spin_enable,
             (long)(cmd.yaw_offset_rad * 1000.0f),
             (long)(cmd.vx * 1000.0f),
             (long)(cmd.vy * 1000.0f),
             (long)(cmd.wz * 1000.0f),
             (long)(filtered_vx * 1000.0f),
             (long)(filtered_vy * 1000.0f),
             (long)(filtered_wz * 1000.0f));
}

void Chassis_Stop(void)
{
    last_wz = 0.0f;
    last_power_scale = 0.0f;
    filtered_vx = 0.0f;
    filtered_vy = 0.0f;
    filtered_wz = 0.0f;
    follow_wz_integral = 0.0f;
    chassis_last_tick = 0U;
    last_control_mode = ROBOT_CONTROL_FOLLOW;
    follow_recovery_pending = 1U;
    memset(last_wheel_ref, 0, sizeof(last_wheel_ref));
    {
        RmCriticalState critical_state = RmCritical_Enter();
        memset(&last_tuning_snapshot, 0, sizeof(last_tuning_snapshot));
        RmCritical_Exit(critical_state);
    }

    g_robot.chassis = (Chassis_Cmd_t){
        .control_mode = CTRL_ZERO_FORCE,
        .ref_type = REF_SPEED,
    };

    if (motor_fr != NULL) {
        (void)PublishMotorCommand(motor_fr, CHASSIS_RUN_LOOP_NORMAL, 0.0f,
                                  MOTOR_STOP);
    }
    if (motor_fl != NULL) {
        (void)PublishMotorCommand(motor_fl, CHASSIS_RUN_LOOP_NORMAL, 0.0f,
                                  MOTOR_STOP);
    }
    if (motor_br != NULL) {
        (void)PublishMotorCommand(motor_br, CHASSIS_RUN_LOOP_NORMAL, 0.0f,
                                  MOTOR_STOP);
    }
    if (motor_bl != NULL) {
        (void)PublishMotorCommand(motor_bl, CHASSIS_RUN_LOOP_NORMAL, 0.0f,
                                  MOTOR_STOP);
    }
}

bool Chassis_IsHealthy(void)
{
    return DJIMotorIsOnline(motor_fr) && DJIMotorIsOnline(motor_fl) &&
           DJIMotorIsOnline(motor_br) && DJIMotorIsOnline(motor_bl);
}

float Chassis_GetWz(void)
{
    return last_wz;
}

float Chassis_GetFRMotorSpeedRefRadS(void)
{
    return last_wheel_ref[0];
}

float Chassis_GetFRMotorSpeedFdbRadS(void)
{
    return ReadMotorSpeedRadS(motor_fr);
}

float Chassis_GetPowerScale(void)
{
    return last_power_scale;
}

bool Chassis_GetTuningSnapshot(ChassisTuningSnapshot *snapshot)
{
    RmCriticalState critical_state;

    if (snapshot == NULL) {
        return false;
    }

    critical_state = RmCritical_Enter();
    *snapshot = last_tuning_snapshot;
    RmCritical_Exit(critical_state);

    snapshot->wheel_fdb_rad_s[0] = ReadMotorSpeedRadS(motor_fr);
    snapshot->wheel_fdb_rad_s[1] = ReadMotorSpeedRadS(motor_fl);
    snapshot->wheel_fdb_rad_s[2] = ReadMotorSpeedRadS(motor_br);
    snapshot->wheel_fdb_rad_s[3] = ReadMotorSpeedRadS(motor_bl);
    return true;
}
