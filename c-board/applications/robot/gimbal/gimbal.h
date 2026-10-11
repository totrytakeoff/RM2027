/**
 * @file gimbal.h
 * @brief 云台执行控制模块
 */

#ifndef ROBOT_GIMBAL_H
#define ROBOT_GIMBAL_H

#include <stdbool.h>

#include "robot_types.h"

typedef struct {
    AxisCtrlMode_e control_mode;
    uint16_t encoder_ecd;
    int16_t motor_current_feedback;
    float encoder_single_deg;
    float encoder_total_deg;
    float encoder_speed_deg_s;
    float operator_speed_command_deg_s;
    float hold_target_deg;
    float imu_angle_deg;
    float imu_gyro_rad_s;
    float imu_gyro_deg_s;
    float current_feedforward;
    float angle_reference_deg;
    float angle_feedback_deg;
    float angle_error_deg;
    float angle_p_deg_s;
    float angle_i_deg_s;
    float angle_d_deg_s;
    float angle_output_deg_s;
    float angle_output_limit_ratio;
    float speed_reference_deg_s;
    float speed_feedback_deg_s;
    float speed_error_deg_s;
    float speed_p_current;
    float speed_i_current;
    float speed_d_current;
    float speed_output_current;
    float speed_output_limit_ratio;
} GimbalAxisTuningSnapshot;

typedef struct {
    GimbalAxisTuningSnapshot yaw;
    GimbalAxisTuningSnapshot pitch;
    float yaw_offset_raw_deg;
    float yaw_offset_logic_deg;
    float yaw_relative_speed_rad_s;
    float yaw_base_rate_estimate_rad_s;
} GimbalTuningSnapshot;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 云台初始化
 */
bool Gimbal_Init(void);

/** Refresh high-rate IMU angle/rate feedback before motor PID execution. */
void Gimbal_MotorStep(void);

/**
 * @brief 云台更新
 * @param input 输入数据
 */
void Gimbal_Update(Input_Data_t *input);

/**
 * @brief 云台停止
 */
void Gimbal_Stop(void);
/** True only when both yaw and pitch motor feedback links are online. */
bool Gimbal_AreMotorsHealthy(void);
bool Gimbal_IsHealthy(void);

/**
 * @brief 获取当前云台模式
 * @return 云台模式
 */
RobotControlMode_e Gimbal_GetMode(void);
float Gimbal_GetYawSpeedRef(void);
float Gimbal_GetYawSpeedFdb(void);
/** Pitch operator-requested angle-target rate, not the inner-loop reference. */
float Gimbal_GetPitchSpeedRef(void);
float Gimbal_GetPitchSpeedFdb(void);
float Gimbal_GetPitchTargetAngle(void);
float Gimbal_GetYawTargetAngle(void);
float Gimbal_GetYawEncoderAngle(void);
float Gimbal_GetPitchEncoderAngle(void);
float Gimbal_GetYawIMUAngle(void);
float Gimbal_GetPitchIMUAngle(void);
float Gimbal_GetPitchGravityFeedforward(void);
float Gimbal_GetYawOffsetRawDeg(void);
float Gimbal_GetYawOffsetLogicDeg(void);
float Gimbal_GetYawOffsetLogicRad(void);
float Gimbal_GetYawRelativeSpeedRadS(void);
float Gimbal_GetYawLogicAngle(void);
AxisCtrlMode_e Gimbal_GetPitchCtrlMode(void);
AxisCtrlMode_e Gimbal_GetYawCtrlMode(void);

/** Copy task-safe diagnostic views of both GM6020 control chains. */
bool Gimbal_GetTuningSnapshot(GimbalTuningSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_GIMBAL_H */
