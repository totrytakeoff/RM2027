/**
 * @file chassis.h
 * @brief 底盘控制模块
 */

#ifndef ROBOT_CHASSIS_H
#define ROBOT_CHASSIS_H

#include <stdbool.h>

#include "robot_types.h"

typedef struct {
    float input_x_intent;
    float input_y_intent;
    float yaw_error_rad;
    float yaw_error_rate_rad_s;
    float follow_p_rad_s;
    float follow_i_rad_s;
    float follow_d_rad_s;
    float follow_raw_wz_rad_s;
    float follow_limited_wz_rad_s;
    float command_vx_m_s;
    float command_vy_m_s;
    float command_wz_rad_s;
    float filtered_vx_m_s;
    float filtered_vy_m_s;
    float filtered_wz_rad_s;
    float spin_translation_scale;
    float wheel_ref_rad_s[4];
    float wheel_fdb_rad_s[4];
} ChassisTuningSnapshot;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 底盘初始化
 */
bool Chassis_Init(void);

/**
 * @brief 底盘更新
 * @param input 输入数据
 */
void Chassis_Update(Input_Data_t *input);

/**
 * @brief 底盘停止
 */
void Chassis_Stop(void);

/** True when all required chassis motor feedback is inside its deadline. */
bool Chassis_IsHealthy(void);

/**
 * @brief 获取底盘旋转角速度(供云台使用)
 * @return 旋转角速度 rad/s
 */
float Chassis_GetWz(void);
/** 前右M3508转子速度参考与反馈，单位rad/s。 */
float Chassis_GetFRMotorSpeedRefRadS(void);
float Chassis_GetFRMotorSpeedFdbRadS(void);
float Chassis_GetPowerScale(void);

/** Copy one coherent controller snapshot for low-priority tuning telemetry. */
bool Chassis_GetTuningSnapshot(ChassisTuningSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CHASSIS_H */
