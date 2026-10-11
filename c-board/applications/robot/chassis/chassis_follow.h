#ifndef ROBOT_CHASSIS_FOLLOW_H
#define ROBOT_CHASSIS_FOLLOW_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float p_rad_s;
    float i_rad_s;
    float d_rad_s;
    float raw_wz_rad_s;
    float limited_wz_rad_s;
} ChassisFollowOutput;

/**
 * @brief 由云台相对底盘角度生成底盘跟随角速度。
 *
 * yaw_error_rad 会在本层归一化到 [-pi, pi]，保证总是沿最近方向恢复
 * 到标定零位。符号约定保持直观：正误差必须产生正角速度；误差正在
 * 减小时，正 Kd 与负误差变化率共同产生反向阻尼。
 */
bool Chassis_CalculateFollowOutput(
    float yaw_error_rad,
    float yaw_error_rate_rad_s,
    float yaw_error_integral_rad_seconds,
    float kp,
    float ki,
    float kd,
    float max_abs_wz_rad_s,
    ChassisFollowOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CHASSIS_FOLLOW_H */
