#ifndef ROBOT_GIMBAL_LIMITS_H
#define ROBOT_GIMBAL_LIMITS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float min_angle_deg;
    float max_angle_deg;
} GimbalPitchLimitConfig;

bool GimbalPitchLimit_IsConfigValid(const GimbalPitchLimitConfig *config);

/** 将 IMU Pitch 角度保持/自动控制目标限制在机械边界内。 */
float GimbalPitchLimit_ClampAngleReference(
    float requested_imu_angle_deg,
    bool imu_angle_valid,
    float current_imu_pitch_deg,
    const GimbalPitchLimitConfig *config);

/** 按无量纲速度意图和实际时间推进 Pitch 角度目标，并限制在机械边界内。 */
float GimbalPitchLimit_AdvanceAngleReference(
    float current_reference_deg,
    float normalized_rate_intent,
    float max_rate_deg_s,
    float elapsed_s,
    const GimbalPitchLimitConfig *config);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_GIMBAL_LIMITS_H */
