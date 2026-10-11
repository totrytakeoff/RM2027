#include "gimbal_limits.h"

#include <math.h>
#include <stddef.h>

#include "gimbal_units.h"

bool GimbalPitchLimit_IsConfigValid(const GimbalPitchLimitConfig *config)
{
    if (config == NULL || !Gimbal_FloatIsFinite(config->min_angle_deg) ||
        !Gimbal_FloatIsFinite(config->max_angle_deg) ||
        config->max_angle_deg <= config->min_angle_deg) {
        return false;
    }
    return true;
}

float GimbalPitchLimit_ClampAngleReference(
    float requested_imu_angle_deg,
    bool imu_angle_valid,
    float current_imu_pitch_deg,
    const GimbalPitchLimitConfig *config)
{
    if (!Gimbal_FloatIsFinite(current_imu_pitch_deg)) {
        return 0.0f;
    }
    if (!Gimbal_FloatIsFinite(requested_imu_angle_deg) || !imu_angle_valid ||
        !GimbalPitchLimit_IsConfigValid(config)) {
        return current_imu_pitch_deg;
    }
    if (requested_imu_angle_deg > config->max_angle_deg) {
        return config->max_angle_deg;
    }
    if (requested_imu_angle_deg < config->min_angle_deg) {
        return config->min_angle_deg;
    }
    return requested_imu_angle_deg;
}

float GimbalPitchLimit_AdvanceAngleReference(
    float current_reference_deg,
    float normalized_rate_intent,
    float max_rate_deg_s,
    float elapsed_s,
    const GimbalPitchLimitConfig *config)
{
    float intent;
    float next_reference;

    if (!Gimbal_FloatIsFinite(current_reference_deg) ||
        !GimbalPitchLimit_IsConfigValid(config)) {
        return 0.0f;
    }

    current_reference_deg = GimbalPitchLimit_ClampAngleReference(
        current_reference_deg, true, current_reference_deg, config);
    if (!Gimbal_FloatIsFinite(normalized_rate_intent) ||
        !Gimbal_FloatIsFinite(max_rate_deg_s) || max_rate_deg_s < 0.0f ||
        !Gimbal_FloatIsFinite(elapsed_s) || elapsed_s < 0.0f) {
        return current_reference_deg;
    }

    intent = normalized_rate_intent;
    if (intent > 1.0f) {
        intent = 1.0f;
    } else if (intent < -1.0f) {
        intent = -1.0f;
    }
    next_reference = current_reference_deg +
                     intent * max_rate_deg_s * elapsed_s;
    return GimbalPitchLimit_ClampAngleReference(
        next_reference, true, current_reference_deg, config);
}
