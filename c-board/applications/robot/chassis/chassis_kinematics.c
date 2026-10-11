#include "chassis_kinematics.h"

#include <math.h>
#include <stddef.h>

void Chassis_RotateToBody(float gimbal_vx,
                                  float gimbal_vy,
                                  float yaw_offset_rad,
                                  float *body_vx,
                                  float *body_vy)
{
    float cosine;
    float sine;
    float theta;
    float rotated_vx;
    float rotated_vy;

    if (body_vx == NULL || body_vy == NULL) {
        return;
    }
    if (!isfinite(gimbal_vx) || !isfinite(gimbal_vy) ||
        !isfinite(yaw_offset_rad)) {
        *body_vx = 0.0f;
        *body_vy = 0.0f;
        return;
    }

    /*
     * yaw_offset = yaw_gimbal - yaw_chassis。输入向量位于云台坐标系，
     * 因此旋转到车体坐标系时应直接使用该相对角，而不是取反。
     */
    theta = yaw_offset_rad;
    cosine = cosf(theta);
    sine = sinf(theta);
    rotated_vx = gimbal_vx * cosine + gimbal_vy * sine;
    rotated_vy = -gimbal_vx * sine + gimbal_vy * cosine;
    *body_vx = rotated_vx;
    *body_vy = rotated_vy;
}

void Chassis_LimitTranslation(float *vx,
                                      float *vy,
                                      float max_speed)
{
    float magnitude;
    float scale;

    if (vx == NULL || vy == NULL) {
        return;
    }
    if (!isfinite(*vx) || !isfinite(*vy) || !isfinite(max_speed) ||
        max_speed <= 0.0f) {
        *vx = 0.0f;
        *vy = 0.0f;
        return;
    }

    magnitude = sqrtf((*vx * *vx) + (*vy * *vy));
    if (!isfinite(magnitude)) {
        *vx = 0.0f;
        *vy = 0.0f;
        return;
    }
    if (magnitude > max_speed) {
        scale = max_speed / magnitude;
        *vx *= scale;
        *vy *= scale;
    }
}

void Chassis_OmniInverse(float vx,
                                 float vy,
                                 float wz,
                                 float wheel_base,
                                 float wheel_radius,
                                 float output[4])
{
    float half_base;

    if (output == NULL) {
        return;
    }
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz) ||
        !isfinite(wheel_base) || !isfinite(wheel_radius) ||
        wheel_base <= 0.0f || wheel_radius <= 0.0f) {
        output[0] = 0.0f;
        output[1] = 0.0f;
        output[2] = 0.0f;
        output[3] = 0.0f;
        return;
    }

    half_base = wheel_base * 0.5f;
    output[0] = (vx - vy - half_base * wz) / wheel_radius;
    output[1] = (vx + vy - half_base * wz) / wheel_radius;
    output[2] = (-vx - vy - half_base * wz) / wheel_radius;
    output[3] = (-vx + vy - half_base * wz) / wheel_radius;
}

float Chassis_WheelToMotorSpeedRadS(float wheel_speed_rad_s,
                                            float reduction_ratio)
{
    if (!isfinite(wheel_speed_rad_s) || !isfinite(reduction_ratio) ||
        reduction_ratio <= 0.0f) {
        return 0.0f;
    }
    return wheel_speed_rad_s * reduction_ratio;
}

void Chassis_NormalizeWheelSpeeds(float speeds[4],
                                          float max_abs_speed)
{
    float maximum = 0.0f;
    float scale;
    unsigned index;

    if (speeds == NULL) {
        return;
    }
    if (!isfinite(max_abs_speed) || max_abs_speed <= 0.0f) {
        for (index = 0U; index < 4U; ++index) {
            speeds[index] = 0.0f;
        }
        return;
    }

    for (index = 0U; index < 4U; ++index) {
        if (!isfinite(speeds[index])) {
            unsigned clear_index;
            for (clear_index = 0U; clear_index < 4U; ++clear_index) {
                speeds[clear_index] = 0.0f;
            }
            return;
        }
        const float magnitude = fabsf(speeds[index]);
        if (magnitude > maximum) {
            maximum = magnitude;
        }
    }
    if (maximum <= max_abs_speed) {
        return;
    }

    scale = max_abs_speed / maximum;
    for (index = 0U; index < 4U; ++index) {
        speeds[index] *= scale;
    }
}

bool Chassis_CombineWheelSpeedsPreserveRotation(
    const float translation[4],
    const float rotation[4],
    float max_abs_speed,
    float output[4],
    float *translation_scale)
{
    float scale = 1.0f;
    unsigned index;

    if (translation_scale != NULL) {
        *translation_scale = 0.0f;
    }
    if (translation == NULL || rotation == NULL || output == NULL ||
        !isfinite(max_abs_speed) || max_abs_speed <= 0.0f) {
        if (output != NULL) {
            for (index = 0U; index < 4U; ++index) {
                output[index] = 0.0f;
            }
        }
        return false;
    }

    for (index = 0U; index < 4U; ++index) {
        float candidate;

        if (!isfinite(translation[index]) || !isfinite(rotation[index])) {
            for (index = 0U; index < 4U; ++index) {
                output[index] = 0.0f;
            }
            return false;
        }
        if (fabsf(rotation[index]) > max_abs_speed) {
            for (index = 0U; index < 4U; ++index) {
                output[index] = rotation[index];
            }
            Chassis_NormalizeWheelSpeeds(output, max_abs_speed);
            return true;
        }
        if (translation[index] > 0.0f) {
            candidate = (max_abs_speed - rotation[index]) /
                        translation[index];
            if (candidate < scale) {
                scale = candidate;
            }
        } else if (translation[index] < 0.0f) {
            candidate = (-max_abs_speed - rotation[index]) /
                        translation[index];
            if (candidate < scale) {
                scale = candidate;
            }
        }
    }

    if (scale < 0.0f) {
        scale = 0.0f;
    } else if (scale > 1.0f) {
        scale = 1.0f;
    }
    for (index = 0U; index < 4U; ++index) {
        output[index] = rotation[index] + scale * translation[index];
        if (output[index] > max_abs_speed) {
            output[index] = max_abs_speed;
        } else if (output[index] < -max_abs_speed) {
            output[index] = -max_abs_speed;
        }
    }
    if (translation_scale != NULL) {
        *translation_scale = scale;
    }
    return true;
}
