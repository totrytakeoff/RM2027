#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "robot_config.h"

#define PI_F 3.14159265358979323846f

static void AssertNear(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

int main(void)
{
    const float friction_target_rpm =
        fabsf(FRICTION_TARGET_SPEED_RAD_S) * 60.0f / (2.0f * PI_F);
    const float friction_kp_per_rpm =
        FRICTION_SPEED_KP * (2.0f * PI_F / 60.0f);
    const float loader_kp_per_rpm = LOADER_SPEED_KP * 6.0f;

    AssertNear(friction_target_rpm, 5000.0f, 0.01f);
    AssertNear(friction_kp_per_rpm, 5.0f, 0.001f);
    AssertNear(LOADER_ANGLE_STEP, 1620.0f, 0.001f);
    AssertNear(LOADER_CONTINUOUS_SPEED_DEG_S, 8100.0f, 0.001f);
    AssertNear(loader_kp_per_rpm, 10.0f, 0.001f);

    assert(FRICTION_READY_ERROR_RATIO <
           FRICTION_READY_DROP_ERROR_RATIO);
    assert(LOADER_SPEED_MAX_OUT <= 10000.0f);
    assert(LOADER_JAM_OUTPUT_RATIO > 0.0f &&
           LOADER_JAM_OUTPUT_RATIO <= 1.0f);

    puts("rm_robot shoot configuration tests passed");
    return 0;
}
