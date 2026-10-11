#include "chassis_follow.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define TWO_PI_RAD 6.28318530717958647692f

static float ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

bool Chassis_CalculateFollowOutput(
    float yaw_error_rad,
    float yaw_error_rate_rad_s,
    float yaw_error_integral_rad_seconds,
    float kp,
    float ki,
    float kd,
    float max_abs_wz_rad_s,
    ChassisFollowOutput *output)
{
    float nearest_yaw_error_rad;

    if (output == NULL) {
        return false;
    }
    memset(output, 0, sizeof(*output));

    if (!isfinite(yaw_error_rad) ||
        !isfinite(yaw_error_rate_rad_s) ||
        !isfinite(yaw_error_integral_rad_seconds) ||
        !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        !isfinite(max_abs_wz_rad_s) || max_abs_wz_rad_s <= 0.0f) {
        return false;
    }

    /*
     * 跟随恢复始终走到标定零位的最近等价角，避免调用方给出累计角时
     * 底盘多转整圈。remainderf 的结果位于 [-pi, pi]。
     */
    nearest_yaw_error_rad = remainderf(yaw_error_rad, TWO_PI_RAD);
    output->p_rad_s = kp * nearest_yaw_error_rad;
    output->i_rad_s = ki * yaw_error_integral_rad_seconds;
    output->d_rad_s = kd * yaw_error_rate_rad_s;
    output->raw_wz_rad_s = output->p_rad_s + output->i_rad_s +
                           output->d_rad_s;

    if (!isfinite(output->raw_wz_rad_s)) {
        memset(output, 0, sizeof(*output));
        return false;
    }

    output->limited_wz_rad_s =
        ClampFloat(output->raw_wz_rad_s,
                   -max_abs_wz_rad_s,
                   max_abs_wz_rad_s);
    return true;
}
