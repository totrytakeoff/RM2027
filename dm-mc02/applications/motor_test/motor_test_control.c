#include "motor_test_control.h"
#include "motor_test_config.h"
#include "chassis_config.h"
#include <math.h>
MotorTestMode RmMotorTest_Mode(const RemoteControlState *remote)
{
    if (!remote || !remote->data_valid || !(remote->switch_valid_mask & 8U)) return MOTOR_TEST_OFF;
    if (remote->switches[3] == REMOTE_SWITCH_DOWN) return MOTOR_TEST_SPEED;
    if (remote->switches[3] == REMOTE_SWITCH_UP) return MOTOR_TEST_POSITION;
    return MOTOR_TEST_OFF;
}
float RmMotorTest_Reference(const RemoteControlState *remote, MotorTestMode mode)
{
    if (!remote || (mode != MOTOR_TEST_SPEED && mode != MOTOR_TEST_POSITION)) return 0.0f;
    unsigned axis = mode == MOTOR_TEST_SPEED ? REMOTE_AXIS_LEFT_Y : REMOTE_AXIS_RIGHT_Y;
    if (!(remote->axis_valid_mask & (1U << axis))) return 0.0f;
    float value = remote->axis_normalized[axis];
    float deadband = CHASSIS_DEMO_DEADBAND_RAW / CHASSIS_DEMO_REMOTE_SPAN;
    if (!isfinite(value) || fabsf(value) <= deadband) return 0.0f;
    value = copysignf(fminf(1.0f, (fabsf(value) - deadband) / (1.0f - deadband)), value);
    return value * MOTOR_TEST_DIRECTION * (mode == MOTOR_TEST_SPEED ? MOTOR_TEST_MAX_RAD_S :
        MOTOR_TEST_POSITION_RANGE_RAD * 57.2957795131f * MOTOR_TEST_REDUCTION);
}
