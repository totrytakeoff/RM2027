#include "et08_input_profile.hpp"

#include <cmath>

#include "et08_control_layout.h"

namespace rm {
namespace robot {
namespace {

uint32_t AxisBit(RemoteControlAxisId axis)
{
    return 1UL << static_cast<uint32_t>(axis);
}

uint32_t SwitchBit(Et08ControlSwitchId control)
{
    return 1UL << static_cast<uint32_t>(control);
}

bool RequiredControlsValid(const RemoteControlState &remote)
{
    const uint32_t required_axes =
        AxisBit(REMOTE_AXIS_LEFT_X) | AxisBit(REMOTE_AXIS_LEFT_Y) |
        AxisBit(REMOTE_AXIS_RIGHT_X) | AxisBit(REMOTE_AXIS_RIGHT_Y);
    const uint32_t required_switches =
        SwitchBit(ET08_CONTROL_SWITCH_SA) |
        SwitchBit(ET08_CONTROL_SWITCH_SB) |
        SwitchBit(ET08_CONTROL_SWITCH_SC) |
        SwitchBit(ET08_CONTROL_SWITCH_SD);

    return remote.type == REMOTE_CONTROL_TYPE_ET08 &&
           remote.link_online != 0U && remote.data_valid != 0U &&
           remote.failsafe == 0U && remote.frame_lost == 0U &&
           (remote.axis_valid_mask & required_axes) == required_axes &&
           (remote.switch_valid_mask & required_switches) ==
               required_switches;
}

float ApplyAxisDeadband(float value,
                        const Et08InputProfileConfig &config)
{
    float threshold = 0.0F;

    if (config.stick_full_scale_raw > 0.0F) {
        threshold = static_cast<float>(config.stick_deadzone_raw) /
                    config.stick_full_scale_raw;
    }
    if (threshold < 0.0F) {
        threshold = 0.0F;
    } else if (threshold >= 1.0F) {
        return 0.0F;
    }

    const float magnitude = std::fabs(value);
    if (magnitude <= threshold) {
        return 0.0F;
    }

    float rescaled = (magnitude - threshold) / (1.0F - threshold);
    if (rescaled > 1.0F) {
        rescaled = 1.0F;
    }
    return value < 0.0F ? -rescaled : rescaled;
}

bool AxisIsNeutral(const RemoteControlState &remote,
                   RemoteControlAxisId axis,
                   int16_t deadzone)
{
    const int32_t raw = remote.axis_raw[static_cast<uint32_t>(axis)];
    const int32_t limit = deadzone >= 0 ? deadzone : -(int32_t)deadzone;
    return raw >= -limit && raw <= limit;
}

} // namespace

bool MapEt08Input(const RemoteControlState &remote,
                  const Et08InputProfileConfig &config,
                  Input_Data_t &output)
{
    output = Input_Data_t{};
    output.emergency_stop = 1U;

    if (!RequiredControlsValid(remote)) {
        return false;
    }

    const RemoteControlSwitchPosition sa =
        remote.switches[ET08_CONTROL_SWITCH_SA];
    const RemoteControlSwitchPosition sb =
        remote.switches[ET08_CONTROL_SWITCH_SB];
    const RemoteControlSwitchPosition sc =
        remote.switches[ET08_CONTROL_SWITCH_SC];
    const RemoteControlSwitchPosition sd =
        remote.switches[ET08_CONTROL_SWITCH_SD];
    const float left_x = ApplyAxisDeadband(
        remote.axis_normalized[REMOTE_AXIS_LEFT_X], config);
    const float left_y = ApplyAxisDeadband(
        remote.axis_normalized[REMOTE_AXIS_LEFT_Y], config);
    const float right_x = ApplyAxisDeadband(
        remote.axis_normalized[REMOTE_AXIS_RIGHT_X], config);
    const float right_y = ApplyAxisDeadband(
        remote.axis_normalized[REMOTE_AXIS_RIGHT_Y], config);

    output.online = 1U;
    output.data_valid = 1U;
    output.emergency_stop = 0U;
    output.operator_enable_request =
        sa == REMOTE_SWITCH_UP ? 1U : 0U;
    output.operator_arm_event =
        ((remote.switch_changed_mask &
          SwitchBit(ET08_CONTROL_SWITCH_SA)) != 0U &&
         sa == REMOTE_SWITCH_UP)
            ? 1U
            : 0U;

    /*
     * 这里只描述 SA 之外的控制是否安全，使安全状态机能在 SA 上拨沿
     * 到来时再次核对当前姿态；SA 是否关闭由 enable_request 单独表达。
     */
    output.operator_safe_position =
        (sb == REMOTE_SWITCH_DOWN && sc == REMOTE_SWITCH_DOWN &&
         sd == REMOTE_SWITCH_DOWN &&
         AxisIsNeutral(remote, REMOTE_AXIS_LEFT_X,
                       config.stick_deadzone_raw) &&
         AxisIsNeutral(remote, REMOTE_AXIS_LEFT_Y,
                       config.stick_deadzone_raw) &&
         AxisIsNeutral(remote, REMOTE_AXIS_RIGHT_X,
                       config.stick_deadzone_raw) &&
         AxisIsNeutral(remote, REMOTE_AXIS_RIGHT_Y,
                       config.stick_deadzone_raw))
            ? 1U
            : 0U;

    if (sb == REMOTE_SWITCH_UP) {
        output.control_mode = ROBOT_CONTROL_SPIN;
    } else if (sb == REMOTE_SWITCH_MIDDLE) {
        output.control_mode = ROBOT_CONTROL_AUTO_AIM;
    } else {
        output.control_mode = ROBOT_CONTROL_FOLLOW;
    }

    if (sc == REMOTE_SWITCH_UP) {
        output.fire_mode = ROBOT_FIRE_CONTINUOUS;
    } else if (sc == REMOTE_SWITCH_MIDDLE) {
        output.fire_mode = ROBOT_FIRE_SINGLE;
    } else {
        output.fire_mode = ROBOT_FIRE_DISABLED;
    }

    output.fire_trigger_down =
        sd == REMOTE_SWITCH_UP ? 1U : 0U;
    output.fire_trigger_pressed =
        ((remote.switch_changed_mask &
          SwitchBit(ET08_CONTROL_SWITCH_SD)) != 0U &&
         sd == REMOTE_SWITCH_UP)
            ? 1U
            : 0U;
    output.fire_trigger_released =
        ((remote.switch_changed_mask &
          SwitchBit(ET08_CONTROL_SWITCH_SD)) != 0U &&
         sd == REMOTE_SWITCH_DOWN)
            ? 1U
            : 0U;

    if (output.operator_enable_request == 0U) {
        output.fire_mode = ROBOT_FIRE_DISABLED;
        output.fire_trigger_down = 0U;
        output.fire_trigger_pressed = 0U;
        output.fire_trigger_released = 0U;
        return true;
    }

    output.chassis_x_intent = -left_x;
    /* 当前 ET08 实测 CH3 前推为负，统一语义仍保持“前进为正”。 */
    output.chassis_y_intent = -left_y;
    output.chassis_rotate_intent = 0.0F;
    output.gimbal_yaw_intent = right_x;
    /* ET08 右摇杆上下方向与 IMU Pitch 语义相反，在适配层统一。 */
    output.gimbal_pitch_intent = -right_y;
    output.yaw_control_active = right_x != 0.0F ? 1U : 0U;
    output.pitch_control_active = output.gimbal_pitch_intent != 0.0F ? 1U : 0U;
    return true;
}

} // namespace robot
} // namespace rm
