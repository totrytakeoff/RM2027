#include <cmath>
#include <cstdint>
#include <cstdio>

#include "et08_control_layout.h"
#include "et08_input_profile.hpp"
#include "safety_manager.h"

static unsigned failures;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,      \
                         #condition);                                          \
            failures++;                                                        \
        }                                                                      \
    } while (false)

static const rm::robot::Et08InputProfileConfig kConfig = {
    50,
    660.0F,
};

static uint32_t AxisBit(RemoteControlAxisId axis)
{
    return 1UL << static_cast<uint32_t>(axis);
}

static uint32_t SwitchBit(Et08ControlSwitchId control)
{
    return 1UL << static_cast<uint32_t>(control);
}

static void SetAxis(RemoteControlState &state,
                    RemoteControlAxisId axis,
                    int16_t raw)
{
    const uint32_t index = static_cast<uint32_t>(axis);
    state.axis_raw[index] = raw;
    state.axis_normalized[index] = static_cast<float>(raw) / 660.0F;
    state.axis_valid_mask |= AxisBit(axis);
}

static RemoteControlState SafeRemote(void)
{
    RemoteControlState state{};
    state.type = REMOTE_CONTROL_TYPE_ET08;
    state.link_online = 1U;
    state.data_valid = 1U;
    state.switch_valid_mask =
        SwitchBit(ET08_CONTROL_SWITCH_SA) |
        SwitchBit(ET08_CONTROL_SWITCH_SB) |
        SwitchBit(ET08_CONTROL_SWITCH_SC) |
        SwitchBit(ET08_CONTROL_SWITCH_SD);
    state.switches[ET08_CONTROL_SWITCH_SA] = REMOTE_SWITCH_DOWN;
    state.switches[ET08_CONTROL_SWITCH_SB] = REMOTE_SWITCH_DOWN;
    state.switches[ET08_CONTROL_SWITCH_SC] = REMOTE_SWITCH_DOWN;
    state.switches[ET08_CONTROL_SWITCH_SD] = REMOTE_SWITCH_DOWN;
    SetAxis(state, REMOTE_AXIS_LEFT_X, 0);
    SetAxis(state, REMOTE_AXIS_LEFT_Y, 0);
    SetAxis(state, REMOTE_AXIS_RIGHT_X, 0);
    SetAxis(state, REMOTE_AXIS_RIGHT_Y, 0);
    return state;
}

static bool Near(float lhs, float rhs)
{
    return std::fabs(lhs - rhs) <= 0.0001F;
}

static void TestCombinedSwitchLayout(void)
{
    const RemoteControlSwitchPosition lower_expected[3] = {
        REMOTE_SWITCH_UP,
        REMOTE_SWITCH_MIDDLE,
        REMOTE_SWITCH_DOWN,
    };

    for (uint8_t state = 0U; state < 6U; ++state) {
        CHECK(ET08_ControlDecodeUpperSwitch(state) ==
              (state <= 2U ? REMOTE_SWITCH_UP : REMOTE_SWITCH_DOWN));
        CHECK(ET08_ControlDecodeLowerSwitch(state) ==
              lower_expected[state % 3U]);
    }
    CHECK(ET08_ControlDecodeUpperSwitch(6U) == REMOTE_SWITCH_INVALID);
    CHECK(ET08_ControlDecodeLowerSwitch(0xFFU) == REMOTE_SWITCH_INVALID);
}

static void TestInvalidSnapshotFailsSafe(void)
{
    RemoteControlState state = SafeRemote();
    Input_Data_t output{};

    state.data_valid = 0U;
    CHECK(!rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.online == 0U);
    CHECK(output.data_valid == 0U);
    CHECK(output.emergency_stop == 1U);
    CHECK(Near(output.chassis_x_intent, 0.0F));

    state = SafeRemote();
    state.axis_valid_mask &= ~AxisBit(REMOTE_AXIS_RIGHT_Y);
    CHECK(!rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.emergency_stop == 1U);
}

static void TestSafeGateAndArmEdge(void)
{
    RemoteControlState state = SafeRemote();
    Input_Data_t output{};

    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.online == 1U);
    CHECK(output.emergency_stop == 0U);
    CHECK(output.operator_enable_request == 0U);
    CHECK(output.operator_safe_position == 1U);
    CHECK(output.operator_arm_event == 0U);
    CHECK(output.fire_mode == ROBOT_FIRE_DISABLED);

    state.switches[ET08_CONTROL_SWITCH_SB] = REMOTE_SWITCH_UP;
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.operator_safe_position == 0U);

    state = SafeRemote();
    state.switches[ET08_CONTROL_SWITCH_SA] = REMOTE_SWITCH_UP;
    state.switch_changed_mask = SwitchBit(ET08_CONTROL_SWITCH_SA);
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.operator_enable_request == 1U);
    CHECK(output.operator_arm_event == 1U);
    CHECK(output.operator_safe_position == 1U);
}

static RmSafetyInputs SafetyInputsFrom(const Input_Data_t &input)
{
    const RmSafetyInputs safety = {
        true,
        input.online != 0U,
        input.emergency_stop != 0U,
        input.operator_enable_request != 0U,
        input.operator_safe_position != 0U,
        input.operator_arm_event != 0U,
        true,
        true,
    };
    return safety;
}

static void TestUnsafeControlsCannotReuseSafeSnapshot(void)
{
    RemoteControlState state = SafeRemote();
    Input_Data_t output{};
    RmSafetyManager manager;

    RmSafety_Init(&manager);
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    RmSafetyInputs safety = SafetyInputsFrom(output);
    CHECK(RmSafety_Update(&manager, &safety));
    CHECK(manager.safe_position_seen);

    /* SA 仍在下位时进入连发并扣住扳机，必须撤销解锁资格。 */
    state.switches[ET08_CONTROL_SWITCH_SC] = REMOTE_SWITCH_UP;
    state.switches[ET08_CONTROL_SWITCH_SD] = REMOTE_SWITCH_UP;
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.operator_safe_position == 0U);
    safety = SafetyInputsFrom(output);
    (void)RmSafety_Update(&manager, &safety);
    CHECK(!manager.safe_position_seen);

    state.switches[ET08_CONTROL_SWITCH_SA] = REMOTE_SWITCH_UP;
    state.switch_changed_mask = SwitchBit(ET08_CONTROL_SWITCH_SA);
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.operator_arm_event == 1U);
    CHECK(output.fire_mode == ROBOT_FIRE_CONTINUOUS);
    CHECK(output.fire_trigger_down == 1U);
    safety = SafetyInputsFrom(output);
    CHECK(RmSafety_Update(&manager, &safety));
    CHECK(manager.state == RM_SAFETY_STATE_DISARMED);
    CHECK(!RmSafety_OutputPermitted(&manager));

    /* 回到完整安全姿态并重新上拨，正常路径仍应成功解锁。 */
    state = SafeRemote();
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    safety = SafetyInputsFrom(output);
    CHECK(RmSafety_Update(&manager, &safety));
    CHECK(manager.safe_position_seen);

    state.switches[ET08_CONTROL_SWITCH_SA] = REMOTE_SWITCH_UP;
    state.switch_changed_mask = SwitchBit(ET08_CONTROL_SWITCH_SA);
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.operator_safe_position == 1U);
    safety = SafetyInputsFrom(output);
    CHECK(RmSafety_Update(&manager, &safety));
    CHECK(manager.state == RM_SAFETY_STATE_ACTIVE);
    CHECK(RmSafety_OutputPermitted(&manager));
}

static void TestModeAndMotionMapping(void)
{
    RemoteControlState state = SafeRemote();
    Input_Data_t output{};

    state.switches[ET08_CONTROL_SWITCH_SA] = REMOTE_SWITCH_UP;
    SetAxis(state, REMOTE_AXIS_LEFT_X, 660);
    SetAxis(state, REMOTE_AXIS_LEFT_Y, -660);
    SetAxis(state, REMOTE_AXIS_RIGHT_X, 355);
    SetAxis(state, REMOTE_AXIS_RIGHT_Y, 355);

    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.control_mode == ROBOT_CONTROL_FOLLOW);
    CHECK(Near(output.chassis_x_intent, -1.0F));
    /* 当前 ET08 实测左摇杆前推时 CH3 为负，统一意图必须为正。 */
    CHECK(Near(output.chassis_y_intent, 1.0F));
    CHECK(Near(output.chassis_rotate_intent, 0.0F));
    CHECK(Near(output.gimbal_yaw_intent, 0.5F));
    CHECK(Near(output.gimbal_pitch_intent, -0.5F));
    CHECK(output.yaw_control_active == 1U);
    CHECK(output.pitch_control_active == 1U);

    state.switches[ET08_CONTROL_SWITCH_SB] = REMOTE_SWITCH_MIDDLE;
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.control_mode == ROBOT_CONTROL_AUTO_AIM);

    state.switches[ET08_CONTROL_SWITCH_SB] = REMOTE_SWITCH_UP;
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.control_mode == ROBOT_CONTROL_SPIN);
}

static void TestFireModeAndTriggerEdges(void)
{
    RemoteControlState state = SafeRemote();
    Input_Data_t output{};

    state.switches[ET08_CONTROL_SWITCH_SA] = REMOTE_SWITCH_UP;
    state.switches[ET08_CONTROL_SWITCH_SC] = REMOTE_SWITCH_MIDDLE;
    state.switches[ET08_CONTROL_SWITCH_SD] = REMOTE_SWITCH_UP;
    state.switch_changed_mask = SwitchBit(ET08_CONTROL_SWITCH_SD);
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.fire_mode == ROBOT_FIRE_SINGLE);
    CHECK(output.fire_trigger_down == 1U);
    CHECK(output.fire_trigger_pressed == 1U);
    CHECK(output.fire_trigger_released == 0U);

    state.switch_changed_mask = 0U;
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.fire_trigger_down == 1U);
    CHECK(output.fire_trigger_pressed == 0U);

    state.switches[ET08_CONTROL_SWITCH_SD] = REMOTE_SWITCH_DOWN;
    state.switch_changed_mask = SwitchBit(ET08_CONTROL_SWITCH_SD);
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.fire_trigger_down == 0U);
    CHECK(output.fire_trigger_released == 1U);

    state.switches[ET08_CONTROL_SWITCH_SC] = REMOTE_SWITCH_UP;
    state.switches[ET08_CONTROL_SWITCH_SD] = REMOTE_SWITCH_UP;
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.fire_mode == ROBOT_FIRE_CONTINUOUS);

    /* SA 下位拥有最高优先级，即便 SC/SD 仍在发射位置也清空请求。 */
    state.switches[ET08_CONTROL_SWITCH_SA] = REMOTE_SWITCH_DOWN;
    CHECK(rm::robot::MapEt08Input(state, kConfig, output));
    CHECK(output.fire_mode == ROBOT_FIRE_DISABLED);
    CHECK(output.fire_trigger_down == 0U);
    CHECK(output.fire_trigger_pressed == 0U);
}

int main()
{
    TestCombinedSwitchLayout();
    TestInvalidSnapshotFailsSafe();
    TestSafeGateAndArmEdge();
    TestUnsafeControlsCannotReuseSafeSnapshot();
    TestModeAndMotionMapping();
    TestFireModeAndTriggerEdges();

    if (failures != 0U) {
        std::fprintf(stderr, "%u ET08-profile checks failed\n", failures);
        return 1;
    }
    std::puts("ET08-profile checks passed");
    return 0;
}
