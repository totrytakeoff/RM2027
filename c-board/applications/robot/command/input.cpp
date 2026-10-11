#include "input.h"

#include <cmath>
#include <cstring>

extern "C" {
#include "robot_config.h"
#include "debug.h"
#include "rm_critical.h"
#include "rm_time.h"
#include "usart.h"
}

#include "remote_control_adapter.hpp"

#if ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_ET08
#include "et08_backend.hpp"
#include "et08_input_profile.hpp"
using ActiveRemoteBackend = rm::remote::Et08Backend;
#elif ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_DT7
#include "dt7_backend.hpp"
using ActiveRemoteBackend = rm::remote::Dt7Backend;
#elif ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_VT
#include "vt_backend.hpp"
using ActiveRemoteBackend = rm::remote::VtBackend;
#else
#error "Unsupported ROBOT_REMOTE_BACKEND"
#endif

using ActiveRemoteAdapter =
    rm::remote::RemoteControlAdapter<ActiveRemoteBackend>;

static ActiveRemoteAdapter remote_adapter;
static RemoteControlState latest_remote_state;

static float __attribute__((unused)) ClampIntent(float value)
{
    if (!std::isfinite(value)) {
        return 0.0F;
    }
    if (value > 1.0F) {
        return 1.0F;
    }
    if (value < -1.0F) {
        return -1.0F;
    }
    return value;
}

#if ROBOT_REMOTE_BACKEND != ROBOT_REMOTE_BACKEND_ET08
static float ApplyAxisDeadband(float value)
{
#if ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_DT7
    const float threshold = static_cast<float>(DT7_STICK_DEADZONE_RAW) /
                            static_cast<float>(DT7_STICK_FULL_SCALE_RAW);
#else
    const float threshold = static_cast<float>(VT_STICK_DEADZONE_RAW) /
                            static_cast<float>(VT_STICK_FULL_SCALE_RAW);
#endif
    const float magnitude = std::fabs(value);

    if (magnitude <= threshold) {
        return 0.0F;
    }
    const float rescaled = (magnitude - threshold) / (1.0F - threshold);
    return value < 0.0F ? -rescaled : rescaled;
}

static bool AxisIsNeutral(const RemoteControlState &state,
                          RemoteControlAxisId axis)
{
    const uint32_t index = static_cast<uint32_t>(axis);
    const uint32_t bit = 1UL << index;
#if ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_DT7
    const float threshold = static_cast<float>(DT7_STICK_DEADZONE_RAW) /
                            static_cast<float>(DT7_STICK_FULL_SCALE_RAW);
#else
    const float threshold = static_cast<float>(VT_STICK_DEADZONE_RAW) /
                            static_cast<float>(VT_STICK_FULL_SCALE_RAW);
#endif

    return (state.axis_valid_mask & bit) != 0U &&
           std::fabs(state.axis_normalized[index]) <= threshold;
}
#endif

#if ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_ET08
static void MapSelectedRemote(const RemoteControlState &state,
                              Input_Data_t &data)
{
    const rm::robot::Et08InputProfileConfig config = {
        ET08_STICK_DEADZONE_RAW,
        ET08_STICK_FULL_SCALE_RAW,
    };

    (void)rm::robot::MapEt08Input(state, config, data);
}
#elif ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_VT
static void MapSelectedRemote(const RemoteControlState &state,
                              Input_Data_t &data)
{
    if (state.data_valid == 0U) {
        data.emergency_stop = 1U;
        return;
    }

    const bool paused = (state.buttons_down & REMOTE_BUTTON_PAUSE) != 0U;
    const bool controls_safe =
        state.keys_down == 0U &&
        (state.buttons_down & ~REMOTE_BUTTON_PAUSE) == 0U &&
        state.mouse_x == 0 && state.mouse_y == 0 && state.mouse_z == 0 &&
        AxisIsNeutral(state, REMOTE_AXIS_LEFT_X) &&
        AxisIsNeutral(state, REMOTE_AXIS_LEFT_Y) &&
        AxisIsNeutral(state, REMOTE_AXIS_RIGHT_X) &&
        AxisIsNeutral(state, REMOTE_AXIS_RIGHT_Y) &&
        AxisIsNeutral(state, REMOTE_AXIS_DIAL);
    data.online = 1U;
    data.data_valid = 1U;
    data.emergency_stop = paused ? 1U : 0U;
    data.operator_enable_request = paused ? 0U : 1U;
    data.operator_safe_position = controls_safe ? 1U : 0U;
    data.operator_arm_event =
        (state.buttons_released & REMOTE_BUTTON_PAUSE) != 0U ? 1U : 0U;
    data.control_mode = ROBOT_CONTROL_FOLLOW;
    data.fire_mode = ROBOT_FIRE_DISABLED;

    float speed = VT_CHASSIS_BASE_INTENT;
    if ((state.keys_down & VT_KEY_SHIFT) != 0U) {
        speed *= VT_CHASSIS_FAST_MULT;
    } else if ((state.keys_down & VT_KEY_CTRL) != 0U) {
        speed *= VT_CHASSIS_SLOW_MULT;
    }
    if ((state.keys_down & VT_KEY_W) != 0U) data.chassis_y_intent += speed;
    if ((state.keys_down & VT_KEY_S) != 0U) data.chassis_y_intent -= speed;
    if ((state.keys_down & VT_KEY_A) != 0U) data.chassis_x_intent += speed;
    if ((state.keys_down & VT_KEY_D) != 0U) data.chassis_x_intent -= speed;
    if ((state.keys_down & VT_KEY_Q) != 0U) data.chassis_rotate_intent -= 1.0F;
    if ((state.keys_down & VT_KEY_E) != 0U) data.chassis_rotate_intent += 1.0F;
    data.chassis_x_intent = ClampIntent(data.chassis_x_intent);
    data.chassis_y_intent = ClampIntent(data.chassis_y_intent);
    data.chassis_rotate_intent = ClampIntent(data.chassis_rotate_intent);

    if (state.mouse_x != 0 || state.mouse_y != 0) {
        data.gimbal_yaw_intent = ClampIntent(
            static_cast<float>(state.mouse_x) * VT_MOUSE_YAW_INTENT_PER_COUNT);
        data.gimbal_pitch_intent = ClampIntent(
            -static_cast<float>(state.mouse_y) * VT_MOUSE_PITCH_INTENT_PER_COUNT);
    } else {
        data.gimbal_yaw_intent = ApplyAxisDeadband(
            state.axis_normalized[REMOTE_AXIS_LEFT_X]);
        data.gimbal_pitch_intent = ApplyAxisDeadband(
            state.axis_normalized[REMOTE_AXIS_LEFT_Y]);
    }
    data.yaw_control_active = data.gimbal_yaw_intent != 0.0F ? 1U : 0U;
    data.pitch_control_active = data.gimbal_pitch_intent != 0.0F ? 1U : 0U;
}
#else
static void MapSelectedRemote(const RemoteControlState &state,
                              Input_Data_t &data)
{
    if (state.data_valid == 0U ||
        (state.switch_valid_mask & 0x3U) != 0x3U) {
        data.emergency_stop = 1U;
        return;
    }

    const RemoteControlSwitchPosition safety = state.switches[1];
    const bool controls_safe =
        state.switches[0] == REMOTE_SWITCH_DOWN &&
        state.keys_down == 0U && state.buttons_down == 0U &&
        state.mouse_x == 0 && state.mouse_y == 0 && state.mouse_z == 0 &&
        AxisIsNeutral(state, REMOTE_AXIS_LEFT_X) &&
        AxisIsNeutral(state, REMOTE_AXIS_LEFT_Y) &&
        AxisIsNeutral(state, REMOTE_AXIS_RIGHT_X) &&
        AxisIsNeutral(state, REMOTE_AXIS_RIGHT_Y) &&
        AxisIsNeutral(state, REMOTE_AXIS_DIAL);
    data.online = 1U;
    data.data_valid = 1U;
    data.operator_enable_request =
        safety == REMOTE_SWITCH_UP ? 1U : 0U;
    data.operator_safe_position = controls_safe ? 1U : 0U;
    data.operator_arm_event =
        ((state.switch_changed_mask & (1UL << 1U)) != 0U &&
         safety == REMOTE_SWITCH_UP)
            ? 1U
            : 0U;
    data.control_mode = ROBOT_CONTROL_FOLLOW;
    data.fire_mode = ROBOT_FIRE_DISABLED;
    if (data.operator_enable_request != 0U) {
        data.chassis_x_intent = -ApplyAxisDeadband(
            state.axis_normalized[REMOTE_AXIS_LEFT_X]);
        data.chassis_y_intent = ApplyAxisDeadband(
            state.axis_normalized[REMOTE_AXIS_LEFT_Y]);
        data.gimbal_yaw_intent = ApplyAxisDeadband(
            state.axis_normalized[REMOTE_AXIS_RIGHT_X]);
        data.gimbal_pitch_intent = ApplyAxisDeadband(
            state.axis_normalized[REMOTE_AXIS_RIGHT_Y]);
    }
    data.yaw_control_active = data.gimbal_yaw_intent != 0.0F ? 1U : 0U;
    data.pitch_control_active = data.gimbal_pitch_intent != 0.0F ? 1U : 0U;
}
#endif

static void PublishRemoteState(const RemoteControlState &state)
{
    const RmCriticalState critical_state = RmCritical_Enter();
    latest_remote_state = state;
    RmCritical_Exit(critical_state);
}

extern "C" bool Input_Init(void)
{
    ActiveRemoteAdapter::Config config{};

#if ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_ET08
    config.uart = &RC_UART;
    config.timeout_ms = ET08_ONLINE_TIMEOUT_MS;
#elif ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_DT7
    config.uart = &RC_UART;
    config.timeout_ms = DT7_ONLINE_TIMEOUT_MS;
#else
    config.uart = &VT_UART;
    config.timeout_ms = VT_ONLINE_TIMEOUT_MS;
#endif

    std::memset(&latest_remote_state, 0, sizeof(latest_remote_state));
    const bool initialized = remote_adapter.init(config);
    MDBG_IN("selected remote backend=%u init=%u",
            static_cast<unsigned>(ROBOT_REMOTE_BACKEND),
            initialized ? 1U : 0U);
    return initialized;
}

extern "C" void Input_GetData(Input_Data_t *data)
{
    RemoteControlState remote{};

    if (data == nullptr) {
        return;
    }

    std::memset(data, 0, sizeof(*data));
    (void)remote_adapter.read(RmTime_NowMs(), remote);
    MapSelectedRemote(remote, *data);
    PublishRemoteState(remote);
}

extern "C" bool Input_GetRemoteState(RemoteControlState *state)
{
    RmCriticalState critical_state;

    if (state == nullptr) {
        return false;
    }
    critical_state = RmCritical_Enter();
    *state = latest_remote_state;
    RmCritical_Exit(critical_state);
    return true;
}

extern "C" uint8_t Input_IsOnline(void)
{
    RemoteControlState state{};
    return Input_GetRemoteState(&state) && state.data_valid != 0U ? 1U : 0U;
}

extern "C" RemoteControlType Input_GetRemoteType(void)
{
    RemoteControlState state{};
    return Input_GetRemoteState(&state) ? state.type
                                        : REMOTE_CONTROL_TYPE_NONE;
}
