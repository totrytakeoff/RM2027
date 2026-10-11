#ifndef DT7_BACKEND_HPP
#define DT7_BACKEND_HPP

#include <cstdint>
#include <cstring>

extern "C" {
#include "dt7_remote.h"
}

#include "remote_control_state.h"

namespace rm {
namespace remote {

class Dt7Backend final {
public:
    struct Config {
        UART_HandleTypeDef *uart;
        uint32_t timeout_ms;
    };

    bool init(const Config &config)
    {
        initialized_ =
            config.uart != nullptr &&
            RemoteControlInitWithTimeout(config.uart, config.timeout_ms) !=
                nullptr;
        return initialized_;
    }

    void read(RemoteControlState &state) const
    {
        RC_ctrl_t input[2]{};
        const RC_ctrl_t *current = &input[TEMP];

        std::memset(&state, 0, sizeof(state));
        state.type = REMOTE_CONTROL_TYPE_DT7;
        state.capabilities = REMOTE_CAPABILITY_STICKS |
                             REMOTE_CAPABILITY_SWITCHES |
                             REMOTE_CAPABILITY_DIAL |
                             REMOTE_CAPABILITY_KEYBOARD |
                             REMOTE_CAPABILITY_MOUSE |
                             REMOTE_CAPABILITY_BUTTONS;
        state.buttons_supported = REMOTE_BUTTON_MOUSE_LEFT |
                                  REMOTE_BUTTON_MOUSE_RIGHT;

        if (!initialized_ || !RemoteControlRead(input)) {
            return;
        }

        state.link_online = 1U;
        state.data_valid = 1U;
        setAxis(state, REMOTE_AXIS_LEFT_X, current->rc.rocker_l_);
        setAxis(state, REMOTE_AXIS_LEFT_Y, current->rc.rocker_l1);
        setAxis(state, REMOTE_AXIS_RIGHT_X, current->rc.rocker_r_);
        setAxis(state, REMOTE_AXIS_RIGHT_Y, current->rc.rocker_r1);
        setAxis(state, REMOTE_AXIS_DIAL, current->rc.dial);

        setSwitch(state, 0U, decodeSwitch(current->rc.switch_left));
        setSwitch(state, 1U, decodeSwitch(current->rc.switch_right));

        state.keys_down = current->key[KEY_PRESS].keys;
        state.mouse_x = current->mouse.x;
        state.mouse_y = current->mouse.y;
        state.mouse_z = current->mouse.z;
        if (current->mouse.press_l != 0U) {
            state.buttons_down |= REMOTE_BUTTON_MOUSE_LEFT;
        }
        if (current->mouse.press_r != 0U) {
            state.buttons_down |= REMOTE_BUTTON_MOUSE_RIGHT;
        }
    }

private:
    static float normalize(int16_t value)
    {
        float normalized = static_cast<float>(value) / 660.0F;
        if (normalized > 1.0F) {
            normalized = 1.0F;
        } else if (normalized < -1.0F) {
            normalized = -1.0F;
        }
        return normalized;
    }

    static void setAxis(RemoteControlState &state,
                        RemoteControlAxisId axis,
                        int16_t raw)
    {
        const uint32_t index = static_cast<uint32_t>(axis);
        state.axis_raw[index] = raw;
        state.axis_normalized[index] = normalize(raw);
        state.axis_valid_mask |= 1UL << index;
    }

    static RemoteControlSwitchPosition decodeSwitch(uint8_t value)
    {
        switch (value) {
        case RC_SW_UP:
            return REMOTE_SWITCH_UP;
        case RC_SW_MID:
            return REMOTE_SWITCH_MIDDLE;
        case RC_SW_DOWN:
            return REMOTE_SWITCH_DOWN;
        default:
            return REMOTE_SWITCH_INVALID;
        }
    }

    static void setSwitch(RemoteControlState &state,
                          uint32_t index,
                          RemoteControlSwitchPosition position)
    {
        state.switches[index] = position;
        if (position != REMOTE_SWITCH_INVALID) {
            state.switch_valid_mask |= 1UL << index;
        }
    }

    bool initialized_ = false;
};

} // namespace remote
} // namespace rm

#endif /* DT7_BACKEND_HPP */
