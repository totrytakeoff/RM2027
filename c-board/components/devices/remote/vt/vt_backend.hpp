#ifndef VT_BACKEND_HPP
#define VT_BACKEND_HPP

#include <cstdint>
#include <cstring>

extern "C" {
#include "vt_remote.h"
}

#include "remote_control_state.h"

namespace rm {
namespace remote {

class VtBackend final {
public:
    struct Config {
        UART_HandleTypeDef *uart;
        uint32_t timeout_ms;
    };

    bool init(const Config &config)
    {
        initialized_ =
            config.uart != nullptr &&
            VT_InitWithTimeout(config.uart, config.timeout_ms) != nullptr;
        return initialized_;
    }

    void read(RemoteControlState &state) const
    {
        VT_Ctrl_t input{};

        std::memset(&state, 0, sizeof(state));
        state.type = REMOTE_CONTROL_TYPE_VT;
        state.capabilities = REMOTE_CAPABILITY_STICKS |
                             REMOTE_CAPABILITY_SWITCHES |
                             REMOTE_CAPABILITY_DIAL |
                             REMOTE_CAPABILITY_KEYBOARD |
                             REMOTE_CAPABILITY_MOUSE |
                             REMOTE_CAPABILITY_BUTTONS;
        state.buttons_supported = REMOTE_BUTTON_MOUSE_LEFT |
                                  REMOTE_BUTTON_MOUSE_RIGHT |
                                  REMOTE_BUTTON_MOUSE_MIDDLE |
                                  REMOTE_BUTTON_TRIGGER |
                                  REMOTE_BUTTON_PAUSE |
                                  REMOTE_BUTTON_CUSTOM_LEFT |
                                  REMOTE_BUTTON_CUSTOM_RIGHT;

        if (!initialized_ || !VT_Read(&input)) {
            return;
        }

        state.link_online = 1U;
        if (input.crc_ok == 0U) {
            return;
        }
        state.data_valid = 1U;

        setAxis(state, REMOTE_AXIS_LEFT_X, input.ch3_left_x.centered);
        setAxis(state, REMOTE_AXIS_LEFT_Y, input.ch2_left_y.centered);
        setAxis(state, REMOTE_AXIS_RIGHT_X, input.ch0_right_x.centered);
        setAxis(state, REMOTE_AXIS_RIGHT_Y, input.ch1_right_y.centered);
        setAxis(state, REMOTE_AXIS_DIAL, input.dial.centered);

        state.switches[0] = decodeGear(input.gear);
        if (state.switches[0] != REMOTE_SWITCH_INVALID) {
            state.switch_valid_mask = 1UL;
        }

        state.keys_down = input.keyboard_value;
        state.mouse_x = input.mouse_x;
        state.mouse_y = input.mouse_y;
        state.mouse_z = input.mouse_z;
        if (input.mouse_left_pressed != 0U) {
            state.buttons_down |= REMOTE_BUTTON_MOUSE_LEFT;
        }
        if (input.mouse_right_pressed != 0U) {
            state.buttons_down |= REMOTE_BUTTON_MOUSE_RIGHT;
        }
        if (input.mouse_middle_pressed != 0U) {
            state.buttons_down |= REMOTE_BUTTON_MOUSE_MIDDLE;
        }
        if (input.trigger_pressed != 0U) {
            state.buttons_down |= REMOTE_BUTTON_TRIGGER;
        }
        if (input.pause_pressed != 0U) {
            state.buttons_down |= REMOTE_BUTTON_PAUSE;
        }
        if (input.custom_left_pressed != 0U) {
            state.buttons_down |= REMOTE_BUTTON_CUSTOM_LEFT;
        }
        if (input.custom_right_pressed != 0U) {
            state.buttons_down |= REMOTE_BUTTON_CUSTOM_RIGHT;
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

    static RemoteControlSwitchPosition decodeGear(uint8_t gear)
    {
        switch (gear) {
        case VT_GEAR_C:
            return REMOTE_SWITCH_DOWN;
        case VT_GEAR_N:
            return REMOTE_SWITCH_MIDDLE;
        case VT_GEAR_S:
            return REMOTE_SWITCH_UP;
        default:
            return REMOTE_SWITCH_INVALID;
        }
    }

    bool initialized_ = false;
};

} // namespace remote
} // namespace rm

#endif /* VT_BACKEND_HPP */
