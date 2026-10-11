#ifndef ET08_BACKEND_HPP
#define ET08_BACKEND_HPP

#include <cstdint>
#include <cstring>

extern "C" {
#include "et08_remote.h"
}

#include "remote_control_state.h"
#include "et08_control_layout.h"

namespace rm {
namespace remote {

class Et08Backend final {
public:
    struct Config {
        UART_HandleTypeDef *uart;
        uint32_t timeout_ms;
    };

    bool init(const Config &config)
    {
        initialized_ =
            config.uart != nullptr &&
            ET08_InitWithTimeout(config.uart, config.timeout_ms) != nullptr;
        return initialized_;
    }

    void read(RemoteControlState &state) const
    {
        ET08_Ctrl_t input{};

        std::memset(&state, 0, sizeof(state));
        state.type = REMOTE_CONTROL_TYPE_ET08;
        state.capabilities = REMOTE_CAPABILITY_STICKS |
                             REMOTE_CAPABILITY_SWITCHES |
                             REMOTE_CAPABILITY_KNOBS |
                             REMOTE_CAPABILITY_AUX_AXES;

        if (!initialized_ || !ET08_Read(&input)) {
            return;
        }

        state.link_online = 1U;
        state.failsafe = input.failsafe;
        state.frame_lost = input.frame_lost;
        if (state.failsafe != 0U || state.frame_lost != 0U) {
            return;
        }

        state.data_valid = 1U;
        setAxis(state, REMOTE_AXIS_LEFT_X, input.left.x);
        setAxis(state, REMOTE_AXIS_LEFT_Y, input.left.y);
        setAxis(state, REMOTE_AXIS_RIGHT_X, input.right.x);
        setAxis(state, REMOTE_AXIS_RIGHT_Y, input.right.y);
        setAxis(state, REMOTE_AXIS_KNOB_LEFT, input.knob_left);
        setAxis(state, REMOTE_AXIS_KNOB_RIGHT, input.knob_right);
        for (uint32_t index = 0U; index < 8U; ++index) {
            setAxis(state,
                    static_cast<RemoteControlAxisId>(REMOTE_AXIS_AUX_0 + index),
                    input.centered_full[index + 8U]);
        }

        setSwitch(state, ET08_CONTROL_SWITCH_SA,
                  ET08_ControlDecodeUpperSwitch(input.switch_sa_sb_state));
        setSwitch(state, ET08_CONTROL_SWITCH_SB,
                  ET08_ControlDecodeLowerSwitch(input.switch_sa_sb_state));
        setSwitch(state, ET08_CONTROL_SWITCH_SC,
                  ET08_ControlDecodeLowerSwitch(input.switch_sd_sc_state));
        setSwitch(state, ET08_CONTROL_SWITCH_SD,
                  ET08_ControlDecodeUpperSwitch(input.switch_sd_sc_state));
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

#endif /* ET08_BACKEND_HPP */
