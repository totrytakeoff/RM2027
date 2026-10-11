#ifndef REMOTE_CONTROL_ADAPTER_HPP
#define REMOTE_CONTROL_ADAPTER_HPP

#include <cstdint>
#include <cstring>

#include "remote_control_state.h"

namespace rm {
namespace remote {

template <typename Backend>
class RemoteControlAdapter final {
public:
    using Config = typename Backend::Config;

    constexpr RemoteControlAdapter()
        : backend_{}, previous_{}, sample_sequence_(0U),
          initialized_(false), previous_valid_(false)
    {
    }

    bool init(const Config &config)
    {
        std::memset(&previous_, 0, sizeof(previous_));
        sample_sequence_ = 0U;
        previous_valid_ = false;
        initialized_ = backend_.init(config);
        return initialized_;
    }

    bool read(uint32_t now_ms, RemoteControlState &output)
    {
        RemoteControlState current{};

        if (!initialized_) {
            output = current;
            return false;
        }

        backend_.read(current);
        current.sample_time_ms = now_ms;
        current.sample_sequence = ++sample_sequence_;
        /* 边沿字段只由适配器生成，后端只负责发布当前电平。 */
        current.keys_pressed = 0U;
        current.keys_released = 0U;
        current.buttons_pressed = 0U;
        current.buttons_released = 0U;
        current.switch_changed_mask = 0U;

        if (current.data_valid != 0U && previous_valid_) {
            current.keys_pressed = current.keys_down & ~previous_.keys_down;
            current.keys_released = previous_.keys_down & ~current.keys_down;
            current.buttons_pressed =
                current.buttons_down & ~previous_.buttons_down;
            current.buttons_released =
                previous_.buttons_down & ~current.buttons_down;

            for (uint32_t index = 0U;
                 index < static_cast<uint32_t>(REMOTE_CONTROL_SWITCH_COUNT);
                 ++index) {
                const uint32_t bit = 1UL << index;
                if ((current.switch_valid_mask & bit) != 0U &&
                    (previous_.switch_valid_mask & bit) != 0U &&
                    current.switches[index] != previous_.switches[index]) {
                    current.switch_changed_mask |= bit;
                }
            }
        }

        if (current.data_valid != 0U) {
            previous_ = current;
            previous_.keys_pressed = 0U;
            previous_.keys_released = 0U;
            previous_.buttons_pressed = 0U;
            previous_.buttons_released = 0U;
            previous_.switch_changed_mask = 0U;
            previous_valid_ = true;
        } else {
            previous_valid_ = false;
        }

        output = current;
        return current.data_valid != 0U;
    }

    bool initialized() const
    {
        return initialized_;
    }

private:
    Backend backend_;
    RemoteControlState previous_;
    uint32_t sample_sequence_;
    bool initialized_;
    bool previous_valid_;
};

} // namespace remote
} // namespace rm

#endif /* REMOTE_CONTROL_ADAPTER_HPP */
