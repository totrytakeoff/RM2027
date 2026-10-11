#ifndef ET08_CONTROL_LAYOUT_H
#define ET08_CONTROL_LAYOUT_H

#include <stdint.h>

#include "remote_control_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/** ET08 物理开关在 RemoteControlState::switches 中的固定槽位。 */
typedef enum {
    ET08_CONTROL_SWITCH_SA = 0U,
    ET08_CONTROL_SWITCH_SB = 1U,
    ET08_CONTROL_SWITCH_SC = 2U,
    ET08_CONTROL_SWITCH_SD = 3U,
} Et08ControlSwitchId;

/** 解码 SA/SD：组合通道的 0~2 为上位，3~5 为下位。 */
static inline RemoteControlSwitchPosition
ET08_ControlDecodeUpperSwitch(uint8_t combined_state)
{
    if (combined_state > 5U) {
        return REMOTE_SWITCH_INVALID;
    }
    return combined_state <= 2U ? REMOTE_SWITCH_UP : REMOTE_SWITCH_DOWN;
}

/** 解码 SB/SC：组合通道在两组中都按上、中、下排列。 */
static inline RemoteControlSwitchPosition
ET08_ControlDecodeLowerSwitch(uint8_t combined_state)
{
    uint8_t position;

    if (combined_state > 5U) {
        return REMOTE_SWITCH_INVALID;
    }
    position = combined_state <= 2U
                   ? combined_state
                   : (uint8_t)(combined_state - 3U);
    switch (position) {
    case 0U:
        return REMOTE_SWITCH_UP;
    case 1U:
        return REMOTE_SWITCH_MIDDLE;
    default:
        return REMOTE_SWITCH_DOWN;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* ET08_CONTROL_LAYOUT_H */
