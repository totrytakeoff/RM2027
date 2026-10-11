#ifndef REMOTE_CONTROL_STATE_H
#define REMOTE_CONTROL_STATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    REMOTE_CONTROL_AXIS_COUNT = 16,
    REMOTE_CONTROL_SWITCH_COUNT = 8,
};

typedef enum {
    REMOTE_CONTROL_TYPE_NONE = 0,
    REMOTE_CONTROL_TYPE_ET08,
    REMOTE_CONTROL_TYPE_DT7,
    REMOTE_CONTROL_TYPE_VT,
} RemoteControlType;

typedef enum {
    REMOTE_AXIS_LEFT_X = 0,
    REMOTE_AXIS_LEFT_Y,
    REMOTE_AXIS_RIGHT_X,
    REMOTE_AXIS_RIGHT_Y,
    REMOTE_AXIS_DIAL,
    REMOTE_AXIS_KNOB_LEFT,
    REMOTE_AXIS_KNOB_RIGHT,
    REMOTE_AXIS_AUX_0,
    REMOTE_AXIS_AUX_1,
    REMOTE_AXIS_AUX_2,
    REMOTE_AXIS_AUX_3,
    REMOTE_AXIS_AUX_4,
    REMOTE_AXIS_AUX_5,
    REMOTE_AXIS_AUX_6,
    REMOTE_AXIS_AUX_7,
    REMOTE_AXIS_AUX_8,
} RemoteControlAxisId;

typedef enum {
    REMOTE_SWITCH_INVALID = 0,
    REMOTE_SWITCH_UP,
    REMOTE_SWITCH_MIDDLE,
    REMOTE_SWITCH_DOWN,
} RemoteControlSwitchPosition;

typedef enum {
    REMOTE_BUTTON_MOUSE_LEFT = (1UL << 0),
    REMOTE_BUTTON_MOUSE_RIGHT = (1UL << 1),
    REMOTE_BUTTON_MOUSE_MIDDLE = (1UL << 2),
    REMOTE_BUTTON_TRIGGER = (1UL << 3),
    REMOTE_BUTTON_PAUSE = (1UL << 4),
    REMOTE_BUTTON_CUSTOM_LEFT = (1UL << 5),
    REMOTE_BUTTON_CUSTOM_RIGHT = (1UL << 6),
} RemoteControlButtonMask;

typedef enum {
    REMOTE_CAPABILITY_STICKS = (1UL << 0),
    REMOTE_CAPABILITY_SWITCHES = (1UL << 1),
    REMOTE_CAPABILITY_DIAL = (1UL << 2),
    REMOTE_CAPABILITY_KNOBS = (1UL << 3),
    REMOTE_CAPABILITY_AUX_AXES = (1UL << 4),
    REMOTE_CAPABILITY_KEYBOARD = (1UL << 5),
    REMOTE_CAPABILITY_MOUSE = (1UL << 6),
    REMOTE_CAPABILITY_BUTTONS = (1UL << 7),
} RemoteControlCapabilityMask;

typedef struct {
    RemoteControlType type;
    uint32_t capabilities;

    uint8_t link_online;
    uint8_t data_valid;
    uint8_t failsafe;
    uint8_t frame_lost;

    uint32_t sample_sequence;
    uint32_t sample_time_ms;

    int16_t axis_raw[REMOTE_CONTROL_AXIS_COUNT];
    float axis_normalized[REMOTE_CONTROL_AXIS_COUNT];
    uint32_t axis_valid_mask;

    RemoteControlSwitchPosition switches[REMOTE_CONTROL_SWITCH_COUNT];
    uint32_t switch_valid_mask;
    uint32_t switch_changed_mask;

    uint32_t keys_down;
    uint32_t keys_pressed;
    uint32_t keys_released;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint32_t buttons_supported;
    uint32_t buttons_down;
    uint32_t buttons_pressed;
    uint32_t buttons_released;
} RemoteControlState;

#ifdef __cplusplus
}
#endif

#endif /* REMOTE_CONTROL_STATE_H */
