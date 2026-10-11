#ifndef ET08_REMOTE_H
#define ET08_REMOTE_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ET08_CHANNEL_COUNT 8
#define ET08_CHANNEL_COUNT_FULL 16
#define ET08_CHANNEL_CENTER 1024
#define ET08_SWITCH_LEVEL_COUNT 6

/* RF206S 实测的六档 SBUS 原始值，由高到低排列。 */
#define ET08_SWITCH_LEVEL_0 1896
#define ET08_SWITCH_LEVEL_1 1694
#define ET08_SWITCH_LEVEL_2 1493
#define ET08_SWITCH_LEVEL_3 554
#define ET08_SWITCH_LEVEL_4 353
#define ET08_SWITCH_LEVEL_5 151

#define ET08_SWITCH_TOLERANCE 200
#define ET08_DEFAULT_TIMEOUT_MS 4000U

typedef enum
{
    ET08_CH1 = 0,
    ET08_CH2 = 1,
    ET08_CH3 = 2,
    ET08_CH4 = 3,
    ET08_CH5 = 4,
    ET08_CH6 = 5,
    ET08_CH7 = 6,
    ET08_CH8 = 7,
} ET08_Channel_t;

// ---------------- Channel Mapping (tunable) ----------------
// If your transmitter/receiver channel assignment differs, adjust these macros.
// Default mapping follows the original implementation:
// - Right stick: CH1/CH2
// - Left stick : CH4/CH3
// - SA/SB      : CH5
// - SD/SC      : CH6
// - Knobs      : CH7/CH8
#ifndef ET08_MAP_RIGHT_X_CH
#define ET08_MAP_RIGHT_X_CH ET08_CH1
#endif
#ifndef ET08_MAP_RIGHT_Y_CH
#define ET08_MAP_RIGHT_Y_CH ET08_CH2
#endif
#ifndef ET08_MAP_LEFT_Y_CH
#define ET08_MAP_LEFT_Y_CH ET08_CH3
#endif
#ifndef ET08_MAP_LEFT_X_CH
#define ET08_MAP_LEFT_X_CH ET08_CH4
#endif
#ifndef ET08_MAP_SA_SB_CH
#define ET08_MAP_SA_SB_CH ET08_CH5
#endif
#ifndef ET08_MAP_SD_SC_CH
#define ET08_MAP_SD_SC_CH ET08_CH6
#endif
#ifndef ET08_MAP_KNOB_LEFT_CH
#define ET08_MAP_KNOB_LEFT_CH ET08_CH7
#endif
#ifndef ET08_MAP_KNOB_RIGHT_CH
#define ET08_MAP_KNOB_RIGHT_CH ET08_CH8
#endif

typedef struct
{
    int16_t x;
    int16_t y;
} ET08_Stick_t;

typedef struct
{
    uint16_t raw[ET08_CHANNEL_COUNT];
    int16_t centered[ET08_CHANNEL_COUNT];

    // Full 16 SBUS channels (for debugging / remapping).
    uint16_t raw_full[ET08_CHANNEL_COUNT_FULL];
    int16_t centered_full[ET08_CHANNEL_COUNT_FULL];

    ET08_Stick_t right;
    ET08_Stick_t left;

    int16_t knob_left;
    int16_t knob_right;

    uint16_t switch_sa_sb_raw;
    int16_t switch_sa_sb_centered;
    uint8_t switch_sa_sb_state;

    uint16_t switch_sd_sc_raw;
    int16_t switch_sd_sc_centered;
    uint8_t switch_sd_sc_state;

    uint8_t frame_lost;
    uint8_t failsafe;
} ET08_Ctrl_t;

ET08_Ctrl_t *ET08_Init(UART_HandleTypeDef *uart_handle);
ET08_Ctrl_t *ET08_InitWithTimeout(UART_HandleTypeDef *uart_handle,
                                  uint32_t timeout_ms);
uint8_t ET08_IsOnline(void);
/** Copy one coherent control snapshot and return its online state. */
bool ET08_Read(ET08_Ctrl_t *snapshot);
/** Compatibility live view; prefer ET08_Read() in concurrent firmware. */
ET08_Ctrl_t *ET08_GetCtrl(void);
uint8_t ET08_MapSwitchState(uint16_t raw_value);

#ifdef __cplusplus
}
#endif

#endif
