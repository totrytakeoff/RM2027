#ifndef VT_REMOTE_H
#define VT_REMOTE_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VT_FRAME_SIZE 21u
#define VT_CH_CENTER 1024
#define VT_CH_MIN 364
#define VT_CH_MAX 1684
#define VT_DEFAULT_TIMEOUT_MS 200U

typedef enum
{
    VT_GEAR_C = 0u,
    VT_GEAR_N = 1u,
    VT_GEAR_S = 2u,
} VT_Gear_t;

typedef struct
{
    uint16_t raw;
    int16_t centered;
} VT_Channel_t;

typedef struct
{
    VT_Channel_t ch0_right_x;
    VT_Channel_t ch1_right_y;
    VT_Channel_t ch2_left_y;
    VT_Channel_t ch3_left_x;
    VT_Channel_t dial;

    uint8_t gear;
    uint8_t pause_pressed;
    uint8_t custom_left_pressed;
    uint8_t custom_right_pressed;
    uint8_t trigger_pressed;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left_pressed;
    uint8_t mouse_right_pressed;
    uint8_t mouse_middle_pressed;

    uint16_t keyboard_value;

    uint8_t crc_ok;
    uint32_t frame_count;
    uint32_t bad_count;
} VT_Ctrl_t;

VT_Ctrl_t *VT_Init(UART_HandleTypeDef *uart_handle);
VT_Ctrl_t *VT_InitWithTimeout(UART_HandleTypeDef *uart_handle,
                              uint32_t timeout_ms);
uint8_t VT_IsOnline(void);
/** Copy one coherent control snapshot and return its online state. */
bool VT_Read(VT_Ctrl_t *snapshot);
/** Compatibility live view; prefer VT_Read() in concurrent firmware. */
VT_Ctrl_t *VT_GetCtrl(void);

#ifdef __cplusplus
}
#endif

#endif
