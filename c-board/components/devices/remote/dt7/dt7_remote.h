#ifndef DT7_REMOTE_H
#define DT7_REMOTE_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    TEMP = 0,
    LAST = 1,
    KEY_PRESS = 0,
    KEY_PRESS_WITH_CTRL = 1,
    KEY_PRESS_WITH_SHIFT = 2,
};

#define DT7_DEFAULT_TIMEOUT_MS 100U
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) ((s) == RC_SW_DOWN)
#define switch_is_mid(s) ((s) == RC_SW_MID)
#define switch_is_up(s) ((s) == RC_SW_UP)

typedef enum {
    Key_W = 0,
    Key_S,
    Key_D,
    Key_A,
    Key_Shift,
    Key_Ctrl,
    Key_Q,
    Key_E,
    Key_R,
    Key_F,
    Key_G,
    Key_Z,
    Key_X,
    Key_C,
    Key_V,
    Key_B,
} RemoteControlKey;

typedef union {
    struct {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t d : 1;
        uint16_t a : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys;
} Key_t;

typedef struct {
    struct {
        int16_t rocker_l_;
        int16_t rocker_l1;
        int16_t rocker_r_;
        int16_t rocker_r1;
        int16_t dial;
        uint8_t switch_left;
        uint8_t switch_right;
    } rc;
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    Key_t key[3];
    uint8_t key_count[3][16];
} RC_ctrl_t;

/** Initialize the single DT7/DR16 DBUS link. */
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);
RC_ctrl_t *RemoteControlInitWithTimeout(UART_HandleTypeDef *rc_usart_handle,
                                       uint32_t timeout_ms);

/** Copy TEMP/LAST as one coherent snapshot. */
bool RemoteControlRead(RC_ctrl_t snapshot[2]);

/** The DT7 link is the only source represented by this compatibility API. */
uint8_t RemoteControlIsOnline(void);
uint8_t RemoteControlIsDt7Online(void);

#ifdef __cplusplus
}
#endif

#endif /* DT7_REMOTE_H */
