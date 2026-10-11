#ifndef RM_REMOTE_H
#define RM_REMOTE_H
#include "remote_control_state.h"
#include <stdbool.h>
#include <stddef.h>
typedef enum { RM_REMOTE_SBUS,RM_REMOTE_DBUS } RmRemoteProtocol;
typedef struct {
    RmRemoteProtocol protocol;
    uint32_t timeout_ms;
    uint16_t center,span,switch_tolerance,switch_levels[6];
    uint8_t right_x,right_y,left_x,left_y,combined_sa_sb,combined_sd_sc,knob_left,knob_right;
} RmRemoteConfig;
typedef struct {
    uint32_t frames,good_frames,bad_frames,frame_lost,failsafe,discontinuities;
} RmRemoteStats;
typedef struct {
    RmRemoteConfig config;
    RemoteControlState state;
    RmRemoteStats stats;
    uint16_t raw_channels[16];
    uint8_t buffer[25],used,good_streak;
    uint32_t generation,last_good_ms;
    bool initialized,have_good,locked;
} RmRemote;
/* Single owner. No UART setup or HAL access belongs in the parser. */
bool RmRemote_Init(RmRemote *,const RmRemoteConfig *);
void RmRemote_Discontinuity(RmRemote *,uint32_t generation);
void RmRemote_Push(RmRemote *,const uint8_t *,size_t,uint32_t sample_ms,uint32_t generation);
bool RmRemote_Read(const RmRemote *,uint32_t now_ms,RemoteControlState *);
/* Consumer consumes accumulated input edges once; synchronize in assembly. */
bool RmRemote_Take(RmRemote *,uint32_t now_ms,RemoteControlState *);
#endif
