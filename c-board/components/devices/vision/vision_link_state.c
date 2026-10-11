/**
 * @file vision_link_state.c
 * @brief 视觉链路状态机实现
 */

#include "vision_link_state.h"

#include <stddef.h>

void RmVisionLinkState_Reset(RmVisionLinkState *state)
{
    if (state == NULL) {
        return;
    }

    state->peer_online = false;
    state->target_valid = false;
    state->target_usable = false;
    state->auto_aim_enabled = false;
}

void RmVisionLinkState_SetAutoAim(RmVisionLinkState *state, bool enabled)
{
    if (state == NULL) {
        return;
    }

    if (state->auto_aim_enabled == enabled) {
        return;
    }

    state->auto_aim_enabled = enabled;
    /* 关闭和重新开启都必须等到新帧，禁止复用旧目标。 */
    state->target_valid = false;
    state->target_usable = false;
}

void RmVisionLinkState_ConsumeFrame(RmVisionLinkState *state,
                                     bool finite,
                                     bool tracking)
{
    if (state == NULL) {
        return;
    }

    /* 合法帧证明对端在线，自瞄是否开启只影响目标接管。 */
    state->peer_online = true;
    state->target_valid = false;
    state->target_usable = false;

    if (!state->auto_aim_enabled || !finite) {
        return;
    }

    state->target_valid = true;
    state->target_usable = tracking;
}

void RmVisionLinkState_OnPeerLost(RmVisionLinkState *state)
{
    if (state == NULL) {
        return;
    }

    state->peer_online = false;
    state->target_valid = false;
    state->target_usable = false;
}

bool RmVisionLinkState_IsOnline(const RmVisionLinkState *state)
{
    return (state != NULL) && state->peer_online;
}

bool RmVisionLinkState_HasTarget(const RmVisionLinkState *state)
{
    return (state != NULL) && state->peer_online &&
           state->auto_aim_enabled && state->target_valid &&
           state->target_usable;
}

bool RmVisionLinkState_HasValidTarget(const RmVisionLinkState *state)
{
    return (state != NULL) && state->target_valid;
}
