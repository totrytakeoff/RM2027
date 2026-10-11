/**
 * @file vision_link_state.h
 * @brief 视觉链路在线状态与目标可用性状态机
 *
 * 该文件只描述协议帧到达后的逻辑状态，不依赖串口、守护或 HAL，
 * 便于把“链路在线”和“目标可用于控制”分别验证。
 */

#ifndef RM_VISION_LINK_STATE_H
#define RM_VISION_LINK_STATE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** 最近是否收到过 CRC/长度合法的目标帧。 */
    bool peer_online;
    /** 最近一帧在自瞄开启且浮点值有限时被接受。 */
    bool target_valid;
    /** 最近一帧还满足 TRACKING=1。 */
    bool target_usable;
    /** 当前是否允许视觉目标接管云台。 */
    bool auto_aim_enabled;
} RmVisionLinkState;

/** 清空链路状态，默认关闭自瞄。 */
void RmVisionLinkState_Reset(RmVisionLinkState *state);

/** 设置自瞄电平；发生边沿时撤销旧目标，必须等待新帧才能重新接管。 */
void RmVisionLinkState_SetAutoAim(RmVisionLinkState *state, bool enabled);

/** 消费一帧已通过协议校验的目标帧。 */
void RmVisionLinkState_ConsumeFrame(RmVisionLinkState *state,
                                     bool finite,
                                     bool tracking);

/** 守护判定超时；在线和目标状态一并失效。 */
void RmVisionLinkState_OnPeerLost(RmVisionLinkState *state);

/** 查询只代表传输层是否收到合法帧，与自瞄开关和 TRACKING 无关。 */
bool RmVisionLinkState_IsOnline(const RmVisionLinkState *state);

/** 查询目标是否满足自瞄接管条件（不含外部守护时钟）。 */
bool RmVisionLinkState_HasTarget(const RmVisionLinkState *state);

/** 查询是否存在一帧有限值目标（TRACKING 可为 0）。 */
bool RmVisionLinkState_HasValidTarget(const RmVisionLinkState *state);

#ifdef __cplusplus
}
#endif

#endif /* RM_VISION_LINK_STATE_H */
