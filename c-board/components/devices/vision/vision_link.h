/**
 * @file vision_link.h
 * @brief 视觉链路：串口收发、帧调度与链路健康
 * @date 2026-09-11
 * @note 协议基线见 docs/vision/vision_protocol.md
 *
 * 职责边界：
 * - 本模块只负责"把姿态发出去、把目标收回来"，不做弹道解算、不做云台控制。
 * - 遥测发送掛在 20 ms 控制周期；目标接收由 5 ms 电机任务的
 *   USARTDispatchPending() 驱动回调，与现有遥控/裁判驱动一致。
 * - 视觉离线是降级条件而非停机条件，由应用层决定云台行为。
 */

#ifndef RM_VISION_LINK_H
#define RM_VISION_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "vision_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 相邻两帧目标到达间隔上限；超时判定视觉离线。 */
#define RM_VISION_LINK_DEFAULT_TIMEOUT_MS 250U

/** 启动宽限：上位机与机器人分开上电，需给视觉栈留出启动时间。 */
#define RM_VISION_LINK_DEFAULT_GRACE_MS 3000U

typedef struct {
    UART_HandleTypeDef *uart;
    /** 无有效目标帧超过该时间判定离线。0 使用默认值。 */
    uint32_t timeout_ms;
    /** 启动宽限，避免上位机未启动就报离线。0 使用默认值。 */
    uint32_t initial_grace_ms;
} RmVisionLinkConfig;

/**
 * @brief 注册串口端点与健康守护。
 * @note 必须在单线程启动阶段调用，且需在 USARTConfigureDispatch() 之后。
 * @return true 注册成功
 */
bool RmVisionLink_Init(const RmVisionLinkConfig *config);

/**
 * @brief 向上位机发送一帧遥测（姿态 + 模式标志）。
 *
 * 速率由调用方（20 ms 控制任务）决定。串口忙时丢弃本帧而不阻塞，
 * 与调参遥测的策略一致。
 *
 * @param now_ms    当前单调毫秒
 * @param yaw_deg   云台 IMU 累计 Yaw，deg
 * @param pitch_deg 云台 IMU Pitch，**本车符号（抬头为负）**，deg
 * @param roll_deg  云台 IMU Roll，deg
 */
void RmVisionLink_Update(uint32_t now_ms,
                         float yaw_deg,
                         float pitch_deg,
                         float roll_deg);

/** @brief 自瞄模式电平，写入遥测 flags 的 AUTO_AIM_ENABLE 位。 */
void RmVisionLink_SetAutoAim(bool enabled);

/** @brief 敌方颜色，0=红 1=蓝。 */
void RmVisionLink_SetEnemyColor(uint8_t color);

/** @brief 弹速，m/s。 */
void RmVisionLink_SetBulletSpeed(uint8_t speed_mps);

/**
 * @brief 请求上位机复位跟踪器。
 * @note 一次性脉冲：只在下一帧成功入队的遥测帧置位，随后自动清除；串口忙时保留请求，
 *       避免以 50 Hz 反复复位或因一次发送失败丢失复位请求。
 */
void RmVisionLink_RequestTrackerReset(void);

/**
 * @brief 视觉对端是否在线。
 *
 * 只表示守护窗口内收到了 CRC/长度合法的目标帧，与自瞄开关、TRACKING
 * 标志和目标浮点值是否可用无关。目标是否能接管云台请使用
 * RmVisionLink_HasTarget()。
 *
 * @note 视觉离线不是停机条件，调用方据此退回手动控制。
 */
bool RmVisionLink_IsOnline(void);

/**
 * @brief 取最近一帧发给上位机的遥测副本，用于诊断。
 * @return true 曾发出过至少一帧；false 时 out 不变
 */
bool RmVisionLink_GetTelemetry(RmVisionTelemetry *out);

/**
 * @brief 是否可以把视觉目标用于云台控制。
 *
 * 四个条件同时成立才为真：链路在线、自瞄已开启、跟踪器 TRACKING=1、
 * 浮点状态有限。任一不满足都返回 false，调用方应退回手动控制。
 */
bool RmVisionLink_HasTarget(void);

/**
 * @brief 取最近一帧自瞄开启时接收的有限值目标副本。
 * @note TRACKING=0 的帧也会保留，但调用方必须先检查 RmVisionLink_HasTarget()。
 */
bool RmVisionLink_GetTarget(RmVisionTarget *out);

/** @brief 视觉链路统计，供诊断/遥测。 */
typedef struct {
    uint32_t rx_frames;        /* 通过 CRC 与长度校验的目标帧 */
    uint32_t crc_errors;       /* CRC 失败次数 */
    uint32_t format_errors;    /* 帧头对但长度不符的次数 */
    uint32_t resync_bytes;     /* 重同步丢弃的字节数 */
    uint32_t rejected_not_armed;  /* 自瞄未开启时丢弃的目标帧 */
    uint32_t rejected_nonfinite;  /* 含 NaN/Inf 被丢弃的目标帧 */
    uint32_t tx_frames;        /* 成功发出的遥测帧 */
    uint32_t tx_dropped;       /* 串口忙或姿态非有限而丢弃的遥测帧 */
    uint32_t target_frames;    /* 判定为可用的目标帧 */
    uint8_t last_target_id;
    uint8_t last_armors_num;
} RmVisionLinkStats;

bool RmVisionLink_GetStats(RmVisionLinkStats *out);

#ifdef __cplusplus
}
#endif

#endif /* RM_VISION_LINK_H */
