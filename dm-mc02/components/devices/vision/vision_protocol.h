/**
 * @file vision_protocol.h
 * @brief 视觉上下位机通讯协议 —— 帧编解码（纯逻辑，不依赖 HAL/FreeRTOS）
 * @date 2026-09-11
 * @note 协议基线见 docs/vision/vision_protocol.md
 *
 * 方向约定（以下均以 MCU 为视角），与上游 rm_serial_driver 保持一致：
 * - 遥测 telemetry：MCU -> 上位机，帧头 0x5A，payload 18 字节，整帧 22 字节
 * - 目标 target   ：上位机 -> MCU，帧头 0xA5，payload 48 字节，整帧 52 字节
 *
 * 上位机 `RMSerialDriver::receiveData()` 收 0x5A，`sendData()` 发 0xA5。
 *
 * 帧格式：header(1) + length(1) + payload(N) + crc16(2, 小端)
 * 校验：CRC-16/MCRF4XX，多项式 0x8408 反射，初值 0xFFFF
 * 字节序：全部小端
 */

#ifndef RM_VISION_PROTOCOL_H
#define RM_VISION_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__cplusplus)
#define RM_VISION_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define RM_VISION_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

/*============================================================================
 * 帧结构常量
 *============================================================================*/
#define RM_VISION_HEADER_TELEMETRY 0x5AU /* MCU -> 上位机 */
#define RM_VISION_HEADER_TARGET 0xA5U    /* 上位机 -> MCU */

#define RM_VISION_TELEMETRY_PAYLOAD_LEN 18U
#define RM_VISION_TARGET_PAYLOAD_LEN 48U

/** header + length + crc16 */
#define RM_VISION_FRAME_OVERHEAD 4U
#define RM_VISION_TELEMETRY_FRAME_LEN \
    (RM_VISION_TELEMETRY_PAYLOAD_LEN + RM_VISION_FRAME_OVERHEAD)
#define RM_VISION_TARGET_FRAME_LEN \
    (RM_VISION_TARGET_PAYLOAD_LEN + RM_VISION_FRAME_OVERHEAD)

/** 接收累积缓冲：至少容纳一个整帧，并为乱码重同步留余量。 */
#define RM_VISION_RX_ACCUM_CAPACITY 64U

/*============================================================================
 * 遥测 flags（MCU -> 上位机）
 *
 * 名称沿用上游 ReceivePacket 的位序，使上位机只改结构体布局、不改位定义。
 *============================================================================*/
#define RM_VISION_TELEM_FLAG_ENEMY_COLOR (1U << 0)        /* 0=红 1=蓝 */
#define RM_VISION_TELEM_FLAG_RESET_TRACKER (1U << 1)      /* 请求清空跟踪器 */
#define RM_VISION_TELEM_FLAG_AUTO_AIM_ENABLE (1U << 2)    /* 自瞄模式使能 */
#define RM_VISION_TELEM_FLAG_MANUAL_TRIM_ENABLE (1U << 3) /* 预留：摇杆微调 */
#define RM_VISION_TELEM_FLAG_FIRE_MODE_AUTO (1U << 4)     /* 预留：自动开火 */

/*============================================================================
 * 目标 flags（上位机 -> MCU）
 *============================================================================*/
#define RM_VISION_TARGET_FLAG_TRACKING (1U << 0)
#define RM_VISION_TARGET_FLAG_ID_SHIFT 1U
#define RM_VISION_TARGET_FLAG_ID_MASK \
    (0x7U << RM_VISION_TARGET_FLAG_ID_SHIFT)
#define RM_VISION_TARGET_FLAG_ARMORS_SHIFT 4U
#define RM_VISION_TARGET_FLAG_ARMORS_MASK \
    (0x7U << RM_VISION_TARGET_FLAG_ARMORS_SHIFT)
#define RM_VISION_TARGET_FLAG_FIRE_ADVICE (1U << 7) /* 预留 */

/** 装甲板数量取值 */
#define RM_VISION_ARMORS_BALANCE_2 2U
#define RM_VISION_ARMORS_OUTPOST_3 3U
#define RM_VISION_ARMORS_NORMAL_4 4U

/*============================================================================
 * 线上结构（wire）—— 逐字节布局，禁止直接用于业务逻辑
 *============================================================================*/
#pragma pack(push, 1)

typedef struct {
    uint8_t header;
    uint8_t length;
    uint16_t flags;
    uint16_t seq;
    uint8_t bullet_speed;
    uint8_t reserved;
    float yaw;
    float pitch;
    float roll;
    uint16_t crc16;
} RmVisionTelemetryWire;

typedef struct {
    uint8_t header;
    uint8_t length;
    uint16_t flags;
    uint16_t seq;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    uint16_t crc16;
} RmVisionTargetWire;

#pragma pack(pop)

/*============================================================================
 * 语义结构 —— 业务逻辑使用，不含帧头/长度/校验
 *============================================================================*/

/** MCU 上报给上位机的状态。角度为本车 IMU 符号。 */
typedef struct {
    uint16_t flags;
    uint16_t seq;
    uint8_t bullet_speed;
    /** 云台 IMU 角，deg。符号约定见 RmVision_PitchImuToWire()。 */
    float yaw;
    float pitch;
    float roll;
} RmVisionTelemetry;

/** 上位机下发的目标状态，云台系，对应 auto_aim_interfaces/msg/Target。 */
typedef struct {
    uint16_t flags;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
} RmVisionTarget;

/** 接收流解码器：处理 UART 分片、粘包与乱码重同步。 */
typedef struct {
    uint8_t storage[RM_VISION_RX_ACCUM_CAPACITY];
    uint16_t length;
    uint32_t accepted_frames;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t resync_bytes;
} RmVisionRxStream;

/*============================================================================
 * 编译期布局断言 —— 任一侧结构变化立即编译失败
 *============================================================================*/
RM_VISION_STATIC_ASSERT(sizeof(RmVisionTelemetryWire) ==
                            RM_VISION_TELEMETRY_FRAME_LEN,
                        "telemetry wire size must match frame length");
RM_VISION_STATIC_ASSERT(sizeof(RmVisionTargetWire) == RM_VISION_TARGET_FRAME_LEN,
                        "target wire size must match frame length");

RM_VISION_STATIC_ASSERT(offsetof(RmVisionTelemetryWire, header) == 0, "tl hdr");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTelemetryWire, length) == 1, "tl len");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTelemetryWire, flags) == 2, "tl flags");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTelemetryWire, seq) == 4, "tl seq");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTelemetryWire, bullet_speed) == 6,
                        "tl bullet");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTelemetryWire, reserved) == 7,
                        "tl rsvd");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTelemetryWire, yaw) == 8, "tl yaw");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTelemetryWire, pitch) == 12,
                        "tl pitch");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTelemetryWire, roll) == 16, "tl roll");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTelemetryWire, crc16) == 20,
                        "tl crc");

RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, header) == 0, "tg hdr");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, length) == 1, "tg len");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, flags) == 2, "tg flags");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, seq) == 4, "tg seq");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, x) == 6, "tg x");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, y) == 10, "tg y");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, z) == 14, "tg z");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, yaw) == 18, "tg yaw");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, vx) == 22, "tg vx");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, vy) == 26, "tg vy");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, vz) == 30, "tg vz");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, v_yaw) == 34, "tg v_yaw");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, r1) == 38, "tg r1");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, r2) == 42, "tg r2");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, dz) == 46, "tg dz");
RM_VISION_STATIC_ASSERT(offsetof(RmVisionTargetWire, crc16) == 50, "tg crc");

/*============================================================================
 * 初始化
 *============================================================================*/

/**
 * @brief 预热 CRC 查表并清零解码器状态。
 * @note 必须在正式路径首次收发前调用，避免查表初始化发生在中断或控制周期内。
 */
void RmVision_Init(void);

/** @brief 清零解码器的缓冲与统计，不重置 CRC 表。 */
void RmVisionRx_Reset(RmVisionRxStream *rx);

/*============================================================================
 * 编码：MCU -> 上位机（遥测帧）
 *============================================================================*/

/**
 * @brief 将语义结构编码为遥测帧（含帧头、长度与 CRC）。
 * @param src     语义结构，不可为 NULL
 * @param buf     输出缓冲
 * @param buf_len 输出缓冲容量，必须 >= RM_VISION_TELEMETRY_FRAME_LEN
 * @return 写入的字节数；参数非法或缓冲不足时返回 0
 */
uint16_t RmVision_EncodeTelemetry(const RmVisionTelemetry *src,
                                  uint8_t *buf,
                                  uint16_t buf_len);

/**
 * @brief 生成一帧"手动模式、零姿态"的默认遥测内容。
 * @note 位域全部为 0、角度全 0，用于链路心跳与初始状态。
 */
void RmVisionTelemetry_InitEmpty(RmVisionTelemetry *pkt);

/*============================================================================
 * 解码：上位机 -> MCU（目标帧）
 *============================================================================*/

/**
 * @brief 解码并校验单个目标帧。
 * @param frame     完整帧起始地址
 * @param frame_len 可用字节数，必须 >= RM_VISION_TARGET_FRAME_LEN
 * @param out       解码结果，可为 NULL（仅校验）
 * @return true 帧头、长度与 CRC 全部匹配
 */
bool RmVision_DecodeTarget(const uint8_t *frame,
                           uint16_t frame_len,
                           RmVisionTarget *out);

/**
 * @brief 向解码器推送任意长度的接收数据。
 *
 * 处理 UART 分片、粘包与乱码：按帧头重同步，逐帧校验。只要本次推送中
 * 解出至少一个有效帧就返回 true，*out 为其中最后一帧（控制以最新为准）。
 *
 * @param rx  解码器状态
 * @param data 收到的字节
 * @param len  字节数
 * @param out  最新有效帧，可为 NULL
 * @return true 本次推送至少解出一个有效帧
 */
bool RmVisionRx_Push(RmVisionRxStream *rx,
                     const uint8_t *data,
                     uint16_t len,
                     RmVisionTarget *out);

/*============================================================================
 * 语义辅助
 *============================================================================*/

/**
 * @brief 校验遥测角度字段均为有限值（非 NaN / 非 Inf）。
 * @note 上行的姿态由本机产生，此处仅作出口自检。
 */
bool RmVisionTelemetry_HasFiniteAngles(const RmVisionTelemetry *pkt);

/**
 * @brief 校验目标状态的全部浮点字段均为有限值。
 * @note 非法浮点必须按"失效关闭"处理，不得传播到云台参考。
 */
bool RmVisionTarget_HasFiniteState(const RmVisionTarget *pkt);

/** @brief 读取目标 flags 中的目标编号（0..7）。 */
uint8_t RmVisionTarget_GetTargetId(const RmVisionTarget *pkt);

/** @brief 读取目标 flags 中的装甲板数量（2/3/4）。 */
uint8_t RmVisionTarget_GetArmorsNum(const RmVisionTarget *pkt);

/** @brief 设置目标 flags 中的目标编号与装甲板数量。 */
void RmVisionTarget_SetTarget(uint16_t *flags, uint8_t target_id,
                              uint8_t armors_num);

/*============================================================================
 * 角度符号转换
 *============================================================================*/

/*
 * 协议角度遵循 ROS REP-103：pitch 抬头为正。
 * 本车 INS 发布的姿态相反：抬头为负（已实板确认，见 2026-09-11 记录）。
 *
 * 因此两个方向都需要取反。该转换只允许出现在协议边界，
 * 内部所有控制量一律保持本车 IMU 符号，避免符号约定在代码中扩散。
 */
#define RM_VISION_PITCH_SIGN (-1.0f)

/** @brief 本车 IMU pitch(deg) -> 协议线上 pitch(deg)。 */
float RmVision_PitchImuToWire(float imu_pitch_deg);

/** @brief 协议线上 pitch(deg) -> 本车 IMU pitch(deg)。 */
float RmVision_PitchWireToImu(float wire_pitch_deg);

#ifdef __cplusplus
}
#endif

#endif /* RM_VISION_PROTOCOL_H */
