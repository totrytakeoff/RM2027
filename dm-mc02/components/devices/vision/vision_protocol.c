/**
 * @file vision_protocol.c
 * @brief 视觉上下位机通讯协议 —— 帧编解码实现
 * @date 2026-09-11
 * @note 协议基线见 docs/vision/vision_protocol.md
 *
 * 本文件是纯逻辑实现：不访问 HAL、FreeRTOS、DWT 或堆。
 * 全部状态由调用方持有，CRC 表在初始化时一次性构建于固定静态存储。
 */

#include "vision_protocol.h"

#include <string.h>

/*============================================================================
 * CRC-16/MCRF4XX
 *
 * 多项式 0x8408（反射形式，对应 0x1021），初值 0xFFFF，不取反。
 * 与上位机 rm_serial_driver/src/crc.cpp 的硬编码表逐字节一致，
 * 也与本仓库 algorithms/checksum/crc16.c 的 crc_16() 行为一致。
 * 两侧均无需修改既有 CRC 实现。
 *============================================================================*/
#define RM_VISION_CRC16_INIT 0xFFFFU
#define RM_VISION_CRC16_POLY 0x8408U

static uint16_t vision_crc16_table[256];
static bool vision_crc16_table_ready = false;

void RmVision_Init(void)
{
    uint16_t i;

    if (vision_crc16_table_ready) {
        return;
    }

    /*
     * 表构建方式是反射算法（0x8408）的规范形式，生成表首项为 0x1189，
     * 与上位机 rm_serial_driver/src/crc.cpp 的硬编码表逐字节一致。
     */
    for (i = 0; i < 256U; i++) {
        uint16_t crc = i;
        uint8_t bit;

        for (bit = 0; bit < 8U; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1) ^ RM_VISION_CRC16_POLY);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
        vision_crc16_table[i] = crc;
    }

    vision_crc16_table_ready = true;
}

/** 计算 data[0..len) 的 CRC-16/MCRF4XX。 */
static uint16_t vision_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = RM_VISION_CRC16_INIT;
    uint16_t i;

    for (i = 0; i < len; i++) {
        crc = (uint16_t)((crc >> 8) ^
                         vision_crc16_table[(crc ^ (uint16_t)data[i]) & 0x00FFU]);
    }
    return crc;
}

/** 为已填好除 crc16 外全部字段的帧写入小端校验值。 */
static void vision_frame_seal(uint8_t *frame, uint16_t frame_len)
{
    uint16_t crc = vision_crc16(frame, (uint16_t)(frame_len - 2U));

    frame[frame_len - 2U] = (uint8_t)(crc & 0x00FFU);
    frame[frame_len - 1U] = (uint8_t)((crc >> 8) & 0x00FFU);
}

/** 校验帧尾小端 CRC。 */
static bool vision_frame_verify(const uint8_t *frame, uint16_t frame_len)
{
    uint16_t expected;
    uint16_t stored;

    if (frame_len < 3U) {
        return false;
    }

    expected = vision_crc16(frame, (uint16_t)(frame_len - 2U));
    stored = (uint16_t)((uint16_t)frame[frame_len - 2U] |
                        ((uint16_t)frame[frame_len - 1U] << 8));

    return expected == stored;
}

/*============================================================================
 * 有限值检查
 *============================================================================*/

/**
 * @brief 判定 float 是否为有限值。
 *
 * 不依赖 isfinite() 的编译器浮点假设：即使构建选项改变，NaN/Inf 防护仍
 * 必须有效。此处改用位模式检查，IEEE-754 中指数域全 1 即为 NaN 或 Inf，
 * 纯整数运算不受快速数学影响。
 */
static bool vision_float_is_finite(float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));

    return (bits & 0x7F800000U) != 0x7F800000U;
}

static bool vision_all_finite(const float *values, uint8_t count)
{
    uint8_t i;

    for (i = 0; i < count; i++) {
        if (!vision_float_is_finite(values[i])) {
            return false;
        }
    }
    return true;
}

/*============================================================================
 * 初始化 / 复位
 *============================================================================*/

void RmVisionRx_Reset(RmVisionRxStream *rx)
{
    if (rx == NULL) {
        return;
    }

    memset(rx->storage, 0, sizeof(rx->storage));
    rx->length = 0U;
    rx->accepted_frames = 0U;
    rx->crc_errors = 0U;
    rx->format_errors = 0U;
    rx->resync_bytes = 0U;
}

/*============================================================================
 * 编码：MCU -> 上位机
 *============================================================================*/

void RmVisionTelemetry_InitEmpty(RmVisionTelemetry *pkt)
{
    if (pkt == NULL) {
        return;
    }

    memset(pkt, 0, sizeof(*pkt));
}

uint16_t RmVision_EncodeTelemetry(const RmVisionTelemetry *src,
                                  uint8_t *buf,
                                  uint16_t buf_len)
{
    RmVisionTelemetryWire wire;

    if (src == NULL || buf == NULL ||
        buf_len < RM_VISION_TELEMETRY_FRAME_LEN) {
        return 0U;
    }

    memset(&wire, 0, sizeof(wire));
    wire.header = RM_VISION_HEADER_TELEMETRY;
    wire.length = RM_VISION_TELEMETRY_PAYLOAD_LEN;
    wire.flags = src->flags;
    wire.seq = src->seq;
    wire.bullet_speed = src->bullet_speed;
    wire.reserved = 0U;
    wire.yaw = src->yaw;
    wire.pitch = src->pitch;
    wire.roll = src->roll;

    memcpy(buf, &wire, sizeof(wire));
    vision_frame_seal(buf, RM_VISION_TELEMETRY_FRAME_LEN);

    return RM_VISION_TELEMETRY_FRAME_LEN;
}

/*============================================================================
 * 解码：上位机 -> MCU
 *============================================================================*/

bool RmVision_DecodeTarget(const uint8_t *frame,
                           uint16_t frame_len,
                           RmVisionTarget *out)
{
    RmVisionTargetWire wire;

    if (frame == NULL || frame_len < RM_VISION_TARGET_FRAME_LEN) {
        return false;
    }
    if (frame[0] != RM_VISION_HEADER_TARGET) {
        return false;
    }
    if (frame[1] != RM_VISION_TARGET_PAYLOAD_LEN) {
        return false;
    }
    if (!vision_frame_verify(frame, RM_VISION_TARGET_FRAME_LEN)) {
        return false;
    }

    memcpy(&wire, frame, sizeof(wire));

    if (out != NULL) {
        out->flags = wire.flags;
        out->x = wire.x;
        out->y = wire.y;
        out->z = wire.z;
        out->yaw = wire.yaw;
        out->vx = wire.vx;
        out->vy = wire.vy;
        out->vz = wire.vz;
        out->v_yaw = wire.v_yaw;
        out->r1 = wire.r1;
        out->r2 = wire.r2;
        out->dz = wire.dz;
    }

    return true;
}

/**
 * @brief 在累积缓冲中丢弃前面的 n 字节。
 */
static void vision_rx_discard(RmVisionRxStream *rx, uint16_t n)
{
    if (n >= rx->length) {
        rx->length = 0U;
        return;
    }

    memmove(rx->storage, rx->storage + n, (size_t)(rx->length - n));
    rx->length = (uint16_t)(rx->length - n);
}

bool RmVisionRx_Push(RmVisionRxStream *rx,
                     const uint8_t *data,
                     uint16_t len,
                     RmVisionTarget *out)
{
    bool got_frame = false;

    if (rx == NULL || data == NULL || len == 0U) {
        return false;
    }

    while (len > 0U) {
        uint16_t space = (uint16_t)(RM_VISION_RX_ACCUM_CAPACITY - rx->length);
        uint16_t chunk = (len < space) ? len : space;

        if (chunk == 0U) {
            /*
             * 缓冲已满仍未凑出有效帧：说明当前帧头是伪同步。
             * 丢弃最早一字节并重新同步，避免永久卡死。
             */
            vision_rx_discard(rx, 1U);
            rx->resync_bytes++;
            continue;
        }

        memcpy(rx->storage + rx->length, data, (size_t)chunk);
        rx->length = (uint16_t)(rx->length + chunk);
        data += chunk;
        len = (uint16_t)(len - chunk);

        /* 尽可能多地解出已完整到达的帧（处理粘包）。 */
        for (;;) {
            if (rx->length < 2U) {
                break;
            }

            if (rx->storage[0] != RM_VISION_HEADER_TARGET) {
                vision_rx_discard(rx, 1U);
                rx->resync_bytes++;
                continue;
            }

            if (rx->storage[1] != RM_VISION_TARGET_PAYLOAD_LEN) {
                /* 帧头对但长度不对：视作伪头，只丢一字节再找。 */
                vision_rx_discard(rx, 1U);
                rx->format_errors++;
                continue;
            }

            if (rx->length < RM_VISION_TARGET_FRAME_LEN) {
                break; /* 帧未收全，等待后续字节 */
            }

            if (vision_frame_verify(rx->storage, RM_VISION_TARGET_FRAME_LEN)) {
                if (RmVision_DecodeTarget(rx->storage,
                                          RM_VISION_TARGET_FRAME_LEN,
                                          out)) {
                    rx->accepted_frames++;
                    got_frame = true;
                    vision_rx_discard(rx, RM_VISION_TARGET_FRAME_LEN);
                    continue;
                }
            }

            /*
             * 校验失败：只丢弃一字节而不是整帧。
             * 若此处是载荷内部恰好出现的伪帧头，整帧丢弃会连带吞掉紧随其后的
             * 真实帧起始。缓冲仅 64 字节，逐字节滑动重同步的代价可以接受。
             */
            rx->crc_errors++;
            rx->resync_bytes++;
            vision_rx_discard(rx, 1U);
        }
    }

    return got_frame;
}

/*============================================================================
 * 语义辅助
 *============================================================================*/

bool RmVisionTelemetry_HasFiniteAngles(const RmVisionTelemetry *pkt)
{
    if (pkt == NULL) {
        return false;
    }

    return vision_all_finite(&pkt->yaw, 3U);
}

bool RmVisionTarget_HasFiniteState(const RmVisionTarget *pkt)
{
    if (pkt == NULL) {
        return false;
    }

    return vision_all_finite(&pkt->x, 11U);
}

uint8_t RmVisionTarget_GetTargetId(const RmVisionTarget *pkt)
{
    if (pkt == NULL) {
        return 0U;
    }

    return (uint8_t)((pkt->flags & RM_VISION_TARGET_FLAG_ID_MASK) >>
                     RM_VISION_TARGET_FLAG_ID_SHIFT);
}

uint8_t RmVisionTarget_GetArmorsNum(const RmVisionTarget *pkt)
{
    if (pkt == NULL) {
        return 0U;
    }

    return (uint8_t)((pkt->flags & RM_VISION_TARGET_FLAG_ARMORS_MASK) >>
                     RM_VISION_TARGET_FLAG_ARMORS_SHIFT);
}

void RmVisionTarget_SetTarget(uint16_t *flags, uint8_t target_id,
                              uint8_t armors_num)
{
    if (flags == NULL) {
        return;
    }

    *flags = (uint16_t)(*flags & ~RM_VISION_TARGET_FLAG_ID_MASK);
    *flags = (uint16_t)(*flags & ~RM_VISION_TARGET_FLAG_ARMORS_MASK);
    *flags = (uint16_t)(
        *flags |
        ((uint16_t)(target_id & 0x7U) << RM_VISION_TARGET_FLAG_ID_SHIFT));
    *flags = (uint16_t)(
        *flags |
        ((uint16_t)(armors_num & 0x7U) << RM_VISION_TARGET_FLAG_ARMORS_SHIFT));
}

/*============================================================================
 * 角度符号转换
 *============================================================================*/

float RmVision_PitchImuToWire(float imu_pitch_deg)
{
    return RM_VISION_PITCH_SIGN * imu_pitch_deg;
}

float RmVision_PitchWireToImu(float wire_pitch_deg)
{
    return RM_VISION_PITCH_SIGN * wire_pitch_deg;
}
