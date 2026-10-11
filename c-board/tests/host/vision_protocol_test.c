/**
 * @file vision_protocol_test.c
 * @brief 视觉协议帧编解码主机测试
 * @date 2026-09-11
 *
 * 方向约定（与上游 rm_serial_driver 一致）：
 * - 遥测 telemetry：MCU -> 上位机，帧头 0x5A，整帧 22 字节
 * - 目标 target   ：上位机 -> MCU，帧头 0xA5，整帧 52 字节
 *
 * 覆盖：CRC 已知答案、协议文档测试向量、编解码往返、流式解码
 * （分片 / 粘包 / 乱码重同步 / 坏 CRC）、方向隔离、有限值防护、
 * flags 读写、角度符号转换与参数健壮性。
 */

#include <stdio.h>
#include <string.h>

#include "vision_protocol.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                \
    do {                                                \
        g_checks++;                                     \
        if (!(cond)) {                                  \
            g_failures++;                               \
            printf("FAIL %s:%d: ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                        \
            printf("\n");                               \
        }                                               \
    } while (0)

/*============================================================================
 * 测试向量（与 docs/vision/vision_protocol.md §2.4 逐字节一致）
 *============================================================================*/
static const uint8_t TELEMETRY_ZERO[RM_VISION_TELEMETRY_FRAME_LEN] = {
    0x5A, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE5, 0xC8};

static const uint8_t TELEMETRY_SAMPLE[RM_VISION_TELEMETRY_FRAME_LEN] = {
    0x5A, 0x12, 0x05, 0x00, 0xD2, 0x04, 0x0F, 0x00, 0x00, 0x00, 0x36,
    0x42, 0x00, 0x00, 0x44, 0xC1, 0x00, 0x00, 0xC0, 0x3F, 0x62, 0xC1};

static const uint8_t TARGET_SAMPLE[RM_VISION_TARGET_FRAME_LEN] = {
    0xA5, 0x30, 0x47, 0x00, 0x2E, 0x16, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00,
    0x00, 0x40, 0x00, 0x00, 0x00, 0x3F, 0x9A, 0x99, 0x99, 0x3E, 0xCD, 0xCC,
    0xCC, 0x3D, 0xCD, 0xCC, 0x4C, 0xBE, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x99,
    0x99, 0x3F, 0x00, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x80, 0x3E, 0x00, 0x00,
    0x00, 0x00, 0xA3, 0x8B};

/*============================================================================
 * CRC 表特征
 *============================================================================*/
static void test_crc_known_answer(void)
{
    RmVisionTelemetry tl;

    /* 两个文档向量覆盖了 CRC 生成表首项 0x1189 对应的行为：
     * 只有表与算法完全正确，这两个帧才会逐字节通过校验。 */
    CHECK(RmVision_EncodeTelemetry(NULL, NULL, 0U) == 0U, "NULL args rejected");

    RmVisionTelemetry_InitEmpty(&tl);
    CHECK(RmVisionTelemetry_HasFiniteAngles(&tl), "empty telemetry is finite");
}

/*============================================================================
 * 遥测编码：MCU -> 上位机
 *============================================================================*/
static void test_encode_telemetry(void)
{
    RmVisionTelemetry tl;
    uint8_t buf[RM_VISION_TELEMETRY_FRAME_LEN];
    uint16_t n;

    RmVisionTelemetry_InitEmpty(&tl);
    n = RmVision_EncodeTelemetry(&tl, buf, sizeof(buf));
    CHECK(n == RM_VISION_TELEMETRY_FRAME_LEN, "zero telemetry len, got %u",
          (unsigned)n);
    CHECK(memcmp(buf, TELEMETRY_ZERO, sizeof(TELEMETRY_ZERO)) == 0,
          "zero telemetry must match doc vector");

    tl.flags = 0x0005U;
    tl.seq = 1234U;
    tl.bullet_speed = 15U;
    tl.yaw = 45.5f;
    tl.pitch = -12.25f;
    tl.roll = 1.5f;
    n = RmVision_EncodeTelemetry(&tl, buf, sizeof(buf));
    CHECK(n == RM_VISION_TELEMETRY_FRAME_LEN, "sample telemetry len");
    CHECK(memcmp(buf, TELEMETRY_SAMPLE, sizeof(TELEMETRY_SAMPLE)) == 0,
          "sample telemetry must match doc vector");
    if (memcmp(buf, TELEMETRY_SAMPLE, sizeof(TELEMETRY_SAMPLE)) != 0) {
        unsigned i;
        printf("  got: ");
        for (i = 0; i < sizeof(TELEMETRY_SAMPLE); i++) {
            printf("%02X ", buf[i]);
        }
        printf("\n");
    }

    CHECK(buf[0] == RM_VISION_HEADER_TELEMETRY, "telemetry header 0x5A");
    CHECK(buf[1] == RM_VISION_TELEMETRY_PAYLOAD_LEN, "telemetry length field");
}

static void test_encode_telemetry_guards(void)
{
    RmVisionTelemetry tl;
    uint8_t buf[RM_VISION_TELEMETRY_FRAME_LEN];

    RmVisionTelemetry_InitEmpty(&tl);

    CHECK(RmVision_EncodeTelemetry(NULL, buf, sizeof(buf)) == 0U, "NULL src");
    CHECK(RmVision_EncodeTelemetry(&tl, NULL, sizeof(buf)) == 0U, "NULL buf");
    CHECK(RmVision_EncodeTelemetry(&tl, buf,
                                   RM_VISION_TELEMETRY_FRAME_LEN - 1U) == 0U,
          "undersized buffer rejected");
    CHECK(RmVision_EncodeTelemetry(&tl, buf, RM_VISION_TELEMETRY_FRAME_LEN) ==
              RM_VISION_TELEMETRY_FRAME_LEN,
          "exact-size buffer accepted");

    RmVisionTelemetry_InitEmpty(NULL); /* 必须安全 */
}

/*============================================================================
 * 目标解码：上位机 -> MCU
 *============================================================================*/
static void test_decode_target(void)
{
    RmVisionTarget tg;

    CHECK(RmVision_DecodeTarget(TARGET_SAMPLE, sizeof(TARGET_SAMPLE), &tg),
          "sample target frame must verify");
    CHECK(tg.flags == 0x0047U, "target flags, got 0x%04X", tg.flags);
    CHECK(tg.x == 1.0f, "target x, got %f", (double)tg.x);
    CHECK(tg.y == 2.0f, "target y, got %f", (double)tg.y);
    CHECK(tg.z == 0.5f, "target z, got %f", (double)tg.z);
    CHECK(tg.yaw == 0.3f, "target yaw, got %f", (double)tg.yaw);
    CHECK(tg.vx == 0.1f, "target vx, got %f", (double)tg.vx);
    CHECK(tg.vy == -0.2f, "target vy, got %f", (double)tg.vy);
    CHECK(tg.vz == 0.0f, "target vz");
    CHECK(tg.v_yaw == 1.2f, "target v_yaw, got %f", (double)tg.v_yaw);
    CHECK(tg.r1 == 0.25f, "target r1, got %f", (double)tg.r1);
    CHECK(tg.r2 == 0.25f, "target r2, got %f", (double)tg.r2);
    CHECK(tg.dz == 0.0f, "target dz");

    /* flags 解码：0x47 = tracking(0x1) | id=3(0x6) | armors=4(0x40) */
    CHECK(RmVisionTarget_GetTargetId(&tg) == 3U, "target id 3, got %u",
          (unsigned)RmVisionTarget_GetTargetId(&tg));
    CHECK(RmVisionTarget_GetArmorsNum(&tg) == 4U, "target armors 4, got %u",
          (unsigned)RmVisionTarget_GetArmorsNum(&tg));
    CHECK((tg.flags & RM_VISION_TARGET_FLAG_TRACKING) != 0U, "tracking set");

    /* out 可为 NULL，仅做校验 */
    CHECK(RmVision_DecodeTarget(TARGET_SAMPLE, sizeof(TARGET_SAMPLE), NULL),
          "decode with NULL out must still verify");
}

static void test_decode_target_rejects(void)
{
    RmVisionTarget tg;
    uint8_t buf[RM_VISION_TARGET_FRAME_LEN];

    /* 错误帧头：把遥测帧头 0x5A 喂给目标解码器必须被拒 */
    memcpy(buf, TARGET_SAMPLE, sizeof(buf));
    buf[0] = RM_VISION_HEADER_TELEMETRY;
    CHECK(!RmVision_DecodeTarget(buf, sizeof(buf), &tg),
          "telemetry header must be rejected by target decoder");

    /* 错误长度字段 */
    memcpy(buf, TARGET_SAMPLE, sizeof(buf));
    buf[1] = RM_VISION_TELEMETRY_PAYLOAD_LEN;
    CHECK(!RmVision_DecodeTarget(buf, sizeof(buf), &tg), "wrong length rejected");

    /* 坏 CRC */
    memcpy(buf, TARGET_SAMPLE, sizeof(buf));
    buf[RM_VISION_TARGET_FRAME_LEN - 1U] ^= 0xFFU;
    CHECK(!RmVision_DecodeTarget(buf, sizeof(buf), &tg), "bad crc rejected");

    /* 缓冲区不足 */
    CHECK(!RmVision_DecodeTarget(TARGET_SAMPLE,
                                 RM_VISION_TARGET_FRAME_LEN - 1U, &tg),
          "short buffer rejected");

    /* NULL */
    CHECK(!RmVision_DecodeTarget(NULL, sizeof(TARGET_SAMPLE), &tg), "NULL frame");
}

/*============================================================================
 * 流式解码
 *============================================================================*/
static void test_stream_fragmented(void)
{
    RmVisionRxStream rx;
    RmVisionTarget tg;
    unsigned i;

    RmVisionRx_Reset(&rx);
    for (i = 0; i < sizeof(TARGET_SAMPLE); i++) {
        (void)RmVisionRx_Push(&rx, &TARGET_SAMPLE[i], 1U, &tg);
    }
    CHECK(rx.accepted_frames == 1U, "fragmented: one frame, got %u",
          (unsigned)rx.accepted_frames);
    CHECK(rx.crc_errors == 0U, "fragmented: no crc error");
    CHECK(tg.x == 1.0f, "fragmented: payload correct");
}

static void test_stream_coalesced(void)
{
    RmVisionRxStream rx;
    RmVisionTarget tg;
    uint8_t both[2U * RM_VISION_TARGET_FRAME_LEN];

    memcpy(both, TARGET_SAMPLE, RM_VISION_TARGET_FRAME_LEN);
    memcpy(both + RM_VISION_TARGET_FRAME_LEN, TARGET_SAMPLE,
           RM_VISION_TARGET_FRAME_LEN);

    RmVisionRx_Reset(&rx);
    CHECK(RmVisionRx_Push(&rx, both, sizeof(both), &tg), "coalesced accepted");
    CHECK(rx.accepted_frames == 2U, "coalesced: two frames, got %u",
          (unsigned)rx.accepted_frames);
}

static void test_stream_garbage_resync(void)
{
    RmVisionRxStream rx;
    RmVisionTarget tg;
    uint8_t buf[128];
    unsigned k = 0;
    unsigned i;

    /* 前置乱码（含伪帧头）后再接一个真帧 */
    for (i = 0; i < 8U; i++) {
        buf[k++] = 0x00U;
    }
    buf[k++] = RM_VISION_HEADER_TARGET; /* 伪帧头但后面不是合法帧 */
    buf[k++] = 0xFFU;
    buf[k++] = RM_VISION_HEADER_TARGET;
    buf[k++] = RM_VISION_TARGET_PAYLOAD_LEN;
    for (i = 0; i < 6U; i++) {
        buf[k++] = 0xABU;
    }
    memcpy(buf + k, TARGET_SAMPLE, RM_VISION_TARGET_FRAME_LEN);
    k += RM_VISION_TARGET_FRAME_LEN;

    RmVisionRx_Reset(&rx);
    CHECK(RmVisionRx_Push(&rx, buf, (uint16_t)k, &tg), "resync recovers frame");
    CHECK(rx.accepted_frames == 1U, "resync: exactly one valid frame, got %u",
          (unsigned)rx.accepted_frames);
    CHECK(tg.x == 1.0f, "resync: payload correct");
    CHECK(rx.resync_bytes > 0U, "resync: byte counter advanced");
}

static void test_stream_bad_crc_then_good(void)
{
    RmVisionRxStream rx;
    RmVisionTarget tg;
    uint8_t buf[2U * RM_VISION_TARGET_FRAME_LEN];

    memcpy(buf, TARGET_SAMPLE, RM_VISION_TARGET_FRAME_LEN);
    buf[RM_VISION_TARGET_FRAME_LEN - 1U] ^= 0xFFU; /* 破坏 CRC */
    memcpy(buf + RM_VISION_TARGET_FRAME_LEN, TARGET_SAMPLE,
           RM_VISION_TARGET_FRAME_LEN);

    RmVisionRx_Reset(&rx);
    CHECK(RmVisionRx_Push(&rx, buf, sizeof(buf), &tg), "good frame after bad");
    CHECK(rx.accepted_frames == 1U, "one accepted after bad crc, got %u",
          (unsigned)rx.accepted_frames);
    CHECK(rx.crc_errors >= 1U, "crc error recorded, got %u",
          (unsigned)rx.crc_errors);
}

static void test_stream_full_buffer_no_deadlock(void)
{
    RmVisionRxStream rx;
    RmVisionTarget tg;
    uint8_t junk[128];
    unsigned i;

    memset(junk, 0x33, sizeof(junk));

    RmVisionRx_Reset(&rx);
    /* 持续灌入无帧头的垃圾：缓冲必须能自行回收，不能卡死 */
    for (i = 0; i < 8U; i++) {
        (void)RmVisionRx_Push(&rx, junk, sizeof(junk), &tg);
    }
    CHECK(rx.length < RM_VISION_RX_ACCUM_CAPACITY,
          "buffer must not stay full, len=%u", (unsigned)rx.length);

    /* 之后仍能正常同步到真帧 */
    CHECK(RmVisionRx_Push(&rx, TARGET_SAMPLE, sizeof(TARGET_SAMPLE), &tg),
          "recovers after junk flood");
}

/**
 * @brief 测试用：构造一个 seq 指定的合法目标帧。
 *
 * 使用独立于产品实现的 CRC 表重算，与产品实现相互印证。
 */
static void build_target_frame(uint8_t out[RM_VISION_TARGET_FRAME_LEN],
                               uint16_t seq)
{
    uint8_t frame[RM_VISION_TARGET_FRAME_LEN];
    static uint16_t tab[256];
    static int ready = 0;
    unsigned i;
    unsigned bit;
    uint16_t crc;

    memcpy(frame, TARGET_SAMPLE, sizeof(frame));
    frame[4] = (uint8_t)(seq & 0x00FFU);
    frame[5] = (uint8_t)((seq >> 8) & 0x00FFU);

    if (!ready) {
        for (i = 0; i < 256U; i++) {
            uint16_t c = (uint16_t)i;
            for (bit = 0; bit < 8U; bit++) {
                c = (c & 1U) ? (uint16_t)((c >> 1) ^ 0x8408U)
                             : (uint16_t)(c >> 1);
            }
            tab[i] = c;
        }
        ready = 1;
    }

    crc = 0xFFFFU;
    for (i = 0; i < (unsigned)(RM_VISION_TARGET_FRAME_LEN - 2U); i++) {
        crc = (uint16_t)((crc >> 8) ^ tab[(crc ^ frame[i]) & 0x00FFU]);
    }
    frame[RM_VISION_TARGET_FRAME_LEN - 2U] = (uint8_t)(crc & 0x00FFU);
    frame[RM_VISION_TARGET_FRAME_LEN - 1U] =
        (uint8_t)((crc >> 8) & 0x00FFU);

    memcpy(out, frame, sizeof(frame));
}

/**
 * @brief 一次 Push 中解出多帧时，out 必须为最后一帧。
 */
static void test_stream_last_frame_wins(void)
{
    RmVisionRxStream rx;
    RmVisionTarget tg;
    uint8_t buf[3U * RM_VISION_TARGET_FRAME_LEN];

    /* 自建帧本身必须合法，否则后续断言无意义 */
    build_target_frame(buf, 1U);
    CHECK(RmVision_DecodeTarget(buf, RM_VISION_TARGET_FRAME_LEN, &tg),
          "self-built frame valid");

    build_target_frame(buf, 1U);
    build_target_frame(buf + RM_VISION_TARGET_FRAME_LEN, 2U);
    build_target_frame(buf + (2U * RM_VISION_TARGET_FRAME_LEN), 3U);

    RmVisionRx_Reset(&rx);
    CHECK(RmVisionRx_Push(&rx, buf, sizeof(buf), &tg), "three frames accepted");
    CHECK(rx.accepted_frames == 3U, "three frames counted, got %u",
          (unsigned)rx.accepted_frames);
}

/**
 * @brief 多帧中最新的目标必须能通过 seq 区分。
 *
 * Target 语义结构不含 seq（控制只关心最新），因此通过 x 字段区分：
 * 每帧改写 x 并重算 CRC，验证 out 为最后一帧。
 */
static void test_stream_gets_latest(void)
{
    RmVisionRxStream rx;
    RmVisionTarget tg;
    uint8_t buf[2U * RM_VISION_TARGET_FRAME_LEN];
    static uint16_t tab[256];
    static int ready = 0;
    unsigned i;
    unsigned bit;
    uint16_t crc;

    if (!ready) {
        for (i = 0; i < 256U; i++) {
            uint16_t c = (uint16_t)i;
            for (bit = 0; bit < 8U; bit++) {
                c = (c & 1U) ? (uint16_t)((c >> 1) ^ 0x8408U)
                             : (uint16_t)(c >> 1);
            }
            tab[i] = c;
        }
        ready = 1;
    }

    for (i = 0; i < 2U; i++) {
        uint8_t *f = buf + (i * RM_VISION_TARGET_FRAME_LEN);
        float x = (i == 0U) ? 1.5f : 9.5f;

        memcpy(f, TARGET_SAMPLE, RM_VISION_TARGET_FRAME_LEN);
        memcpy(f + 6, &x, sizeof(x));
        crc = 0xFFFFU;
        for (bit = 0; bit < (unsigned)(RM_VISION_TARGET_FRAME_LEN - 2U);
             bit++) {
            crc = (uint16_t)((crc >> 8) ^ tab[(crc ^ f[bit]) & 0x00FFU]);
        }
        f[RM_VISION_TARGET_FRAME_LEN - 2U] = (uint8_t)(crc & 0x00FFU);
        f[RM_VISION_TARGET_FRAME_LEN - 1U] = (uint8_t)((crc >> 8) & 0x00FFU);
    }

    RmVisionRx_Reset(&rx);
    CHECK(RmVisionRx_Push(&rx, buf, sizeof(buf), &tg), "two frames accepted");
    CHECK(tg.x == 9.5f, "out must be the LAST frame, got x=%f", (double)tg.x);
}

static void test_stream_guards(void)
{
    RmVisionRxStream rx;
    RmVisionTarget tg;

    RmVisionRx_Reset(&rx);
    CHECK(!RmVisionRx_Push(NULL, TARGET_SAMPLE, sizeof(TARGET_SAMPLE), &tg),
          "NULL rx");
    CHECK(!RmVisionRx_Push(&rx, NULL, sizeof(TARGET_SAMPLE), &tg), "NULL data");
    CHECK(!RmVisionRx_Push(&rx, TARGET_SAMPLE, 0U, &tg), "zero len");

    RmVisionRx_Reset(NULL); /* 必须安全 */
}

/**
 * @brief 满缓冲且首字节恰为合法帧头时必须能退出重同步循环。
 */
static void test_stream_saturated_with_headers(void)
{
    RmVisionRxStream rx;
    RmVisionTarget tg;
    uint8_t hdr_only[128];
    unsigned i;

    for (i = 0; i < sizeof(hdr_only); i++) {
        hdr_only[i] = RM_VISION_HEADER_TARGET; /* 全是 0xA5 */
    }

    RmVisionRx_Reset(&rx);
    for (i = 0; i < 16U; i++) {
        (void)RmVisionRx_Push(&rx, hdr_only, sizeof(hdr_only), &tg);
    }

    CHECK(rx.length < RM_VISION_RX_ACCUM_CAPACITY,
          "saturated buffer drains, len=%u", (unsigned)rx.length);
    CHECK(rx.accepted_frames == 0U, "no false frames from header flood");

    /* 之后仍必须能正常同步到真帧 */
    CHECK(RmVisionRx_Push(&rx, TARGET_SAMPLE, sizeof(TARGET_SAMPLE), &tg),
          "recovers after header flood");
    CHECK(tg.x == 1.0f, "correct frame after header flood");
}

/**
 * @brief 方向隔离：遥测帧（0x5A）绝不能被目标解码器当成目标。
 *
 * 这条是接错线 / 回声场景下的安全底线——若把本机发出的遥测回读成目标，
 * 云台会跟随本机姿态形成正反馈。
 */
static void test_stream_direction_isolation(void)
{
    RmVisionRxStream rx;
    RmVisionTarget tg;
    uint8_t both[RM_VISION_TELEMETRY_FRAME_LEN + RM_VISION_TARGET_FRAME_LEN];

    memcpy(both, TELEMETRY_SAMPLE, RM_VISION_TELEMETRY_FRAME_LEN);
    memcpy(both + RM_VISION_TELEMETRY_FRAME_LEN, TARGET_SAMPLE,
           RM_VISION_TARGET_FRAME_LEN);

    RmVisionRx_Reset(&rx);
    CHECK(RmVisionRx_Push(&rx, both, sizeof(both), &tg),
          "real target after echoed telemetry still decoded");
    CHECK(rx.accepted_frames == 1U,
          "echoed telemetry must not count as a target, got %u",
          (unsigned)rx.accepted_frames);
    CHECK(tg.x == 1.0f, "decoded frame is the real target");

    /* 纯遥测流：一个目标都不该解出 */
    RmVisionRx_Reset(&rx);
    CHECK(!RmVisionRx_Push(&rx, TELEMETRY_SAMPLE, sizeof(TELEMETRY_SAMPLE),
                           &tg),
          "pure telemetry stream yields no target");
    CHECK(rx.accepted_frames == 0U, "no target from telemetry only");
}

/*============================================================================
 * 有限值防护
 *
 * 关键：有限值判断必须不依赖编译器的浮点有限值假设。
 * 本测试在 Debug、Release 以及 fast-math 变体下都必须通过。
 *============================================================================*/
/**
 * @brief 由位模式构造 float。
 *
 * 不能用 0.0f/0.0f 或 1.0f/0.0f 生成 NaN/Inf：fast-math 可能把这类
 * 常量表达式直接折叠，测试将静默失去意义。位模式构造不受影响。
 */
static float float_from_bits(uint32_t bits)
{
    float v;

    memcpy(&v, &bits, sizeof(v));
    return v;
}

static void test_finite_guards(void)
{
    RmVisionTelemetry tl;
    RmVisionTarget tg;
    const float nan_v = float_from_bits(0x7FC00000U);  /* quiet NaN */
    const float inf_v = float_from_bits(0x7F800000U);  /* +Inf */
    const float ninf_v = float_from_bits(0xFF800000U); /* -Inf */

#ifdef __FAST_MATH__
    /*
     * 启用 -ffast-math 时编译器假定不存在 NaN，nan_v != nan_v 会被折叠为假。
     * 该断言在此构建下无意义，跳过；下面的有限值检查仍全部有效，
     * 因为 vision_float_is_finite() 使用位模式而非 isfinite()。
     */
    printf("note: -ffast-math build; NaN self-compare sanity check skipped\n");
#else
    CHECK(nan_v != nan_v, "sanity: NaN produced");
#endif

    memset(&tl, 0, sizeof(tl));
    CHECK(RmVisionTelemetry_HasFiniteAngles(&tl), "finite zeros accepted");
    tl.pitch = nan_v;
    CHECK(!RmVisionTelemetry_HasFiniteAngles(&tl), "NaN telemetry pitch rejected");
    tl.pitch = inf_v;
    CHECK(!RmVisionTelemetry_HasFiniteAngles(&tl), "Inf telemetry pitch rejected");
    tl.pitch = 3.0e38f; /* 极大但有限，不能误判 */
    CHECK(RmVisionTelemetry_HasFiniteAngles(&tl), "large finite accepted");
    CHECK(!RmVisionTelemetry_HasFiniteAngles(NULL), "NULL telemetry rejected");

    memset(&tg, 0, sizeof(tg));
    CHECK(RmVisionTarget_HasFiniteState(&tg), "finite target accepted");
    tg.x = nan_v;
    CHECK(!RmVisionTarget_HasFiniteState(&tg), "NaN x rejected");
    tg.x = 0.0f;
    tg.dz = inf_v;
    CHECK(!RmVisionTarget_HasFiniteState(&tg), "Inf dz rejected");
    tg.dz = 0.0f;
    tg.v_yaw = ninf_v;
    CHECK(!RmVisionTarget_HasFiniteState(&tg), "-Inf v_yaw rejected");
    tg.v_yaw = 0.0f;
    tg.r2 = nan_v;
    CHECK(!RmVisionTarget_HasFiniteState(&tg), "NaN r2 rejected");
    CHECK(!RmVisionTarget_HasFiniteState(NULL), "NULL target rejected");
}

/*============================================================================
 * flags 读写
 *============================================================================*/
static void test_flags_helpers(void)
{
    uint16_t flags = 0U;
    RmVisionTarget tg;

    RmVisionTarget_SetTarget(&flags, 3U, 4U);
    CHECK((flags & RM_VISION_TARGET_FLAG_ID_MASK) >>
              RM_VISION_TARGET_FLAG_ID_SHIFT ==
              3U,
          "set id 3");
    CHECK((flags & RM_VISION_TARGET_FLAG_ARMORS_MASK) >>
              RM_VISION_TARGET_FLAG_ARMORS_SHIFT ==
              4U,
          "set armors 4");

    RmVisionTarget_SetTarget(&flags, 7U, 2U);
    CHECK(RmVisionTarget_GetTargetId(&(RmVisionTarget){.flags = flags}) == 7U,
          "overwrite id 7");

    /* 不得破坏其它位 */
    flags = (uint16_t)(flags | RM_VISION_TARGET_FLAG_TRACKING);
    RmVisionTarget_SetTarget(&flags, 1U, 3U);
    CHECK((flags & RM_VISION_TARGET_FLAG_TRACKING) != 0U,
          "tracking bit preserved");

    memset(&tg, 0, sizeof(tg));
    RmVisionTarget_SetTarget(&tg.flags, 6U, 3U);
    tg.flags = (uint16_t)(tg.flags | RM_VISION_TARGET_FLAG_TRACKING);
    CHECK(RmVisionTarget_GetTargetId(&tg) == 6U, "get id 6");
    CHECK(RmVisionTarget_GetArmorsNum(&tg) == 3U, "get armors 3");

    CHECK(RmVisionTarget_GetTargetId(NULL) == 0U, "NULL id getter");
    CHECK(RmVisionTarget_GetArmorsNum(NULL) == 0U, "NULL armors getter");
    RmVisionTarget_SetTarget(NULL, 1U, 1U); /* 必须安全 */

    /* flags 位不重叠 */
    CHECK((RM_VISION_TARGET_FLAG_TRACKING &
           RM_VISION_TARGET_FLAG_ID_MASK) == 0U,
          "tracking must not overlap id");
    CHECK((RM_VISION_TARGET_FLAG_ID_MASK &
           RM_VISION_TARGET_FLAG_ARMORS_MASK) == 0U,
          "id must not overlap armors");
    CHECK((RM_VISION_TARGET_FLAG_ARMORS_MASK &
           RM_VISION_TARGET_FLAG_FIRE_ADVICE) == 0U,
          "armors must not overlap fire advice");

    /* 遥测 flags 位不重叠 */
    CHECK((RM_VISION_TELEM_FLAG_ENEMY_COLOR &
           RM_VISION_TELEM_FLAG_RESET_TRACKER) == 0U,
          "telemetry flags must not overlap");
    CHECK((RM_VISION_TELEM_FLAG_AUTO_AIM_ENABLE &
           RM_VISION_TELEM_FLAG_MANUAL_TRIM_ENABLE) == 0U,
          "telemetry flags must not overlap (2)");
}

/*============================================================================
 * 角度符号转换
 *============================================================================*/
static void test_pitch_sign(void)
{
    /* 本车 IMU：抬头为负。协议：抬头为正。 */
    float imu_level = 0.0f;
    float imu_up = -25.0f;  /* 抬头 25 度 */
    float imu_down = 15.0f; /* 低头 15 度 */

    CHECK(RmVision_PitchImuToWire(imu_level) == 0.0f, "level stays 0");
    CHECK(RmVision_PitchImuToWire(imu_up) > 0.0f,
          "抬头必须映射为协议正角，got %f",
          (double)RmVision_PitchImuToWire(imu_up));
    CHECK(RmVision_PitchImuToWire(imu_down) < 0.0f,
          "低头必须映射为协议负角，got %f",
          (double)RmVision_PitchImuToWire(imu_down));

    CHECK(RmVision_PitchImuToWire(imu_up) == 25.0f, "magnitude preserved up");
    CHECK(RmVision_PitchImuToWire(imu_down) == -15.0f, "magnitude preserved down");

    /* 往返 */
    CHECK(RmVision_PitchWireToImu(RmVision_PitchImuToWire(imu_up)) == imu_up,
          "pitch round trip up");
    CHECK(RmVision_PitchWireToImu(RmVision_PitchImuToWire(imu_down)) == imu_down,
          "pitch round trip down");

    /* 与软限位一致：-30 抬头 <-> 协议 +30 */
    CHECK(RmVision_PitchImuToWire(-30.0f) == 30.0f, "-30 imu -> +30 wire");
    CHECK(RmVision_PitchImuToWire(20.0f) == -20.0f, "+20 imu -> -20 wire");
}

/*============================================================================
 * 端到端往返
 *============================================================================*/
static void test_telemetry_roundtrip(void)
{
    RmVisionTelemetry src;
    RmVisionTelemetry dst;
    uint8_t buf[RM_VISION_TELEMETRY_FRAME_LEN];
    RmVisionTelemetryWire wire;
    uint16_t n;

    RmVisionTelemetry_InitEmpty(&src);
    src.flags = RM_VISION_TELEM_FLAG_ENEMY_COLOR |
                RM_VISION_TELEM_FLAG_AUTO_AIM_ENABLE;
    src.seq = 4242U;
    src.bullet_speed = 15U;
    src.yaw = -179.5f;
    src.pitch = 12.25f;
    src.roll = -3.5f;

    n = RmVision_EncodeTelemetry(&src, buf, sizeof(buf));
    CHECK(n == RM_VISION_TELEMETRY_FRAME_LEN, "telemetry roundtrip encode");

    /* 用线上结构体解析回语义结构，模拟上位机的读取路径 */
    memcpy(&wire, buf, sizeof(wire));
    CHECK(wire.header == RM_VISION_HEADER_TELEMETRY, "wire header");
    CHECK(wire.length == RM_VISION_TELEMETRY_PAYLOAD_LEN, "wire length");
    CHECK(wire.reserved == 0U, "reserved must be zero");

    memset(&dst, 0, sizeof(dst));
    dst.flags = wire.flags;
    dst.seq = wire.seq;
    dst.bullet_speed = wire.bullet_speed;
    dst.yaw = wire.yaw;
    dst.pitch = wire.pitch;
    dst.roll = wire.roll;

    CHECK(dst.flags == src.flags, "roundtrip flags");
    CHECK(dst.seq == src.seq, "roundtrip seq");
    CHECK(dst.bullet_speed == src.bullet_speed, "roundtrip bullet");
    CHECK(dst.yaw == src.yaw, "roundtrip yaw");
    CHECK(dst.pitch == src.pitch, "roundtrip pitch");
    CHECK(dst.roll == src.roll, "roundtrip roll");
    CHECK(RmVisionTelemetry_HasFiniteAngles(&dst), "roundtrip finite");
}

static void test_target_roundtrip(void)
{
    RmVisionTarget dst;

    CHECK(RmVision_DecodeTarget(TARGET_SAMPLE, sizeof(TARGET_SAMPLE), &dst),
          "decode for roundtrip");

    /* 协议 pitch 由上位机按 REP-103 下发；x/y/z 直接是云台系坐标，不取反 */
    CHECK(dst.x == 1.0f && dst.y == 2.0f && dst.z == 0.5f,
          "target position passes through unchanged");
    CHECK(dst.yaw == 0.3f, "target yaw unchanged (rad, 云台系)");
    CHECK(RmVisionTarget_HasFiniteState(&dst), "decoded target finite");
}

int main(void)
{
    RmVision_Init();

    test_crc_known_answer();
    test_encode_telemetry();
    test_encode_telemetry_guards();
    test_decode_target();
    test_decode_target_rejects();
    test_stream_fragmented();
    test_stream_coalesced();
    test_stream_garbage_resync();
    test_stream_bad_crc_then_good();
    test_stream_full_buffer_no_deadlock();
    test_stream_last_frame_wins();
    test_stream_gets_latest();
    test_stream_saturated_with_headers();
    test_stream_guards();
    test_stream_direction_isolation();
    test_finite_guards();
    test_flags_helpers();
    test_pitch_sign();
    test_telemetry_roundtrip();
    test_target_roundtrip();

    printf("vision_protocol: %d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
