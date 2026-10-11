/**
 * @file vision_aim_test.c
 * @brief 视觉自瞄角度解算主机测试
 * @date 2026-09-11
 *
 * 覆盖 gimbal_units.h 中的方位/仰角换算，重点是**符号约定**与
 * **云台系相对方位**这两个最容易出错、且出错就"打反方向"的环节。
 * 对应 docs/vision/vision_protocol.md §1.3.1 与 §5.2。
 */

#include <math.h>
#include <stdio.h>

#include "gimbal_units.h"

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

/** 浮点近似比较。 */
static int near(float a, float b, float tol)
{
    return fabsf(a - b) <= tol;
}

static float float_from_bits(unsigned bits)
{
    float v;

    /* 用 memcpy 避免 -ffast-math 下的常量折叠 */
    const unsigned pattern = bits;
    __builtin_memcpy(&v, &pattern, sizeof(v));
    return v;
}

/*============================================================================
 * 单位换算
 *============================================================================*/
static void test_unit_conversion(void)
{
    CHECK(near(Gimbal_RadToDeg(0.0f), 0.0f, 1e-6f), "0 rad -> 0 deg");
    CHECK(near(Gimbal_RadToDeg(1.5707963f), 90.0f, 1e-3f),
          "pi/2 -> 90 deg, got %f", (double)Gimbal_RadToDeg(1.5707963f));
    CHECK(near(Gimbal_RadToDeg(-1.5707963f), -90.0f, 1e-3f),
          "-pi/2 -> -90 deg");
}

/*============================================================================
 * Yaw：云台系相对方位 -> IMU 累计角参考
 *
 * 铁律：yaw_ref = yaw_now + atan2(y, x)。
 * 目标正前方 -> 增量 0；目标在左 -> 增量正；目标在右 -> 增量负。
 *============================================================================*/
static void test_yaw_reference(void)
{
    const float yaw_now = 100.0f;

    /* 正前方：方位角 0，不得改变参考 */
    CHECK(near(Gimbal_TargetYawReferenceDeg(yaw_now, 5.0f, 0.0f), 100.0f, 1e-4f),
          "target straight ahead -> no yaw change, got %f",
          (double)Gimbal_TargetYawReferenceDeg(yaw_now, 5.0f, 0.0f));

    /* 目标在左（+Y）：俯视逆时针，增量应 > 0 */
    {
        float r = Gimbal_TargetYawReferenceDeg(yaw_now, 1.0f, 1.0f);
        CHECK(near(r, 145.0f, 1e-3f),
              "target 45deg-left (1,1) -> yaw_now+45, got %f", (double)r);
    }
    /* 目标在右（-Y）：增量应 < 0 */
    {
        float r = Gimbal_TargetYawReferenceDeg(yaw_now, 1.0f, -1.0f);
        CHECK(near(r, 55.0f, 1e-3f),
              "target 45deg-right (1,-1) -> yaw_now-45, got %f", (double)r);
    }
    /* 正后方：180，方向不明确但量值必须对 */
    {
        float r = Gimbal_TargetYawReferenceDeg(yaw_now, -3.0f, 0.0f);
        CHECK(near(fabsf(r - yaw_now), 180.0f, 1e-3f),
              "target behind -> +/-180, got %f", (double)r);
    }

    /* 退化输入：保持当前指向 */
    CHECK(Gimbal_TargetYawReferenceDeg(yaw_now, 0.0f, 0.0f) == yaw_now,
          "zero vector -> hold");
    CHECK(Gimbal_TargetYawReferenceDeg(yaw_now,
                                       float_from_bits(0x7FC00000U), 1.0f) ==
              yaw_now,
          "NaN x -> hold");
    CHECK(Gimbal_TargetYawReferenceDeg(yaw_now, 1.0f,
                                       float_from_bits(0x7F800000U)) == yaw_now,
          "Inf y -> hold");
}

/*============================================================================
 * 正反馈回归：云台转动后目标方位随之改变
 *
 * 这是"用 atan2 绝对值当参考"会在实车上暴露的故障。构造一个静态目标，
 * 用相对方位迭代推进，检查云台是否收敛到正对目标并保持稳定。
 *============================================================================*/
static void test_no_positive_feedback(void)
{
    /*
     * 世界系下目标固定在云台 INITIAL_YAW + 30 deg 方向、水平距离 5 m、
     * 高度 0（与云台同高）。
     */
    const float target_world_bearing = 30.0f;
    const float distance = 5.0f;
    const float step = 40.0f; /* deg/周期，模拟限幅后的推进量 */
    float yaw_ref = 0.0f;     /* 当前角度参考 */
    int i;

    for (i = 0; i < 200; i++) {
        /* 上位机在云台系下观测到的相对方位 */
        float delta = target_world_bearing - yaw_ref;
        float x = distance * cosf(delta * 0.01745329251994329577f);
        float y = distance * sinf(delta * 0.01745329251994329577f);
        float next = Gimbal_TargetYawReferenceDeg(yaw_ref, x, y);
        float applied = next - yaw_ref;

        if (applied > step) {
            applied = step;
        } else if (applied < -step) {
            applied = -step;
        }
        yaw_ref += applied;
    }

    CHECK(near(yaw_ref, target_world_bearing, 1.0f),
          "relative-bearing integration must converge to target, got %f",
          (double)yaw_ref);

    /* 收敛后必须停住，不能持续过冲 */
    {
        float delta = target_world_bearing - yaw_ref;
        float x = distance * cosf(delta * 0.01745329251994329577f);
        float y = distance * sinf(delta * 0.01745329251994329577f);
        float next = Gimbal_TargetYawReferenceDeg(yaw_ref, x, y);

        CHECK(near(next, yaw_ref, 0.01f),
              "converged reference must be a fixed point, got %f vs %f",
              (double)next, (double)yaw_ref);
    }
}

/*============================================================================
 * Pitch：目标仰角（本车 IMU 符号：抬头为负）
 *============================================================================*/
static void test_pitch_reference(void)
{
    const float min_h = 0.10f;
    const float nan_v = float_from_bits(0x7FC00000U);

    /* 同高目标：仰角 0 */
    CHECK(near(Gimbal_TargetPitchReferenceDeg(5.0f, 0.0f, 0.0f, min_h),
               0.0f, 1e-4f),
          "level target -> 0 pitch");

    /* 目标在上方（z>0）：IMU 抬头，本车符号为负 */
    {
        float r = Gimbal_TargetPitchReferenceDeg(1.0f, 0.0f, 1.0f, min_h);
        CHECK(r < 0.0f, "target above -> negative IMU pitch, got %f",
              (double)r);
        CHECK(near(r, -45.0f, 1e-3f), "45deg up -> -45 imu, got %f", (double)r);
    }
    /* 目标在下方（z<0）：IMU 低头，本车符号为正 */
    {
        float r = Gimbal_TargetPitchReferenceDeg(1.0f, 0.0f, -1.0f, min_h);
        CHECK(r > 0.0f, "target below -> positive IMU pitch, got %f",
              (double)r);
        CHECK(near(r, 45.0f, 1e-3f), "45deg down -> +45 imu, got %f",
              (double)r);
    }
    /* 水平距离用 sqrt(x^2+y^2)，不是只用 x */
    {
        float r = Gimbal_TargetPitchReferenceDeg(0.0f, 1.0f, 1.0f, min_h);
        CHECK(near(r, -45.0f, 1e-3f),
              "horizontal distance must include y, got %f", (double)r);
    }

    /* 退化：水平距离过小 -> NaN，调用方据此放弃该帧 */
    {
        float d1 = Gimbal_TargetPitchReferenceDeg(0.0f, 0.0f, 5.0f, min_h);
        float d2 = Gimbal_TargetPitchReferenceDeg(0.05f, 0.05f, 1.0f, min_h);

        CHECK(!Gimbal_FloatIsFinite(d1),
              "zero horizontal distance must be non-finite, got %f",
              (double)d1);
        CHECK(!Gimbal_FloatIsFinite(d2),
              "below-min horizontal distance must be non-finite, got %f",
              (double)d2);
    }

    /* 非有限输入必须被位模式检查拦下（fast-math 下同样成立） */
    CHECK(!Gimbal_FloatIsFinite(
              Gimbal_TargetPitchReferenceDeg(nan_v, 0.0f, 1.0f, min_h)),
          "NaN x must yield non-finite");
    CHECK(!Gimbal_FloatIsFinite(Gimbal_TargetPitchReferenceDeg(
              1.0f, float_from_bits(0x7F800000U), 1.0f, min_h)),
          "Inf y must yield non-finite");
    CHECK(!Gimbal_FloatIsFinite(Gimbal_TargetPitchReferenceDeg(
              1.0f, 0.0f, float_from_bits(0xFF800000U), min_h)),
          "-Inf z must yield non-finite");
}

/**
 * @brief 多圈累计角参考的推进：不得归一化、必须限幅。
 *
 * 回归用例：早期实现用 WrapAngleDeg180 折参考，导致 yaw_hold_ref 与
 * attitude.YawTotalAngle（多圈累计）在跨越 ±180 时产生 360 deg 阶跃。
 */
static void test_advance_angle_ref(void)
{
    /* 正常推进 */
    CHECK(Gimbal_AdvanceAngleRefDeg(10.0f, 5.0f, 100.0f) == 15.0f,
          "normal advance");

    /* 限幅 */
    CHECK(Gimbal_AdvanceAngleRefDeg(10.0f, 500.0f, 40.0f) == 50.0f,
          "positive clamp");
    CHECK(Gimbal_AdvanceAngleRefDeg(10.0f, -500.0f, 40.0f) == -30.0f,
          "negative clamp");

    /* 关键：跨越 ±180 不得折返，必须继续累加 */
    CHECK(Gimbal_AdvanceAngleRefDeg(179.0f, 5.0f, 40.0f) == 184.0f,
          "crossing +180 must keep accumulating (no wrap)");
    CHECK(Gimbal_AdvanceAngleRefDeg(-179.0f, -5.0f, 40.0f) == -184.0f,
          "crossing -180 must keep accumulating (no wrap)");

    /* 多圈后仍线性累加：3 圈附近不得被折叠 */
    CHECK(Gimbal_AdvanceAngleRefDeg(1075.0f, 5.0f, 40.0f) == 1080.0f,
          "multi-turn reference preserved");

    /* 退化输入：保持原值 */
    CHECK(Gimbal_AdvanceAngleRefDeg(10.0f, 5.0f, 0.0f) == 10.0f,
          "zero limit holds");
    CHECK(Gimbal_AdvanceAngleRefDeg(10.0f, 5.0f, -1.0f) == 10.0f,
          "negative limit holds");
    CHECK(Gimbal_AdvanceAngleRefDeg(10.0f, 5.0f,
                                    float_from_bits(0x7F800000U)) == 10.0f,
          "Inf limit holds");
    CHECK(Gimbal_AdvanceAngleRefDeg(10.0f,
                                    float_from_bits(0x7FC00000U),
                                    40.0f) == 10.0f,
          "NaN delta holds");
    CHECK(Gimbal_AdvanceAngleRefDeg(float_from_bits(0x7FC00000U), 5.0f,
                                    40.0f) !=
              Gimbal_AdvanceAngleRefDeg(float_from_bits(0x7FC00000U), 5.0f,
                                        40.0f),
          "NaN reference stays NaN (caller must reject)");
}

/**
 * @brief 位模式有限值检查必须不依赖 isfinite()。
 *
 * 即使使用 fast-math 选项，本测试也必须通过。
 */
static void test_finite_check_under_fast_math(void)
{
    const float nan_v = float_from_bits(0x7FC00000U);
    const float inf_v = float_from_bits(0x7F800000U);
    const float ninf_v = float_from_bits(0xFF800000U);

    CHECK(Gimbal_FloatIsFinite(0.0f), "zero is finite");
    CHECK(Gimbal_FloatIsFinite(-1.0f), "-1 is finite");
    CHECK(Gimbal_FloatIsFinite(3.0e38f), "large finite accepted");
    CHECK(Gimbal_FloatIsFinite(1.0e-38f), "tiny finite accepted");
    CHECK(!Gimbal_FloatIsFinite(nan_v), "NaN rejected");
    CHECK(!Gimbal_FloatIsFinite(inf_v), "+Inf rejected");
    CHECK(!Gimbal_FloatIsFinite(ninf_v), "-Inf rejected");
    CHECK(!Gimbal_FloatIsFinite(Gimbal_NaNF()), "Gimbal_NaNF rejected");

    /*
     * 交叉验证：未启用 fast-math 时 isfinite() 与位模式检查结论一致。
     * 启用 fast-math 时该对比无意义，故跳过。
     */
#ifndef __FAST_MATH__
    CHECK(isfinite(3.0e38f) == (Gimbal_FloatIsFinite(3.0e38f) != 0),
          "bit-pattern check must agree with isfinite for finite values");
    CHECK(!isfinite(inf_v) && !Gimbal_FloatIsFinite(inf_v),
          "bit-pattern check must agree with isfinite for Inf");
#else
    printf("note: -ffast-math build; isfinite() cross-check skipped\n");
#endif
}

/*============================================================================
 * 与协议 pitch 符号的一致性
 *
 * 遥测侧 RmVision_PitchImuToWire 把"抬头为负"转成"抬头为正"；
 * 目标侧 Gimbal_TargetPitchReferenceDeg 反向。两者必须互为镜像，
 * 否则上报与接收会用相反的符号理解同一个物理量。
 *============================================================================*/
static void test_pitch_sign_consistency(void)
{
    /* 目标在正上方 45 deg：目标侧解出 IMU 抬头 -> 负值 */
    float imu_target = Gimbal_TargetPitchReferenceDeg(1.0f, 0.0f, 1.0f, 0.1f);
    /* 上报该姿态时协议侧应为正（REP-103 抬头为正） */
    float wire = -imu_target; /* 等价于 RmVision_PitchImuToWire 的符号项 */

    CHECK(imu_target < 0.0f && wire > 0.0f,
          "imu/wire pitch must have opposite signs: imu=%f wire=%f",
          (double)imu_target, (double)wire);
    CHECK(near(wire, 45.0f, 1e-3f), "wire pitch should be +45, got %f",
          (double)wire);
}

int main(void)
{
    test_unit_conversion();
    test_yaw_reference();
    test_no_positive_feedback();
    test_pitch_reference();
    test_advance_angle_ref();
    test_finite_check_under_fast_math();
    test_pitch_sign_consistency();

    printf("vision_aim: %d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
