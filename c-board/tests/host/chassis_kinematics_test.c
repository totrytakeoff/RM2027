#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "chassis_follow.h"
#include "chassis_kinematics.h"

static unsigned failures;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,           \
                    #condition);                                               \
            failures++;                                                        \
        }                                                                      \
    } while (false)

static bool Near(float lhs, float rhs)
{
    float error = lhs - rhs;
    if (error < 0.0f) {
        error = -error;
    }
    return error <= 0.0002f;
}

static void TestCoordinateRotation(void)
{
    float vx;
    float vy;
    const float half_pi = 1.57079632679489661923f;

    Chassis_RotateToBody(1.0f, 2.0f, 0.0f, &vx, &vy);
    CHECK(Near(vx, 1.0f));
    CHECK(Near(vy, 2.0f));

    /* 云台相对底盘左转 90°：云台前方就是底盘左方。 */
    Chassis_RotateToBody(0.0f, 1.0f, half_pi, &vx, &vy);
    CHECK(Near(vx, 1.0f));
    CHECK(Near(vy, 0.0f));

    /* 此时云台左方就是底盘后方。 */
    Chassis_RotateToBody(1.0f, 0.0f, half_pi, &vx, &vy);
    CHECK(Near(vx, 0.0f));
    CHECK(Near(vy, -1.0f));

    /* 云台相对底盘右转 90° 时方向应严格反对称。 */
    Chassis_RotateToBody(0.0f, 1.0f, -half_pi, &vx, &vy);
    CHECK(Near(vx, -1.0f));
    CHECK(Near(vy, 0.0f));
    Chassis_RotateToBody(1.0f, 0.0f, -half_pi, &vx, &vy);
    CHECK(Near(vx, 0.0f));
    CHECK(Near(vy, 1.0f));
}

static void TestTranslationVectorLimit(void)
{
    float vx = 3.0f;
    float vy = 4.0f;

    Chassis_LimitTranslation(&vx, &vy, 0.75f);
    CHECK(Near(vx, 0.45f));
    CHECK(Near(vy, 0.60f));

    Chassis_LimitTranslation(&vx, &vy, 0.0f);
    CHECK(Near(vx, 0.0f));
    CHECK(Near(vy, 0.0f));
}

static void TestOmniInverseAndNormalization(void)
{
    float speeds[4];

    Chassis_OmniInverse(1.0f, 0.0f, 0.0f, 0.34f, 0.1f,
                                speeds);
    CHECK(Near(speeds[0], 10.0f));
    CHECK(Near(speeds[1], 10.0f));
    CHECK(Near(speeds[2], -10.0f));
    CHECK(Near(speeds[3], -10.0f));

    /* 执行层统一定义 +vy 为车体向前。 */
    Chassis_OmniInverse(0.0f, 1.0f, 0.0f, 0.34f, 0.1f,
                                speeds);
    CHECK(Near(speeds[0], -10.0f));
    CHECK(Near(speeds[1], 10.0f));
    CHECK(Near(speeds[2], -10.0f));
    CHECK(Near(speeds[3], 10.0f));

    speeds[0] = 20.0f;
    speeds[1] = -10.0f;
    speeds[2] = 5.0f;
    speeds[3] = -2.0f;
    Chassis_NormalizeWheelSpeeds(speeds, 10.0f);
    CHECK(Near(speeds[0], 10.0f));
    CHECK(Near(speeds[1], -5.0f));
    CHECK(Near(speeds[2], 2.5f));
    CHECK(Near(speeds[3], -1.0f));
}

static void TestFollowControllerSignAndDamping(void)
{
    ChassisFollowOutput output;
    const float two_pi = 6.28318530717958647692f;

    CHECK(Chassis_CalculateFollowOutput(
        0.2f, 0.0f, 0.0f, 5.0f, 0.0f, 0.1f, 2.5f, &output));
    CHECK(Near(output.p_rad_s, 1.0f));
    CHECK(Near(output.limited_wz_rad_s, 1.0f));

    /* 正误差正在减小时 e_dot<0，正 Kd 必须降低追赶角速度。 */
    CHECK(Chassis_CalculateFollowOutput(
        0.2f, -2.0f, 0.0f, 5.0f, 0.0f, 0.1f, 2.5f, &output));
    CHECK(Near(output.d_rad_s, -0.2f));
    CHECK(Near(output.raw_wz_rad_s, 0.8f));
    CHECK(Near(output.limited_wz_rad_s, 0.8f));

    CHECK(Chassis_CalculateFollowOutput(
        1.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.1f, 2.5f, &output));
    CHECK(Near(output.limited_wz_rad_s, 2.5f));

    /* 累计角跨过整圈后仍必须走到零位的最近方向。 */
    CHECK(Chassis_CalculateFollowOutput(
        two_pi + 0.2f, 0.0f, 0.0f, 5.0f, 0.0f, 0.1f, 2.5f, &output));
    CHECK(Near(output.p_rad_s, 1.0f));
    CHECK(Chassis_CalculateFollowOutput(
        -two_pi - 0.2f, 0.0f, 0.0f, 5.0f, 0.0f, 0.1f, 2.5f, &output));
    CHECK(Near(output.p_rad_s, -1.0f));

    CHECK(!Chassis_CalculateFollowOutput(
        NAN, 0.0f, 0.0f, 5.0f, 0.0f, 0.1f, 2.5f, &output));
    CHECK(Near(output.limited_wz_rad_s, 0.0f));
}

static void TestFullTranslationUsesConfiguredMotorRange(void)
{
    const float motor_max_rad_s = 523.5987756f;
    const float reduction = 19.20320856f;
    const float wheel_radius = 0.075f;
    const float max_translation =
        motor_max_rad_s * wheel_radius / reduction;
    float wheel_speeds[4];

    Chassis_OmniInverse(max_translation, 0.0f, 0.0f,
                                0.34f, wheel_radius, wheel_speeds);
    for (unsigned index = 0U; index < 4U; ++index) {
        const float motor_rad_s = Chassis_WheelToMotorSpeedRadS(
            wheel_speeds[index], reduction);
        CHECK(Near(fabsf(motor_rad_s), motor_max_rad_s));
    }
    CHECK(Near(Chassis_WheelToMotorSpeedRadS(NAN, reduction), 0.0f));
    CHECK(Near(Chassis_WheelToMotorSpeedRadS(1.0f, 0.0f), 0.0f));
}

static void TestSpinPreservesRotationAndScalesTranslation(void)
{
    const float translation[4] = {400.0f, -400.0f, 400.0f, -400.0f};
    const float rotation[4] = {300.0f, 300.0f, 300.0f, 300.0f};
    float output[4];
    float scale;

    CHECK(Chassis_CombineWheelSpeedsPreserveRotation(
        translation, rotation, 500.0f, output, &scale));
    CHECK(Near(scale, 0.5f));
    CHECK(Near(output[0], 500.0f));
    CHECK(Near(output[1], 100.0f));
    CHECK(Near(output[2], 500.0f));
    CHECK(Near(output[3], 100.0f));

    {
        const float full_rotation[4] = {
            500.0f, 500.0f, 500.0f, 500.0f,
        };
        CHECK(Chassis_CombineWheelSpeedsPreserveRotation(
            translation, full_rotation, 500.0f, output, &scale));
        CHECK(Near(scale, 0.0f));
        for (unsigned index = 0U; index < 4U; ++index) {
            CHECK(Near(output[index], 500.0f));
        }
    }

    {
        const float reverse_rotation[4] = {
            -300.0f, -300.0f, -300.0f, -300.0f,
        };
        CHECK(Chassis_CombineWheelSpeedsPreserveRotation(
            translation, reverse_rotation, 500.0f, output, &scale));
        CHECK(Near(scale, 0.5f));
        CHECK(Near(output[0], -100.0f));
        CHECK(Near(output[1], -500.0f));
    }

    {
        const float invalid_translation[4] = {
            NAN, 0.0f, 0.0f, 0.0f,
        };
        CHECK(!Chassis_CombineWheelSpeedsPreserveRotation(
            invalid_translation, rotation, 500.0f, output, &scale));
        CHECK(Near(scale, 0.0f));
        for (unsigned index = 0U; index < 4U; ++index) {
            CHECK(Near(output[index], 0.0f));
        }
    }
}

static void TestInvalidNumbersFailClosed(void)
{
    float vx = NAN;
    float vy = 1.0f;
    float speeds[4] = {1.0f, 2.0f, NAN, 4.0f};

    Chassis_LimitTranslation(&vx, &vy, 0.75f);
    CHECK(Near(vx, 0.0f));
    CHECK(Near(vy, 0.0f));

    Chassis_RotateToBody(1.0f, 2.0f, INFINITY, &vx, &vy);
    CHECK(Near(vx, 0.0f));
    CHECK(Near(vy, 0.0f));

    Chassis_NormalizeWheelSpeeds(speeds, 10.0f);
    CHECK(Near(speeds[0], 0.0f));
    CHECK(Near(speeds[1], 0.0f));
    CHECK(Near(speeds[2], 0.0f));
    CHECK(Near(speeds[3], 0.0f));
}

int main(void)
{
    TestCoordinateRotation();
    TestTranslationVectorLimit();
    TestOmniInverseAndNormalization();
    TestFullTranslationUsesConfiguredMotorRange();
    TestSpinPreservesRotationAndScalesTranslation();
    TestInvalidNumbersFailClosed();
    TestFollowControllerSignAndDamping();

    if (failures != 0U) {
        fprintf(stderr, "%u chassis-kinematics checks failed\n", failures);
        return 1;
    }
    puts("chassis-kinematics checks passed");
    return 0;
}
