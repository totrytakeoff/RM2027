#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "gimbal_limits.h"

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
    return error <= 0.0001f;
}

static const GimbalPitchLimitConfig limits = {
    .min_angle_deg = -30.0f,
    .max_angle_deg = 20.0f,
};

static void TestConfigurationAndInvalidFeedback(void)
{
    GimbalPitchLimitConfig invalid = limits;

    CHECK(GimbalPitchLimit_IsConfigValid(&limits));
    CHECK(!GimbalPitchLimit_IsConfigValid(NULL));
    invalid.max_angle_deg = invalid.min_angle_deg;
    CHECK(!GimbalPitchLimit_IsConfigValid(&invalid));
    CHECK(Near(GimbalPitchLimit_ClampAngleReference(
                   20.0f, false, 10.0f, &limits),
               10.0f));
    CHECK(Near(GimbalPitchLimit_ClampAngleReference(
                   NAN, true, 10.0f, &limits),
               10.0f));
}

static void TestAngleReferenceUsesSameBoundary(void)
{
    CHECK(Near(GimbalPitchLimit_ClampAngleReference(
                   120.0f, true, 18.5f, &limits),
               20.0f));
    CHECK(Near(GimbalPitchLimit_ClampAngleReference(
                   120.0f, true, 20.0f, &limits),
               20.0f));
    CHECK(Near(GimbalPitchLimit_ClampAngleReference(
                   -80.0f, true, 20.0f, &limits),
               -30.0f));
    CHECK(Near(GimbalPitchLimit_ClampAngleReference(
                   20.0f, true, 0.0f, &limits),
               20.0f));
}

static void TestAngleReferenceTimeIntegration(void)
{
    CHECK(Near(GimbalPitchLimit_AdvanceAngleReference(
                   0.0f, 0.5f, 100.0f, 0.02f, &limits),
               1.0f));
    CHECK(Near(GimbalPitchLimit_AdvanceAngleReference(
                   0.0f, -0.5f, 100.0f, 0.02f, &limits),
               -1.0f));
    CHECK(Near(GimbalPitchLimit_AdvanceAngleReference(
                   19.0f, 2.0f, 100.0f, 0.02f, &limits),
               20.0f));
    CHECK(Near(GimbalPitchLimit_AdvanceAngleReference(
                   -29.0f, -2.0f, 100.0f, 0.02f, &limits),
               -30.0f));
    CHECK(Near(GimbalPitchLimit_AdvanceAngleReference(
                   3.0f, NAN, 100.0f, 0.02f, &limits),
               3.0f));
    CHECK(Near(GimbalPitchLimit_AdvanceAngleReference(
                   3.0f, 1.0f, 100.0f, -0.02f, &limits),
               3.0f));
}

int main(void)
{
    TestConfigurationAndInvalidFeedback();
    TestAngleReferenceUsesSameBoundary();
    TestAngleReferenceTimeIntegration();

    if (failures != 0U) {
        fprintf(stderr, "%u gimbal-limit checks failed\n", failures);
        return 1;
    }
    puts("gimbal-limit checks passed");
    return 0;
}
