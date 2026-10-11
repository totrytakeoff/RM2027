#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "gimbal_units.h"

static unsigned failures;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,          \
                    #condition);                                               \
            failures++;                                                        \
        }                                                                      \
    } while (false)

static bool Near(float lhs, float rhs)
{
    return fabsf(lhs - rhs) <= 0.001f;
}

int main(void)
{
    CHECK(Near(Gimbal_RadPerSecToDegPerSec(0.0f), 0.0f));
    CHECK(Near(Gimbal_RadPerSecToDegPerSec(1.0f), 57.2957795f));
    CHECK(Near(Gimbal_RadPerSecToDegPerSec(-3.14159265358979323846f),
               -180.0f));
    CHECK(Near(Gimbal_EstimateBaseRateRadS(1.0f, -7.0f),
               8.0f));
    CHECK(Near(Gimbal_EstimateBaseRateRadS(-1.0f, 7.0f),
               -8.0f));
    CHECK(Near(Gimbal_EstimateBaseRateRadS(NAN, 0.0f),
               0.0f));
    CHECK(Near(Gimbal_GravityFeedforward(0.0f, 0.0f,
                                                 2500.0f, 4000.0f),
               2500.0f));
    CHECK(Near(Gimbal_GravityFeedforward(90.0f, 0.0f,
                                                 2500.0f, 4000.0f),
               0.0f));
    CHECK(Near(Gimbal_GravityFeedforward(0.0f, 0.0f,
                                                 5000.0f, 4000.0f),
               4000.0f));
    CHECK(Near(Gimbal_GravityFeedforward(NAN, 0.0f,
                                                 2500.0f, 4000.0f),
               0.0f));
    CHECK(Near(Gimbal_BaseRateCurrentFeedforward(
                   8.0f, 800.0f, 0.2f, 10000.0f),
               -6400.0f));
    CHECK(Near(Gimbal_BaseRateCurrentFeedforward(
                   -8.0f, 800.0f, 0.2f, 10000.0f),
               6400.0f));
    CHECK(Near(Gimbal_BaseRateCurrentFeedforward(
                   -0.1f, 800.0f, 0.2f, 10000.0f),
               0.0f));
    CHECK(Near(Gimbal_BaseRateCurrentFeedforward(
                   20.0f, 800.0f, 0.2f, 10000.0f),
               -10000.0f));
    CHECK(Near(Gimbal_BaseRateCurrentFeedforward(
                   NAN, 800.0f, 0.2f, 10000.0f),
               0.0f));

    if (failures != 0U) {
        fprintf(stderr, "%u gimbal-unit checks failed\n", failures);
        return 1;
    }

    puts("gimbal-unit checks passed");
    return 0;
}
