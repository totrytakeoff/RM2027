#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "gimbal_axis_state.h"

static unsigned failures;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,           \
                    #condition);                                               \
            failures++;                                                        \
        }                                                                      \
    } while (false)

static const GimbalAxisStateConfig config = {
    .brake_speed_epsilon = 10.0f,
    .brake_stable_count_required = 3U,
    .brake_timeout_ms = 220U,
};

static void TestNormalTransition(void)
{
    GimbalAxisState state;
    GimbalAxisTransition transition;

    GimbalAxisState_Init(&state);
    CHECK(state.mode == AXIS_CTRL_ANGLE);

    CHECK(GimbalAxisState_Update(
        &state, &config, true, 0.0f, 100U, &transition));
    CHECK(transition.changed);
    CHECK(transition.previous_mode == AXIS_CTRL_ANGLE);
    CHECK(state.mode == AXIS_CTRL_SPEED);

    CHECK(GimbalAxisState_Update(
        &state, &config, false, 80.0f, 120U, &transition));
    CHECK(transition.changed);
    CHECK(state.mode == AXIS_CTRL_BRAKE);

    CHECK(GimbalAxisState_Update(
        &state, &config, false, 5.0f, 140U, &transition));
    CHECK(state.mode == AXIS_CTRL_BRAKE);
    CHECK(state.brake_stable_count == 1U);
    CHECK(GimbalAxisState_Update(
        &state, &config, false, -5.0f, 160U, &transition));
    CHECK(state.mode == AXIS_CTRL_BRAKE);
    CHECK(GimbalAxisState_Update(
        &state, &config, false, 0.0f, 180U, &transition));
    CHECK(transition.changed);
    CHECK(state.mode == AXIS_CTRL_ANGLE);
}

static void TestTimeoutAndReactivation(void)
{
    GimbalAxisState state;
    GimbalAxisTransition transition;

    GimbalAxisState_Init(&state);
    CHECK(GimbalAxisState_Update(
        &state, &config, true, 0.0f, 1000U, NULL));
    CHECK(GimbalAxisState_Update(
        &state, &config, false, 100.0f, 1020U, NULL));
    CHECK(GimbalAxisState_Update(
        &state, &config, true, 100.0f, 1100U, &transition));
    CHECK(transition.changed);
    CHECK(state.mode == AXIS_CTRL_SPEED);

    CHECK(GimbalAxisState_Update(
        &state, &config, false, 100.0f, 1200U, NULL));
    CHECK(GimbalAxisState_Update(
        &state, &config, false, 100.0f, 1420U, &transition));
    CHECK(transition.changed);
    CHECK(state.mode == AXIS_CTRL_ANGLE);
}

static void TestInvalidInputFailsClosed(void)
{
    GimbalAxisState state;
    GimbalAxisStateConfig invalid = config;

    GimbalAxisState_Init(&state);
    invalid.brake_stable_count_required = 0U;
    CHECK(!GimbalAxisState_Update(
        &state, &invalid, true, 0.0f, 0U, NULL));
    CHECK(state.mode == AXIS_CTRL_ANGLE);
    CHECK(!GimbalAxisState_Update(
        &state, &config, true, NAN, 0U, NULL));
    CHECK(state.mode == AXIS_CTRL_ANGLE);
}

static void TestTimeoutAcrossTickWrap(void)
{
    GimbalAxisState state;
    GimbalAxisTransition transition;

    GimbalAxisState_Init(&state);
    CHECK(GimbalAxisState_Update(
        &state, &config, true, 0.0f, UINT32_MAX - 100U, NULL));
    CHECK(GimbalAxisState_Update(
        &state, &config, false, 100.0f, UINT32_MAX - 50U, NULL));
    CHECK(GimbalAxisState_Update(
        &state, &config, false, 100.0f, 170U, &transition));
    CHECK(transition.changed);
    CHECK(state.mode == AXIS_CTRL_ANGLE);
}

int main(void)
{
    TestNormalTransition();
    TestTimeoutAndReactivation();
    TestInvalidInputFailsClosed();
    TestTimeoutAcrossTickWrap();

    if (failures != 0U) {
        fprintf(stderr, "%u checks failed\n", failures);
        return 1;
    }
    puts("gimbal_axis_state_test: PASS");
    return 0;
}
