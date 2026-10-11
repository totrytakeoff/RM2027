/**
 * @file vision_link_state_test.c
 * @brief 视觉在线、目标接管和降级边沿的主机测试
 */

#include <stdbool.h>
#include <stdio.h>

#include "vision_link_state.h"

static unsigned checks;
static unsigned failures;

#define CHECK(condition)                                                        \
    do {                                                                        \
        checks++;                                                               \
        if (!(condition)) {                                                     \
            failures++;                                                         \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,           \
                    #condition);                                               \
        }                                                                       \
    } while (false)

static void test_online_is_independent_from_target(void)
{
    RmVisionLinkState state;

    RmVisionLinkState_Reset(&state);
    CHECK(!RmVisionLinkState_IsOnline(&state));
    CHECK(!RmVisionLinkState_HasValidTarget(&state));
    CHECK(!RmVisionLinkState_HasTarget(&state));

    /* 手动模式收到合法帧：在线，但不允许目标接管。 */
    RmVisionLinkState_ConsumeFrame(&state, true, true);
    CHECK(RmVisionLinkState_IsOnline(&state));
    CHECK(!RmVisionLinkState_HasValidTarget(&state));
    CHECK(!RmVisionLinkState_HasTarget(&state));

    /* 自瞄开启后仍需等待一帧，不能复用手动模式期间的旧帧。 */
    RmVisionLinkState_SetAutoAim(&state, true);
    CHECK(RmVisionLinkState_IsOnline(&state));
    CHECK(!RmVisionLinkState_HasTarget(&state));

    RmVisionLinkState_ConsumeFrame(&state, true, false);
    CHECK(RmVisionLinkState_IsOnline(&state));
    CHECK(RmVisionLinkState_HasValidTarget(&state));
    CHECK(!RmVisionLinkState_HasTarget(&state));

    RmVisionLinkState_ConsumeFrame(&state, true, true);
    CHECK(RmVisionLinkState_IsOnline(&state));
    CHECK(RmVisionLinkState_HasValidTarget(&state));
    CHECK(RmVisionLinkState_HasTarget(&state));
}

static void test_invalid_and_lost_frames_fail_safe(void)
{
    RmVisionLinkState state;

    RmVisionLinkState_Reset(&state);
    RmVisionLinkState_SetAutoAim(&state, true);
    RmVisionLinkState_ConsumeFrame(&state, true, true);
    CHECK(RmVisionLinkState_HasTarget(&state));

    /* 非有限目标仍证明链路在线，但要撤销旧目标。 */
    RmVisionLinkState_ConsumeFrame(&state, false, true);
    CHECK(RmVisionLinkState_IsOnline(&state));
    CHECK(!RmVisionLinkState_HasValidTarget(&state));
    CHECK(!RmVisionLinkState_HasTarget(&state));

    RmVisionLinkState_ConsumeFrame(&state, true, true);
    CHECK(RmVisionLinkState_HasTarget(&state));
    RmVisionLinkState_OnPeerLost(&state);
    CHECK(!RmVisionLinkState_IsOnline(&state));
    CHECK(!RmVisionLinkState_HasValidTarget(&state));
    CHECK(!RmVisionLinkState_HasTarget(&state));

    /* 离线后即使保留自瞄电平，也必须等待新帧恢复。 */
    RmVisionLinkState_ConsumeFrame(&state, true, true);
    CHECK(RmVisionLinkState_HasTarget(&state));
}

static void test_auto_aim_edge_clears_target(void)
{
    RmVisionLinkState state;

    RmVisionLinkState_Reset(&state);
    RmVisionLinkState_SetAutoAim(&state, true);
    RmVisionLinkState_ConsumeFrame(&state, true, true);
    CHECK(RmVisionLinkState_HasTarget(&state));

    RmVisionLinkState_SetAutoAim(&state, false);
    CHECK(RmVisionLinkState_IsOnline(&state));
    CHECK(!RmVisionLinkState_HasValidTarget(&state));
    CHECK(!RmVisionLinkState_HasTarget(&state));

    RmVisionLinkState_SetAutoAim(&state, true);
    CHECK(!RmVisionLinkState_HasTarget(&state));
    RmVisionLinkState_ConsumeFrame(&state, true, true);
    CHECK(RmVisionLinkState_HasTarget(&state));
}

int main(void)
{
    test_online_is_independent_from_target();
    test_invalid_and_lost_frames_fail_safe();
    test_auto_aim_edge_clears_target();

    printf("vision_link_state: %u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
