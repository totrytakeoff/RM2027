#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rm_rx_queue.h"

static unsigned failures;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,           \
                    #condition);                                               \
            failures++;                                                        \
        }                                                                      \
    } while (false)

static void CheckPop(RmRxQueue *queue,
                     const uint8_t *expected,
                     uint16_t expected_length)
{
    uint8_t output[8] = {0};
    uint16_t length = 0U;

    CHECK(RmRxQueue_Pop(queue, output, sizeof(output), &length));
    CHECK(length == expected_length);
    CHECK(memcmp(output, expected, expected_length) == 0);
}

static void TestFifoAndWrap(void)
{
    uint8_t storage[3][4];
    uint16_t lengths[3];
    RmRxQueue queue;
    const uint8_t first[] = {1U, 2U};
    const uint8_t second[] = {3U};
    const uint8_t third[] = {4U, 5U, 6U};
    const uint8_t fourth[] = {7U, 8U};

    CHECK(RmRxQueue_Init(&queue, storage, lengths, 4U, 3U));
    CHECK(RmRxQueue_PushLatest(&queue, first, sizeof(first)) ==
          RM_RX_QUEUE_PUSH_STORED);
    CHECK(RmRxQueue_PushLatest(&queue, second, sizeof(second)) ==
          RM_RX_QUEUE_PUSH_STORED);
    CheckPop(&queue, first, sizeof(first));
    CHECK(RmRxQueue_PushLatest(&queue, third, sizeof(third)) ==
          RM_RX_QUEUE_PUSH_STORED);
    CHECK(RmRxQueue_PushLatest(&queue, fourth, sizeof(fourth)) ==
          RM_RX_QUEUE_PUSH_STORED);
    CheckPop(&queue, second, sizeof(second));
    CheckPop(&queue, third, sizeof(third));
    CheckPop(&queue, fourth, sizeof(fourth));
    CHECK(RmRxQueue_Pending(&queue) == 0U);
    CHECK(queue.push_count == 4U);
    CHECK(queue.pop_count == 4U);
}

static void TestOverflowKeepsNewest(void)
{
    uint8_t storage[3][2];
    uint16_t lengths[3];
    RmRxQueue queue;
    const uint8_t values[] = {10U, 11U, 12U, 13U};

    CHECK(RmRxQueue_Init(&queue, storage, lengths, 2U, 3U));
    for (uint8_t i = 0U; i < 3U; ++i) {
        CHECK(RmRxQueue_PushLatest(&queue, &values[i], 1U) ==
              RM_RX_QUEUE_PUSH_STORED);
    }
    CHECK(RmRxQueue_PushLatest(&queue, &values[3], 1U) ==
          RM_RX_QUEUE_PUSH_OVERWROTE_OLDEST);
    CHECK(queue.overwrite_count == 1U);
    CHECK(RmRxQueue_Pending(&queue) == 3U);
    CheckPop(&queue, &values[1], 1U);
    CheckPop(&queue, &values[2], 1U);
    CheckPop(&queue, &values[3], 1U);
}

static void TestSingleSlotCoalescesToLatest(void)
{
    uint8_t storage[1][4];
    uint16_t lengths[1];
    RmRxQueue queue;
    const uint8_t old_value[] = {1U, 1U};
    const uint8_t latest_value[] = {2U, 2U, 2U};

    CHECK(RmRxQueue_Init(&queue, storage, lengths, 4U, 1U));
    CHECK(RmRxQueue_PushLatest(&queue, old_value, sizeof(old_value)) ==
          RM_RX_QUEUE_PUSH_STORED);
    CHECK(RmRxQueue_PushLatest(&queue,
                               latest_value,
                               sizeof(latest_value)) ==
          RM_RX_QUEUE_PUSH_OVERWROTE_OLDEST);
    CheckPop(&queue, latest_value, sizeof(latest_value));
    CHECK(queue.overwrite_count == 1U);
}

static void TestRejectedInputPreservesQueue(void)
{
    uint8_t storage[2][2];
    uint16_t lengths[2];
    uint8_t output[2];
    uint16_t length = 0U;
    RmRxQueue queue;
    const uint8_t valid[] = {9U, 8U};
    const uint8_t oversized[] = {1U, 2U, 3U};

    CHECK(!RmRxQueue_Init(NULL, storage, lengths, 2U, 2U));
    CHECK(RmRxQueue_Init(&queue, storage, lengths, 2U, 2U));
    CHECK(RmRxQueue_PushLatest(&queue, valid, sizeof(valid)) ==
          RM_RX_QUEUE_PUSH_STORED);
    CHECK(RmRxQueue_PushLatest(&queue, oversized, sizeof(oversized)) ==
          RM_RX_QUEUE_PUSH_REJECTED);
    CHECK(!RmRxQueue_Pop(&queue, output, 1U, &length));
    CHECK(RmRxQueue_Pending(&queue) == 1U);
    CheckPop(&queue, valid, sizeof(valid));
}

int main(void)
{
    TestFifoAndWrap();
    TestOverflowKeepsNewest();
    TestSingleSlotCoalescesToLatest();
    TestRejectedInputPreservesQueue();

    if (failures != 0U) {
        fprintf(stderr, "%u receive-queue checks failed\n", failures);
        return 1;
    }

    puts("receive-queue checks passed");
    return 0;
}
