#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "daemon.h"

static unsigned failures;
static uint32_t fake_now_ms;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,           \
                    #condition);                                               \
            failures++;                                                        \
        }                                                                      \
    } while (false)

uint32_t RmTime_NowMs(void)
{
    return fake_now_ms;
}

static void CountOffline(void *owner)
{
    unsigned *count = owner;
    (*count)++;
}

static void TestDeadlineAndReload(void)
{
    unsigned offline_count = 0U;
    const DaemonConfig config = {
        .timeout_ms = 40U,
        .initial_grace_ms = 10U,
        .callback = CountOffline,
        .owner = &offline_count,
    };
    DaemonInstance *instance;

    fake_now_ms = 100U;
    DaemonServiceInit();
    instance = DaemonRegister(&config);
    CHECK(instance != NULL);
    CHECK(!DaemonIsOnline(instance));

    fake_now_ms = 109U;
    DaemonTask();
    CHECK(!DaemonIsOnline(instance));
    CHECK(offline_count == 0U);

    fake_now_ms = 110U;
    CHECK(!DaemonIsOnline(instance));
    DaemonTask();
    DaemonTask();
    CHECK(offline_count == 1U);

    fake_now_ms = 120U;
    CHECK(DaemonSetTimeout(instance, 40U));
    DaemonReload(instance);
    CHECK(DaemonIsOnline(instance));
    fake_now_ms = 159U;
    CHECK(DaemonIsOnline(instance));
    fake_now_ms = 160U;
    DaemonTask();
    CHECK(!DaemonIsOnline(instance));
    CHECK(offline_count == 2U);
}

static void TestPollRateDoesNotChangeTimeout(void)
{
    unsigned offline_count = 0U;
    const DaemonConfig config = {
        .timeout_ms = 25U,
        .initial_grace_ms = 50U,
        .callback = CountOffline,
        .owner = &offline_count,
    };
    DaemonInstance *instance;

    fake_now_ms = 500U;
    DaemonServiceInit();
    instance = DaemonRegister(&config);
    CHECK(instance != NULL);
    DaemonReload(instance);

    fake_now_ms = 524U;
    CHECK(DaemonIsOnline(instance));
    fake_now_ms = 525U;
    CHECK(!DaemonIsOnline(instance));
    CHECK(offline_count == 0U);
    DaemonTask();
    CHECK(offline_count == 1U);
}

static void TestClockWrap(void)
{
    unsigned offline_count = 0U;
    const DaemonConfig config = {
        .timeout_ms = 10U,
        .initial_grace_ms = 10U,
        .callback = CountOffline,
        .owner = &offline_count,
    };
    DaemonInstance *instance;

    fake_now_ms = UINT32_MAX - 5U;
    DaemonServiceInit();
    instance = DaemonRegister(&config);
    CHECK(instance != NULL);
    DaemonReload(instance);

    fake_now_ms = 3U;
    CHECK(DaemonIsOnline(instance));
    fake_now_ms = 4U;
    CHECK(!DaemonIsOnline(instance));
    DaemonTask();
    CHECK(offline_count == 1U);
}

static void TestOfflineStateStaysLatchedAcrossHalfRange(void)
{
    unsigned offline_count = 0U;
    const DaemonConfig config = {
        .timeout_ms = 10U,
        .initial_grace_ms = 10U,
        .callback = CountOffline,
        .owner = &offline_count,
    };
    DaemonInstance *instance;

    fake_now_ms = 100U;
    DaemonServiceInit();
    instance = DaemonRegister(&config);
    CHECK(instance != NULL);
    DaemonReload(instance);

    fake_now_ms = 110U;
    DaemonTask();
    CHECK(!DaemonIsOnline(instance));
    CHECK(offline_count == 1U);

    /* Serial deadline arithmetic becomes ambiguous this far after expiry. */
    fake_now_ms += UINT32_C(0x80000000);
    CHECK(!DaemonIsOnline(instance));

    DaemonReload(instance);
    CHECK(DaemonIsOnline(instance));
}

static void TestDefaultsCapacityAndNullSafety(void)
{
    const DaemonConfig config = {0};
    DaemonInstance *first = NULL;

    fake_now_ms = 0U;
    DaemonServiceInit();
    for (uint32_t i = 0U; i < DAEMON_MAX_INSTANCES; ++i) {
        DaemonInstance *instance = DaemonRegister(&config);
        CHECK(instance != NULL);
        if (i == 0U) {
            first = instance;
        }
    }
    CHECK(DaemonRegister(&config) == NULL);
    CHECK(DaemonRegister(NULL) == NULL);
    CHECK(!DaemonIsOnline(first));
    DaemonReload(first);

    fake_now_ms = DAEMON_DEFAULT_INITIAL_GRACE_MS - 1U;
    CHECK(DaemonIsOnline(first));
    fake_now_ms = DAEMON_DEFAULT_INITIAL_GRACE_MS;
    CHECK(!DaemonIsOnline(first));
    CHECK(!DaemonIsOnline(NULL));
    DaemonReload(NULL);
    CHECK(!DaemonSetTimeout(NULL, 10U));
    CHECK(!DaemonSetTimeout(first, 0U));
}

static void TestRejectsAmbiguousDeadlines(void)
{
    const DaemonConfig invalid_timeout = {
        .timeout_ms = UINT32_C(0x80000000),
        .initial_grace_ms = 10U,
    };
    const DaemonConfig invalid_grace = {
        .timeout_ms = 10U,
        .initial_grace_ms = UINT32_C(0x80000000),
    };
    const DaemonConfig valid = {
        .timeout_ms = 10U,
        .initial_grace_ms = 10U,
    };

    DaemonServiceInit();
    CHECK(DaemonRegister(&invalid_timeout) == NULL);
    CHECK(DaemonRegister(&invalid_grace) == NULL);
    CHECK(DaemonRegister(&valid) != NULL);
}

int main(void)
{
    TestDeadlineAndReload();
    TestPollRateDoesNotChangeTimeout();
    TestClockWrap();
    TestOfflineStateStaysLatchedAcrossHalfRange();
    TestDefaultsCapacityAndNullSafety();
    TestRejectsAmbiguousDeadlines();

    if (failures != 0U) {
        fprintf(stderr, "%u device-health checks failed\n", failures);
        return 1;
    }

    puts("device-health checks passed");
    return 0;
}
