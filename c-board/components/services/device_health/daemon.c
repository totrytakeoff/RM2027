#include "daemon.h"

#include <stddef.h>
#include <stdatomic.h>

#include "rm_time.h"

struct DaemonInstance {
    _Atomic uint32_t timeout_ms;
    _Atomic uint32_t deadline_ms;
    _Atomic uint32_t offline_notified;
    _Atomic uint32_t has_feed;
    DaemonOfflineCallback callback;
    void *owner;
};

static DaemonInstance daemon_instances[DAEMON_MAX_INSTANCES];
static size_t daemon_count;

void DaemonServiceInit(void)
{
    for (size_t i = 0U; i < DAEMON_MAX_INSTANCES; ++i) {
        atomic_store_explicit(&daemon_instances[i].timeout_ms,
                              0U,
                              memory_order_relaxed);
        atomic_store_explicit(&daemon_instances[i].deadline_ms,
                              0U,
                              memory_order_relaxed);
        atomic_store_explicit(&daemon_instances[i].offline_notified,
                              0U,
                              memory_order_relaxed);
        atomic_store_explicit(&daemon_instances[i].has_feed,
                              0U,
                              memory_order_relaxed);
        daemon_instances[i].callback = NULL;
        daemon_instances[i].owner = NULL;
    }
    daemon_count = 0U;
}

DaemonInstance *DaemonRegister(const DaemonConfig *config)
{
    DaemonInstance *instance;
    uint32_t timeout_ms;
    uint32_t initial_grace_ms;

    if ((config == NULL) || (daemon_count >= DAEMON_MAX_INSTANCES)) {
        return NULL;
    }

    timeout_ms = (config->timeout_ms != 0U)
                     ? config->timeout_ms
                     : DAEMON_DEFAULT_TIMEOUT_MS;
    initial_grace_ms = (config->initial_grace_ms != 0U)
                           ? config->initial_grace_ms
                           : DAEMON_DEFAULT_INITIAL_GRACE_MS;
    if ((timeout_ms >= UINT32_C(0x80000000)) ||
        (initial_grace_ms >= UINT32_C(0x80000000))) {
        return NULL;
    }

    instance = &daemon_instances[daemon_count++];
    atomic_store_explicit(&instance->timeout_ms,
                          timeout_ms,
                          memory_order_relaxed);
    atomic_store_explicit(&instance->deadline_ms,
                          RmTime_NowMs() + initial_grace_ms,
                          memory_order_relaxed);
    atomic_store_explicit(&instance->offline_notified,
                          0U,
                          memory_order_relaxed);
    atomic_store_explicit(&instance->has_feed,
                          0U,
                          memory_order_relaxed);
    instance->callback = config->callback;
    instance->owner = config->owner;
    return instance;
}

void DaemonReload(DaemonInstance *instance)
{
    if (instance == NULL) {
        return;
    }

    const uint32_t timeout_ms = atomic_load_explicit(&instance->timeout_ms,
                                                     memory_order_relaxed);
    atomic_store_explicit(&instance->deadline_ms,
                          RmTime_NowMs() + timeout_ms,
                          memory_order_release);
    atomic_store_explicit(&instance->has_feed,
                          1U,
                          memory_order_release);
    atomic_store_explicit(&instance->offline_notified,
                          0U,
                          memory_order_release);
}

bool DaemonSetTimeout(DaemonInstance *instance, uint32_t timeout_ms)
{
    if ((instance == NULL) || (timeout_ms == 0U) ||
        (timeout_ms >= UINT32_C(0x80000000))) {
        return false;
    }

    atomic_store_explicit(&instance->timeout_ms,
                          timeout_ms,
                          memory_order_relaxed);
    return true;
}

bool DaemonIsOnline(const DaemonInstance *instance)
{
    if (instance == NULL) {
        return false;
    }

    if (atomic_load_explicit(&instance->has_feed,
                             memory_order_acquire) == 0U) {
        return false;
    }
    if (atomic_load_explicit(&instance->offline_notified,
                             memory_order_acquire) != 0U) {
        return false;
    }

    const uint32_t deadline_ms = atomic_load_explicit(&instance->deadline_ms,
                                                      memory_order_acquire);
    return !RmTime_DeadlineReached(RmTime_NowMs(), deadline_ms);
}

void DaemonTask(void)
{
    const uint32_t now_ms = RmTime_NowMs();

    for (size_t i = 0U; i < daemon_count; ++i) {
        DaemonInstance *instance = &daemon_instances[i];
        uint32_t expected = 0U;
        uint32_t deadline_ms = atomic_load_explicit(&instance->deadline_ms,
                                                    memory_order_acquire);

        if (!RmTime_DeadlineReached(now_ms, deadline_ms)) {
            continue;
        }
        if (!atomic_compare_exchange_strong_explicit(
                &instance->offline_notified,
                &expected,
                1U,
                memory_order_acq_rel,
                memory_order_acquire)) {
            continue;
        }

        /*
         * A CAN/UART IRQ may feed the instance between the first deadline
         * read and the transition claim. Re-read after claiming so that such
         * a feed cannot suppress the next genuine offline notification.
         */
        deadline_ms = atomic_load_explicit(&instance->deadline_ms,
                                           memory_order_acquire);
        if (!RmTime_DeadlineReached(RmTime_NowMs(), deadline_ms)) {
            atomic_store_explicit(&instance->offline_notified,
                                  0U,
                                  memory_order_release);
            continue;
        }

        if (instance->callback != NULL) {
            instance->callback(instance->owner);
        }
    }
}
