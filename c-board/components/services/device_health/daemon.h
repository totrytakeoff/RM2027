#ifndef RM_DEVICE_HEALTH_DAEMON_H
#define RM_DEVICE_HEALTH_DAEMON_H

#include <stdbool.h>
#include <stdint.h>

#define DAEMON_MAX_INSTANCES 64U
#define DAEMON_DEFAULT_TIMEOUT_MS 1000U
#define DAEMON_DEFAULT_INITIAL_GRACE_MS 1000U

typedef struct DaemonInstance DaemonInstance;
typedef void (*DaemonOfflineCallback)(void *owner);

typedef struct {
    /** Maximum interval between valid feeds. Zero selects the default. */
    uint32_t timeout_ms;
    /** Startup grace period. Zero selects the default. */
    uint32_t initial_grace_ms;
    DaemonOfflineCallback callback;
    void *owner;
} DaemonConfig;

/** Reset the fixed-capacity registry. Call only during single-threaded startup. */
void DaemonServiceInit(void);

/** Register a health deadline during single-threaded startup. */
DaemonInstance *DaemonRegister(const DaemonConfig *config);

/** Feed an instance using the current monotonic millisecond clock. */
void DaemonReload(DaemonInstance *instance);

/** Change the timeout used by future feeds without altering the current grace. */
bool DaemonSetTimeout(DaemonInstance *instance, uint32_t timeout_ms);

/** Return whether the instance has been fed and is inside its deadline. */
bool DaemonIsOnline(const DaemonInstance *instance);

/** Poll all instances and dispatch each offline transition once. */
void DaemonTask(void);

#endif /* RM_DEVICE_HEALTH_DAEMON_H */
