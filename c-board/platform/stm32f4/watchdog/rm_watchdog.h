#ifndef RM_WATCHDOG_H
#define RM_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Start the independent hardware watchdog.
 *
 * timeout_ms is a guaranteed minimum based on a conservative 48 kHz LSI
 * upper bound.  The real reset delay is normally longer because the LSI is
 * deliberately imprecise.
 */
bool RmWatchdog_Start(uint32_t timeout_ms);

/** Refresh only after all application-level health checks have passed. */
bool RmWatchdog_Refresh(void);

bool RmWatchdog_IsActive(void);
bool RmWatchdog_WasReset(void);

#endif /* RM_WATCHDOG_H */
