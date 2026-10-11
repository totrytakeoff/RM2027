#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RM_SAFETY_STATE_BOOT = 0,
    RM_SAFETY_STATE_DISARMED,
    RM_SAFETY_STATE_ACTIVE,
    RM_SAFETY_STATE_STOPPED,
} RmSafetyState;

typedef enum {
    RM_SAFETY_REASON_NONE = 0U,
    RM_SAFETY_REASON_NOT_READY = (1U << 0),
    RM_SAFETY_REASON_INPUT_OFFLINE = (1U << 1),
    RM_SAFETY_REASON_EMERGENCY_STOP = (1U << 2),
    RM_SAFETY_REASON_TASK_UNHEALTHY = (1U << 3),
    RM_SAFETY_REASON_INVALID_INPUT = (1U << 4),
    RM_SAFETY_REASON_DEVICE_UNHEALTHY = (1U << 5),
    RM_SAFETY_REASON_OPERATOR_DISARMED = (1U << 6),
    RM_SAFETY_REASON_REARM_REQUIRED = (1U << 7),
} RmSafetyReason;

typedef struct {
    bool initialization_complete;
    bool input_online;
    bool emergency_stop;
    bool operator_enable_request;
    /** True while every control except the enable gate is in its safe posture. */
    bool operator_safe_position;
    bool operator_arm_event;
    bool task_health_ok;
    bool device_health_ok;
} RmSafetyInputs;

typedef struct {
    /** Require a safe-position sample followed by a fresh arm event. */
    bool require_explicit_rearm;
} RmSafetyPolicy;

typedef struct {
    RmSafetyState state;
    uint32_t reasons;
    uint32_t transition_count;
    bool output_permitted;
    bool safe_position_seen;
    RmSafetyPolicy policy;
} RmSafetyManager;

/** Initialize a manager with the conservative explicit-rearm policy. */
void RmSafety_Init(RmSafetyManager *manager);

/** Initialize a manager with an application-selected arming policy. */
void RmSafety_InitWithPolicy(RmSafetyManager *manager,
                             const RmSafetyPolicy *policy);

/**
 * Evaluate all safety inputs. Returns true only when observable state changed.
 * A null input snapshot transitions a valid manager to a stopped fail-safe.
 *
 * With require_explicit_rearm enabled, every boot or fault requires an
 * explicit safe-position observation while the enable gate is closed,
 * followed by a new arm event. With it disabled, the operator enable request
 * acts as a level gate and activation resumes as soon as hard faults clear.
 */
bool RmSafety_Update(RmSafetyManager *manager, const RmSafetyInputs *inputs);

/** Outputs may be updated only while the manager is active. */
bool RmSafety_OutputPermitted(const RmSafetyManager *manager);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MANAGER_H */
