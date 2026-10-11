#include "safety_manager.h"

#include <stddef.h>

void RmSafety_Init(RmSafetyManager *manager)
{
    static const RmSafetyPolicy conservative_policy = {
        .require_explicit_rearm = true,
    };

    RmSafety_InitWithPolicy(manager, &conservative_policy);
}

void RmSafety_InitWithPolicy(RmSafetyManager *manager,
                             const RmSafetyPolicy *policy)
{
    if (manager == NULL) {
        return;
    }

    manager->state = RM_SAFETY_STATE_BOOT;
    manager->reasons = RM_SAFETY_REASON_NOT_READY;
    manager->transition_count = 0U;
    manager->output_permitted = false;
    manager->safe_position_seen = false;
    manager->policy.require_explicit_rearm =
        (policy == NULL) ? true : policy->require_explicit_rearm;
}

bool RmSafety_Update(RmSafetyManager *manager, const RmSafetyInputs *inputs)
{
    RmSafetyState next_state;
    uint32_t next_reasons = RM_SAFETY_REASON_NONE;
    bool next_output_permitted;
    bool changed;

    if (manager == NULL) {
        return false;
    }

    if (inputs == NULL) {
        changed = (manager->state != RM_SAFETY_STATE_STOPPED) ||
                  (manager->reasons != RM_SAFETY_REASON_INVALID_INPUT) ||
                  manager->output_permitted;
        if (changed) {
            manager->state = RM_SAFETY_STATE_STOPPED;
            manager->reasons = RM_SAFETY_REASON_INVALID_INPUT;
            manager->output_permitted = false;
            manager->transition_count++;
        }
        manager->safe_position_seen = false;
        return changed;
    }

    if (!inputs->initialization_complete) {
        next_reasons |= RM_SAFETY_REASON_NOT_READY;
    }
    if (!inputs->input_online) {
        next_reasons |= RM_SAFETY_REASON_INPUT_OFFLINE;
    }
    if (inputs->emergency_stop) {
        next_reasons |= RM_SAFETY_REASON_EMERGENCY_STOP;
    }
    if (!inputs->task_health_ok) {
        next_reasons |= RM_SAFETY_REASON_TASK_UNHEALTHY;
    }
    if (!inputs->device_health_ok) {
        next_reasons |= RM_SAFETY_REASON_DEVICE_UNHEALTHY;
    }

    if (!manager->policy.require_explicit_rearm) {
        manager->safe_position_seen = false;
    } else if (next_reasons != RM_SAFETY_REASON_NONE) {
        manager->safe_position_seen = false;
    } else if (!inputs->operator_enable_request) {
        /*
         * The qualification is valid only while the operator gate is closed
         * and the remaining controls are still in their safe posture.  Do not
         * retain an earlier safe sample after a mode, trigger, or stick moves.
         */
        manager->safe_position_seen = inputs->operator_safe_position;
    } else if (!inputs->operator_safe_position) {
        /* An unsafe arm attempt consumes any earlier qualification. */
        manager->safe_position_seen = false;
    }

    if (next_reasons == RM_SAFETY_REASON_NONE &&
        inputs->operator_enable_request &&
        (manager->output_permitted ||
         !manager->policy.require_explicit_rearm)) {
        next_state = RM_SAFETY_STATE_ACTIVE;
    } else if (next_reasons == RM_SAFETY_REASON_NONE &&
               manager->safe_position_seen &&
               inputs->operator_safe_position &&
               inputs->operator_enable_request &&
               inputs->operator_arm_event) {
        next_state = RM_SAFETY_STATE_ACTIVE;
        manager->safe_position_seen = false;
    } else if ((next_reasons & RM_SAFETY_REASON_NOT_READY) != 0U) {
        next_state = RM_SAFETY_STATE_DISARMED;
    } else if (next_reasons == RM_SAFETY_REASON_NONE) {
        next_state = RM_SAFETY_STATE_DISARMED;
        next_reasons |= RM_SAFETY_REASON_OPERATOR_DISARMED;
        if (inputs->operator_enable_request) {
            next_reasons |= RM_SAFETY_REASON_REARM_REQUIRED;
        }
    } else {
        next_state = RM_SAFETY_STATE_STOPPED;
    }
    next_output_permitted = (next_state == RM_SAFETY_STATE_ACTIVE);

    changed = (manager->state != next_state) ||
              (manager->reasons != next_reasons) ||
              (manager->output_permitted != next_output_permitted);
    if (changed) {
        manager->state = next_state;
        manager->reasons = next_reasons;
        manager->output_permitted = next_output_permitted;
        manager->transition_count++;
    }

    return changed;
}

bool RmSafety_OutputPermitted(const RmSafetyManager *manager)
{
    return (manager != NULL) && manager->output_permitted;
}
