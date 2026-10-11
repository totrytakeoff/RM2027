#include "gimbal_axis_state.h"

#include <math.h>
#include <stddef.h>

#include "gimbal_units.h"

static bool ConfigIsValid(const GimbalAxisStateConfig *config)
{
    return config != NULL &&
           Gimbal_FloatIsFinite(config->brake_speed_epsilon) &&
           config->brake_speed_epsilon >= 0.0f &&
           config->brake_stable_count_required > 0U &&
           config->brake_timeout_ms > 0U;
}

void GimbalAxisState_Init(GimbalAxisState *state)
{
    if (state == NULL) {
        return;
    }
    state->mode = AXIS_CTRL_ANGLE;
    state->brake_stable_count = 0U;
    state->brake_start_ms = 0U;
}

bool GimbalAxisState_Update(
    GimbalAxisState *state,
    const GimbalAxisStateConfig *config,
    bool command_active,
    float speed_feedback,
    uint32_t now_ms,
    GimbalAxisTransition *transition)
{
    AxisCtrlMode_e previous_mode;

    if (state == NULL) {
        return false;
    }
    if (!ConfigIsValid(config) || !Gimbal_FloatIsFinite(speed_feedback)) {
        GimbalAxisState_Init(state);
        return false;
    }
    if (state->mode != AXIS_CTRL_ANGLE &&
        state->mode != AXIS_CTRL_SPEED &&
        state->mode != AXIS_CTRL_BRAKE) {
        GimbalAxisState_Init(state);
        return false;
    }

    previous_mode = state->mode;
    if (command_active) {
        state->mode = AXIS_CTRL_SPEED;
        state->brake_stable_count = 0U;
        state->brake_start_ms = 0U;
    } else if (state->mode == AXIS_CTRL_SPEED) {
        state->mode = AXIS_CTRL_BRAKE;
        state->brake_stable_count = 0U;
        state->brake_start_ms = now_ms;
    } else if (state->mode == AXIS_CTRL_BRAKE) {
        if (fabsf(speed_feedback) <= config->brake_speed_epsilon) {
            if (state->brake_stable_count < UINT8_MAX) {
                state->brake_stable_count++;
            }
        } else {
            state->brake_stable_count = 0U;
        }

        if (state->brake_stable_count >=
                config->brake_stable_count_required ||
            (uint32_t)(now_ms - state->brake_start_ms) >=
                config->brake_timeout_ms) {
            state->mode = AXIS_CTRL_ANGLE;
            state->brake_stable_count = 0U;
            state->brake_start_ms = 0U;
        }
    }

    if (transition != NULL) {
        transition->previous_mode = previous_mode;
        transition->current_mode = state->mode;
        transition->changed = previous_mode != state->mode;
    }
    return true;
}
