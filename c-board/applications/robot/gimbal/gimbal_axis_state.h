#ifndef ROBOT_GIMBAL_AXIS_STATE_H
#define ROBOT_GIMBAL_AXIS_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float brake_speed_epsilon;
    uint8_t brake_stable_count_required;
    uint32_t brake_timeout_ms;
} GimbalAxisStateConfig;

typedef struct {
    AxisCtrlMode_e mode;
    uint8_t brake_stable_count;
    uint32_t brake_start_ms;
} GimbalAxisState;

typedef struct {
    AxisCtrlMode_e previous_mode;
    AxisCtrlMode_e current_mode;
    bool changed;
} GimbalAxisTransition;

void GimbalAxisState_Init(GimbalAxisState *state);

bool GimbalAxisState_Update(
    GimbalAxisState *state,
    const GimbalAxisStateConfig *config,
    bool command_active,
    float speed_feedback,
    uint32_t now_ms,
    GimbalAxisTransition *transition);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_GIMBAL_AXIS_STATE_H */
