#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_types.h"
#include "safety_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the robot modules. Call before the scheduler starts. */
bool RobotApp_Init(void);

/** Execute the 5 ms actuator stage from the dedicated motor task. */
void RobotApp_MotorStep(void);

/** Execute one deterministic high-level control step. */
void RobotApp_ControlStep(uint32_t now_ms);

/** Execute non-critical, rate-limited diagnostics. */
void RobotApp_DiagnosticsStep(uint32_t now_ms);

/** Send one dedicated, non-blocking tuning telemetry frame. */
void RobotApp_TuningTelemetryStep(uint32_t now_ms);

/** Immediately place every actuator module in its stopped state. */
void RobotApp_ForceSafeStop(void);

/** Publish scheduler health; the control task owns the safety transition. */
void RobotApp_SetTaskHealth(bool healthy);

/** Current safety state for telemetry and hardware acceptance checks. */
RmSafetyState RobotApp_GetSafetyState(void);

/** Active safety reasons as an RmSafetyReason bit mask. */
uint32_t RobotApp_GetSafetyReasons(void);

/** Read-only access for transitional diagnostics. */
const Robot_Context_t *RobotApp_GetContext(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_APP_H */
