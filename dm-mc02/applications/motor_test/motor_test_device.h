#ifndef RM_MOTOR_TEST_DEVICE_H
#define RM_MOTOR_TEST_DEVICE_H
#include "bringup_app.h"
#include "motor_test_config.h"
#if MOTOR_TEST_MODEL == MOTOR_TEST_MODEL_M3508
static inline void RmMotorTest_DeviceInit(RmRuntimeConfig *runtime) { (void)runtime; }
static inline bool RmMotorTest_DeviceRead(RmDjiFeedback *out) { return RmBringup_ReadMotor(0U,out); }
static inline bool RmMotorTest_DeviceZero(void) { return RmBringup_ZeroMotorPosition(0U); }
static inline bool RmMotorTest_DeviceSubmit(const RmDjiCommand *cmd) { return RmBringup_SubmitMotorCommands(cmd,1U,cmd->epoch); }
#else
void RmMotorTest_DeviceInit(RmRuntimeConfig *runtime);
bool RmMotorTest_DeviceRead(RmDjiFeedback *out);
bool RmMotorTest_DeviceZero(void);
bool RmMotorTest_DeviceSubmit(const RmDjiCommand *cmd);
/* Bitmask of received PMAX, VMAX, TMAX, control mode and timeout. */
uint32_t RmMotorTest_DmConfigMask(void);
uint32_t RmMotorTest_DmFaults(void);
float RmMotorTest_DmTorque(void);
#endif
#endif
