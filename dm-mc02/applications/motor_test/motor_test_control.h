#ifndef RM_MOTOR_TEST_CONTROL_H
#define RM_MOTOR_TEST_CONTROL_H
#include "remote_control_state.h"
#include "dji.h"
typedef enum { MOTOR_TEST_OFF, MOTOR_TEST_SPEED, MOTOR_TEST_POSITION } MotorTestMode;
MotorTestMode RmMotorTest_Mode(const RemoteControlState *remote);
/* Returns rotor rad/s in speed mode, rotor degrees in position mode. */
float RmMotorTest_Reference(const RemoteControlState *remote, MotorTestMode mode);
#endif
