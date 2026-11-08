#ifndef RM_MOTOR_TEST_CONFIG_H
#define RM_MOTOR_TEST_CONFIG_H
#define MOTOR_TEST_MODEL_M3508 3508
#define MOTOR_TEST_MODEL_DM4310 4310
#ifndef MOTOR_TEST_MODEL
#define MOTOR_TEST_MODEL MOTOR_TEST_MODEL_M3508
#endif
#define MOTOR_TEST_CAN RM_CAN_1
#define MOTOR_TEST_ID 1U
#define MOTOR_TEST_DIRECTION 1.0f
/* Add each future model's driver binding, reduction and gains as a new branch. */
#if MOTOR_TEST_MODEL == MOTOR_TEST_MODEL_M3508
#define MOTOR_TEST_DJI_MODEL RM_DJI_M3508
#define MOTOR_TEST_REDUCTION (268.0f / 19.0f)
#define MOTOR_TEST_MAX_RAD_S 523.5987756f
/* Speed PI: ESC command / (rotor rad/s), integral gain per second. */
#define MOTOR_TEST_SPEED_KP 20.0f
#define MOTOR_TEST_SPEED_KI 80.0f
#define MOTOR_TEST_CURRENT_LIMIT 16384.0f
#define MOTOR_TEST_INTEGRAL_LIMIT 5000.0f
#elif MOTOR_TEST_MODEL == MOTOR_TEST_MODEL_DM4310
/* Protocol uses OUTPUT SHAFT units; do not multiply by the physical 10:1 gearbox. */
#define MOTOR_TEST_REDUCTION 1.0f
#define MOTOR_TEST_DM_MASTER_ID 0U
#define MOTOR_TEST_MAX_RAD_S 20.943951024f /* 200 rpm: 24V datasheet no-load speed */
#define MOTOR_TEST_DM_TORQUE_LIMIT 7.0f /* V1.1 peak; update for confirmed hardware */
#define MOTOR_TEST_SPEED_KP 0.2f
#define MOTOR_TEST_SPEED_KI 0.5f
#define MOTOR_TEST_CURRENT_LIMIT MOTOR_TEST_DM_TORQUE_LIMIT /* Nm, not C620 units */
#define MOTOR_TEST_INTEGRAL_LIMIT MOTOR_TEST_DM_TORQUE_LIMIT
#else
#error "Unsupported MOTOR_TEST_MODEL: add and validate its binding and parameters first"
#endif
/* Output shaft travel from the captured zero: -pi .. +pi radians.
   Native DJI angle loop takes rotor degrees and outputs rotor rad/s. */
#define MOTOR_TEST_POSITION_RANGE_RAD 3.14159265359f
#define MOTOR_TEST_POSITION_KP (4.0f * 0.017453292519943295f)
#define MOTOR_TEST_POSITION_MAX_RAD_S MOTOR_TEST_MAX_RAD_S
#endif
