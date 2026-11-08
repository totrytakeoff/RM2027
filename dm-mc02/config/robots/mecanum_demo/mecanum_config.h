#ifndef RM_MECANUM_DEMO_CONFIG_H
#define RM_MECANUM_DEMO_CONFIG_H
/* Vehicle top view, front at top: [FL=4 FR=3; BL=1 BR=2].
   Wheel order FR, FL, BR, BL; geometry retains RM2026 defaults. */
#define MECANUM_DEMO_CAN RM_CAN_1
#define MECANUM_DEMO_FR_ID 3U
#define MECANUM_DEMO_FL_ID 4U
#define MECANUM_DEMO_BR_ID 2U
#define MECANUM_DEMO_BL_ID 1U
#define MECANUM_DEMO_FR_SIGN -1
#define MECANUM_DEMO_FL_SIGN 1
#define MECANUM_DEMO_BR_SIGN -1
#define MECANUM_DEMO_BL_SIGN 1
/* Independent X-wheel calibration: body x=right, y=forward, wz=CCW.
   Positive ground speed is forward; mirrored right motors use reference_sign=-1.
   ET08 forward raw is negative: map it to positive forward. Verify on stands. */
#define MECANUM_DEMO_REMOTE_X_SIGN -1.0f
#define MECANUM_DEMO_REMOTE_Y_SIGN -1.0f
#define MECANUM_DEMO_IMU_YAW_SIGN 1.0f
#define MECANUM_DEMO_WHEEL_RADIUS 0.075f
/* X rollers, wheel-centre front/back and left/right distances (metres). */
#define MECANUM_DEMO_WHEEL_LENGTH 0.34f
#define MECANUM_DEMO_WHEEL_WIDTH 0.34f
#define MECANUM_DEMO_ROTATION_ARM (0.5f*(MECANUM_DEMO_WHEEL_LENGTH+MECANUM_DEMO_WHEEL_WIDTH))
#define MECANUM_DEMO_REDUCTION 19.20320856f
#define MECANUM_DEMO_MOTOR_MAX_RAD_S 523.5987756f
#define MECANUM_DEMO_VMAX (MECANUM_DEMO_MOTOR_MAX_RAD_S*MECANUM_DEMO_WHEEL_RADIUS/MECANUM_DEMO_REDUCTION)
#define MECANUM_DEMO_WMAX (MECANUM_DEMO_VMAX/MECANUM_DEMO_ROTATION_ARM)
/* SB 上档小陀螺速度 = WMAX × 此比例；范围 (0,1)，不受跟随 yaw 上限限制。 */
#define MECANUM_DEMO_SPIN_RATIO 0.60f
/* 右摇杆满杆时的目标航向推进速度，单位 rad/s
   松杆后目标航向保持。实际转速还受 YAW_MAX_RAD_S、轮速限幅及负载影响。 */
#define MECANUM_DEMO_YAW_RATE_RAD_S (360.0f*0.017453292519943295f)
/* 跟随控制：wz = Kp×角误差 + Ki×误差积分 - Kd×陀螺角速度 + 目标角速度。
   角误差单位 rad，角速度单位 rad/s；Kp 控制追赶强度，Ki 补偿静差，Kd 提供阻尼。 */
#define MECANUM_DEMO_YAW_KP 20.0f
#define MECANUM_DEMO_YAW_KI 0.0f
#define MECANUM_DEMO_YAW_KD 1.0f
/* FOLLOW/AUTO_AIM 槽位的 yaw 指令绝对上限，单位 rad/
   提高满杆目标速度时需同时检查此上限；不限制 SB 上档的小陀螺速度。 */
#define MECANUM_DEMO_YAW_MAX_RAD_S 10.0f
/* yaw 积分项对角速度指令的最大贡献，单位 rad/s；Ki=0 时积分关闭。 */
#define MECANUM_DEMO_YAW_I_MAX_RAD_S 0.26179939f
#define MECANUM_DEMO_FILTER_OLD 0.15f
#define MECANUM_DEMO_DEADBAND_RAW 100
#define MECANUM_DEMO_REMOTE_SPAN 660.0f
#define MECANUM_DEMO_STARTUP_MS 2000U
#define MECANUM_DEMO_FEEDBACK_MS 20U
#define MECANUM_DEMO_SPEED_KP 229.18312f
#define MECANUM_DEMO_SPEED_KI 2.291831f
#define MECANUM_DEMO_SPEED_MAX_OUT 15000.0f
/* Same SA level gate and individual offline-motor isolation as RM2026. */
#define MECANUM_DEMO_REQUIRE_REARM false
#define MECANUM_DEMO_REQUIRE_ALL_MOTORS false
#define MECANUM_DEMO_SERIAL_BAUD 115200U
/* Diagnostics sample every 10 ms; send one window average every 200 ms. */
#define MECANUM_DEMO_VOFA_PERIOD_MS 200U
/* Telemetry only: nominal voltage times absolute motor-current sum.
   This estimate is not measured DC input power and never limits output. */
#define MECANUM_DEMO_SUPPLY_VOLTAGE 24.0f
#define MECANUM_DEMO_CURRENT_RAW_PER_AMP (16384.0f/20.0f)
#endif
