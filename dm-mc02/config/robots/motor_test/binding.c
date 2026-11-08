#include "binding.h"
#include <stddef.h>
#include "chassis_config.h"
#include "motor_test_config.h"
#if MOTOR_TEST_MODEL == MOTOR_TEST_MODEL_M3508
#define MOTOR(id_,sign_) { .bus=MOTOR_TEST_CAN,.model=MOTOR_TEST_DJI_MODEL,.id=(id_), \
    .loops=RM_DJI_LOOP_ANGLE|RM_DJI_LOOP_SPEED,.reference_sign=(sign_), \
    .output_sign=1,.angle_sign=1,.speed_sign=1,.speed_in_rad_s=true, \
    .feedback_timeout_ms=CHASSIS_DEMO_FEEDBACK_MS,.command_timeout_ms=100U, \
    .angle_pid={.Kp=MOTOR_TEST_POSITION_KP,.MaxOut=MOTOR_TEST_POSITION_MAX_RAD_S}, \
    .speed_pid={.Kp=MOTOR_TEST_SPEED_KP,.Ki=MOTOR_TEST_SPEED_KI, \
        .MaxOut=MOTOR_TEST_CURRENT_LIMIT,.IntegralLimit=MOTOR_TEST_INTEGRAL_LIMIT, \
        .Improve=PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement} }
static const RmDjiConfig motors[]={ MOTOR(MOTOR_TEST_ID,1) };
#endif
const RmBringupBinding rm_bringup_binding={
    .ins={.axes={1,2,3},.calibration_samples=128,.calibration_timeout_us=5000000U,
        .freshness_us=5000U,.min_dt_s=0.00025f,.max_dt_s=0.01f,
        .stationary_gyro_rad_s=0.1f,.gravity_tolerance=0.5f,
        .q1=10.0f,.q2=0.001f,.r=1000000.0f,.lambda=1.0f,.motion_lpf_s=0.0085f},
    .remote={.protocol=RM_REMOTE_SBUS,.timeout_ms=100U,.center=1024,.span=(uint16_t)CHASSIS_DEMO_REMOTE_SPAN,
        .switch_tolerance=200,.switch_levels={1896,1694,1493,554,353,151},
        .right_x=0,.right_y=1,.left_x=3,.left_y=2,.combined_sa_sb=4,
        .combined_sd_sc=5,.knob_left=6,.knob_right=7},
#if MOTOR_TEST_MODEL == MOTOR_TEST_MODEL_M3508
    .motors=motors,.motor_count=1U
#else
    .motors=NULL,.motor_count=0U
#endif
};
