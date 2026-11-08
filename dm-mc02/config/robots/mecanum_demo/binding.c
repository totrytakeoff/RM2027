#include "binding.h"
#include "mecanum_config.h"
#define MOTOR(id_,sign_) { .bus=MECANUM_DEMO_CAN,.model=RM_DJI_M3508,.id=(id_), \
    .loops=RM_DJI_LOOP_SPEED|RM_DJI_LOOP_CURRENT,.reference_sign=(sign_), \
    .output_sign=1,.angle_sign=1,.speed_sign=1,.speed_in_rad_s=true, \
    .feedback_timeout_ms=MECANUM_DEMO_FEEDBACK_MS,.command_timeout_ms=100U, \
    .speed_pid={.Kp=MECANUM_DEMO_SPEED_KP,.Ki=MECANUM_DEMO_SPEED_KI, \
        .MaxOut=MECANUM_DEMO_SPEED_MAX_OUT,.IntegralLimit=1000.0f, \
        .Improve=PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement}, \
    .current_pid={.Kp=0.35f,.MaxOut=15000.0f,.IntegralLimit=1000.0f, \
        .Improve=PID_Trapezoid_Intergral|PID_Integral_Limit} }
static const RmDjiConfig motors[]={
    MOTOR(MECANUM_DEMO_FR_ID,MECANUM_DEMO_FR_SIGN),
    MOTOR(MECANUM_DEMO_FL_ID,MECANUM_DEMO_FL_SIGN),
    MOTOR(MECANUM_DEMO_BR_ID,MECANUM_DEMO_BR_SIGN),
    MOTOR(MECANUM_DEMO_BL_ID,MECANUM_DEMO_BL_SIGN)
};
const RmBringupBinding rm_bringup_binding={
    .ins={.axes={1,2,3},.calibration_samples=128,.calibration_timeout_us=5000000U,
        .freshness_us=5000U,.min_dt_s=0.00025f,.max_dt_s=0.01f,
        .stationary_gyro_rad_s=0.1f,.gravity_tolerance=0.5f,
        .q1=10.0f,.q2=0.001f,.r=1000000.0f,.lambda=1.0f,.motion_lpf_s=0.0085f},
    .remote={.protocol=RM_REMOTE_SBUS,.timeout_ms=100U,.center=1024,.span=(uint16_t)MECANUM_DEMO_REMOTE_SPAN,
        .switch_tolerance=200,.switch_levels={1896,1694,1493,554,353,151},
        .right_x=0,.right_y=1,.left_x=3,.left_y=2,.combined_sa_sb=4,
        .combined_sd_sc=5,.knob_left=6,.knob_right=7},
    .motors=motors,.motor_count=4U
};
