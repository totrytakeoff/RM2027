#include "binding.h"
const RmBringupBinding rm_bringup_binding={
    .ins={.axes={1,2,3},.calibration_samples=128,.calibration_timeout_us=5000000U,
        .freshness_us=5000U,.min_dt_s=0.00025f,.max_dt_s=0.01f,
        .stationary_gyro_rad_s=0.1f,.gravity_tolerance=0.5f,
        .q1=10.0f,.q2=0.001f,.r=1000000.0f,.lambda=1.0f,.accel_lpf_s=0.0f,.motion_lpf_s=0.0085f},
    .remote={
#if RM_UART5_DBUS
        .protocol=RM_REMOTE_DBUS,
#else
        .protocol=RM_REMOTE_SBUS,
#endif
        .timeout_ms=100U,.center=1024,.span=660,.switch_tolerance=200,
        .switch_levels={1896,1694,1493,554,353,151},
        .right_x=0,.right_y=1,.left_x=3,.left_y=2,.combined_sa_sb=4,.combined_sd_sc=5,.knob_left=6,.knob_right=7},
    /* No actual motor wiring has been confirmed. Passive candidate monitoring
       does not register actuators or send zero frames to guessed IDs. */
    .motors=NULL,.motor_count=0
};
