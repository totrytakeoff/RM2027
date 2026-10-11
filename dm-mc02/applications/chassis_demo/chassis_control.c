/* ET08 intent, filtering and wheel mixing ported from RM2026 robot/chassis.
   A standalone IMU heading target replaces the unavailable gimbal encoder. */
#include "chassis_control.h"
#include "chassis_config.h"
#include "chassis_kinematics.h"
#include "chassis_follow.h"
#include <math.h>
#include <string.h>
#define RAD_PER_DEG 0.017453292519943295f
#define TWO_PI 6.283185307179586f
bool RmChassis_ConfigValid(void)
{
    return isfinite(CHASSIS_DEMO_VMAX) && CHASSIS_DEMO_VMAX>0.0f &&
        isfinite(CHASSIS_DEMO_WMAX) && CHASSIS_DEMO_WMAX>0.0f &&
        isfinite(CHASSIS_DEMO_WHEEL_RADIUS) && CHASSIS_DEMO_WHEEL_RADIUS>0.0f &&
        isfinite(CHASSIS_DEMO_WHEEL_BASE) && CHASSIS_DEMO_WHEEL_BASE>0.0f &&
        isfinite(CHASSIS_DEMO_REDUCTION) && CHASSIS_DEMO_REDUCTION>0.0f &&
        isfinite(CHASSIS_DEMO_MOTOR_MAX_RAD_S) && CHASSIS_DEMO_MOTOR_MAX_RAD_S>0.0f &&
        CHASSIS_DEMO_SPIN_RATIO>0.0f && CHASSIS_DEMO_SPIN_RATIO<1.0f &&
        CHASSIS_DEMO_FILTER_OLD>=0.0f && CHASSIS_DEMO_FILTER_OLD<1.0f &&
        CHASSIS_DEMO_DEADBAND_RAW>=0 && CHASSIS_DEMO_DEADBAND_RAW<CHASSIS_DEMO_REMOTE_SPAN &&
        CHASSIS_DEMO_REMOTE_SPAN<=2047.0f &&
        (CHASSIS_DEMO_REMOTE_X_SIGN==1.0f || CHASSIS_DEMO_REMOTE_X_SIGN==-1.0f) &&
        (CHASSIS_DEMO_REMOTE_Y_SIGN==1.0f || CHASSIS_DEMO_REMOTE_Y_SIGN==-1.0f) &&
        (CHASSIS_DEMO_IMU_YAW_SIGN==1.0f || CHASSIS_DEMO_IMU_YAW_SIGN==-1.0f) &&
        isfinite(CHASSIS_DEMO_YAW_RATE_RAD_S) && CHASSIS_DEMO_YAW_RATE_RAD_S>0.0f &&
        isfinite(CHASSIS_DEMO_YAW_KP) && isfinite(CHASSIS_DEMO_YAW_KI) &&
        isfinite(CHASSIS_DEMO_YAW_KD) && CHASSIS_DEMO_YAW_MAX_RAD_S>0.0f &&
        isfinite(CHASSIS_DEMO_YAW_MAX_RAD_S) && CHASSIS_DEMO_YAW_I_MAX_RAD_S>=0.0f &&
        isfinite(CHASSIS_DEMO_YAW_I_MAX_RAD_S);
}
static float clamp(float v,float limit) { return fmaxf(-limit,fminf(limit,v)); }
static float stick(float value)
{
    if (!isfinite(value)) return 0.0f;
    float threshold=(float)CHASSIS_DEMO_DEADBAND_RAW/CHASSIS_DEMO_REMOTE_SPAN;
    float magnitude=fabsf(value);
    if (magnitude<=threshold) return 0.0f;
    return copysignf(fminf(1.0f,(magnitude-threshold)/(1.0f-threshold)),value);
}
bool RmChassis_MapEt08(const RemoteControlState *r,RmChassisIntent *out)
{
    if (!out) return false;
    memset(out,0,sizeof(*out));
    if (!r || r->type!=REMOTE_CONTROL_TYPE_ET08 || !r->link_online || !r->data_valid ||
        r->failsafe || r->frame_lost || (r->axis_valid_mask&0xfU)!=0xfU ||
        (r->switch_valid_mask&0xfU)!=0xfU) return false;
    for (unsigned i=0;i<4U;++i) {
        if (!isfinite(r->axis_normalized[i]) || r->switches[i]<REMOTE_SWITCH_UP ||
            r->switches[i]>REMOTE_SWITCH_DOWN) return false;
    }
    out->valid=true; out->enable=r->switches[0]==REMOTE_SWITCH_UP;
    out->arm_event=out->enable && (r->switch_changed_mask&1U)!=0U;
    out->safe_position=r->switches[1]==REMOTE_SWITCH_DOWN && r->switches[2]==REMOTE_SWITCH_DOWN &&
        r->switches[3]==REMOTE_SWITCH_DOWN;
    for (unsigned i=0;i<4U;++i) if (r->axis_raw[i]>CHASSIS_DEMO_DEADBAND_RAW ||
        r->axis_raw[i]<-CHASSIS_DEMO_DEADBAND_RAW) out->safe_position=false;
    out->mode=r->switches[1]==REMOTE_SWITCH_UP?RM_CHASSIS_SPIN:
        r->switches[1]==REMOTE_SWITCH_MIDDLE?RM_CHASSIS_AUTO_AIM:RM_CHASSIS_FOLLOW;
    if (out->enable) {
        out->x=CHASSIS_DEMO_REMOTE_X_SIGN*stick(r->axis_normalized[REMOTE_AXIS_LEFT_X]);
        out->y=CHASSIS_DEMO_REMOTE_Y_SIGN*stick(r->axis_normalized[REMOTE_AXIS_LEFT_Y]);
        out->yaw=stick(r->axis_normalized[REMOTE_AXIS_RIGHT_X]);
    }
    return true;
}
void RmChassis_Reset(RmChassisController *c)
{
    if (c) { memset(c,0,sizeof(*c)); c->recovery_pending=true; }
}
bool RmChassis_Step(RmChassisController *c,const RmChassisIntent *intent,
    const RmInsSnapshot *imu,float dt,RmChassisOutput *out)
{
    if (!out) return false;
    memset(out,0,sizeof(*out));
    if (!RmChassis_ConfigValid() || !c || !intent || !intent->valid || !intent->enable || !imu || !imu->valid ||
        !isfinite(dt) || dt<0.001f || dt>0.05f || !isfinite(imu->yaw_total_deg) ||
        !isfinite(imu->gyro[2]) || !isfinite(intent->x) || !isfinite(intent->y) ||
        !isfinite(intent->yaw) || (unsigned)intent->mode>RM_CHASSIS_SPIN ||
        (c->active && (!isfinite(c->target_yaw_rad) || !isfinite(c->filtered_vx) ||
            !isfinite(c->filtered_vy) || !isfinite(c->filtered_wz) || !isfinite(c->integral))))
        { RmChassis_Reset(c); return false; }
    float yaw=imu->yaw_total_deg*RAD_PER_DEG*CHASSIS_DEMO_IMU_YAW_SIGN;
    float rate=imu->gyro[2]*CHASSIS_DEMO_IMU_YAW_SIGN;
    if (!c->active) {
        RmChassis_Reset(c); c->active=true; c->target_yaw_rad=yaw; c->mode=intent->mode;
    }
    if (intent->mode!=c->mode) {
        if (c->mode==RM_CHASSIS_SPIN) c->recovery_pending=true;
        c->filtered_wz=0.0f; c->integral=0.0f; c->mode=intent->mode;
    }
    float target_rate=clamp(intent->yaw,1.0f)*CHASSIS_DEMO_YAW_RATE_RAD_S;
    c->target_yaw_rad+=target_rate*dt;
    float error=remainderf(c->target_yaw_rad-yaw,TWO_PI);
    c->target_yaw_rad=yaw+error; /* nearest equivalent heading, no multi-turn catch-up */
    float vx=clamp(intent->x,1.0f)*CHASSIS_DEMO_VMAX;
    float vy=clamp(intent->y,1.0f)*CHASSIS_DEMO_VMAX;
    Chassis_LimitTranslation(&vx,&vy,CHASSIS_DEMO_VMAX);
    Chassis_RotateToBody(vx,vy,error,&vx,&vy);
    if (fabsf(vx)<0.08f) vx=0.0f;
    if (fabsf(vy)<0.08f) vy=0.0f;
    bool follow=intent->mode!=RM_CHASSIS_SPIN;
    float wz;
    if (follow) {
        if (fabsf(CHASSIS_DEMO_YAW_KI)>1e-6f)
            c->integral=clamp(c->integral+error*dt,CHASSIS_DEMO_YAW_I_MAX_RAD_S/fabsf(CHASSIS_DEMO_YAW_KI));
        else c->integral=0.0f;
        ChassisFollowOutput feedback;
        /* Measured-rate damping avoids a derivative kick on stick changes.
           Target rate is a separate feedforward term, not differentiated. */
        if (!Chassis_CalculateFollowOutput(error,-rate,c->integral,
            CHASSIS_DEMO_YAW_KP,CHASSIS_DEMO_YAW_KI,CHASSIS_DEMO_YAW_KD,
            CHASSIS_DEMO_YAW_MAX_RAD_S,&feedback)) { RmChassis_Reset(c); return false; }
        wz=clamp(feedback.raw_wz_rad_s+target_rate,CHASSIS_DEMO_YAW_MAX_RAD_S);
    } else {
        c->integral=0.0f; wz=CHASSIS_DEMO_WMAX*CHASSIS_DEMO_SPIN_RATIO;
    }
    if (fabsf(wz)<0.10f) wz=0.0f;
    wz=clamp(wz,CHASSIS_DEMO_WMAX);
    const float old=CHASSIS_DEMO_FILTER_OLD;
    if (vx==0.0f && vy==0.0f && wz==0.0f) c->filtered_vx=c->filtered_vy=c->filtered_wz=0.0f;
    else {
        c->filtered_vx=c->filtered_vx*old+vx*(1.0f-old);
        c->filtered_vy=c->filtered_vy*old+vy*(1.0f-old);
        c->filtered_wz=follow && c->recovery_pending?wz:c->filtered_wz*old+wz*(1.0f-old);
    }
    if (follow) c->recovery_pending=false;
    float translation[4],rotation[4];
    Chassis_OmniInverse(c->filtered_vx,c->filtered_vy,0.0f,CHASSIS_DEMO_WHEEL_BASE,CHASSIS_DEMO_WHEEL_RADIUS,translation);
    Chassis_OmniInverse(0.0f,0.0f,c->filtered_wz,CHASSIS_DEMO_WHEEL_BASE,CHASSIS_DEMO_WHEEL_RADIUS,rotation);
    for (unsigned i=0;i<4U;++i) { translation[i]*=CHASSIS_DEMO_REDUCTION; rotation[i]*=CHASSIS_DEMO_REDUCTION; }
    out->translation_scale=1.0f;
    if (!follow) {
        if (!Chassis_CombineWheelSpeedsPreserveRotation(translation,rotation,CHASSIS_DEMO_MOTOR_MAX_RAD_S,
            out->wheel_ref_rad_s,&out->translation_scale)) { RmChassis_Reset(c); return false; }
    } else {
        for (unsigned i=0;i<4U;++i) out->wheel_ref_rad_s[i]=translation[i]+rotation[i];
        Chassis_NormalizeWheelSpeeds(out->wheel_ref_rad_s,CHASSIS_DEMO_MOTOR_MAX_RAD_S);
    }
    for (unsigned i=0;i<4U;++i) if (fabsf(out->wheel_ref_rad_s[i])<(follow?0.34906585f:2.09439510f)) out->wheel_ref_rad_s[i]=0.0f;
    out->yaw_rad=yaw; out->target_yaw_rad=c->target_yaw_rad; out->yaw_error_rad=error;
    out->yaw_rate_rad_s=rate; out->vx_m_s=c->filtered_vx; out->vy_m_s=c->filtered_vy;
    out->wz_rad_s=c->filtered_wz; out->mode=c->mode;
    return true;
}
