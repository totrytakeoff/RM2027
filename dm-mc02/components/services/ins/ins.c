/* RM2026 INS/EKF flow, with bounded stationary initialization and explicit time.
   Original INS/EKF authorship: Wang Hongxi; EKF source retains its own header. */
#include "ins.h"
#include <math.h>
#include <string.h>
static float norm3(const float v[3]) { return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
static bool axes_valid(const int8_t axes[3])
{
    unsigned p[3]; int sign=1;
    for (unsigned i=0;i<3U;++i) {
        int a=axes[i]; if (a==0 || a < -3 || a>3) return false;
        p[i]=(unsigned)(a<0?-a:a)-1U; if (a<0) sign=-sign;
    }
    if (p[0]==p[1] || p[1]==p[2] || p[0]==p[2]) return false;
    for (unsigned i=0;i<3U;++i) for (unsigned j=i+1U;j<3U;++j) if (p[i]>p[j]) sign=-sign;
    return sign==1;
}
void RmIns_Fail(RmIns *s,RmInsError error)
{
    if (!s) return;
    s->snapshot.state=RM_INS_FAILED; s->snapshot.error=error; s->snapshot.valid=false;
}
bool RmIns_Init(RmIns *s,const RmInsConfig *c,uint64_t now_us)
{
    if (!s) return false;
    memset(s,0,sizeof(*s)); s->start_us=now_us;
    if (!c || !axes_valid(c->axes) || !c->calibration_samples || c->calibration_samples>4096U ||
        !c->calibration_timeout_us || !c->freshness_us ||
        !isfinite(c->min_dt_s) || !isfinite(c->max_dt_s) || c->min_dt_s<=0 ||
        c->max_dt_s<c->min_dt_s || c->max_dt_s>0.02f ||
        !isfinite(c->stationary_gyro_rad_s) || c->stationary_gyro_rad_s<=0 ||
        !isfinite(c->gravity_tolerance) || c->gravity_tolerance<=0 || c->gravity_tolerance>2.0f ||
        !isfinite(c->q1) || c->q1<0 || !isfinite(c->q2) || c->q2<0 ||
        !isfinite(c->r) || c->r<=0 || !isfinite(c->lambda) || c->lambda<=0 || c->lambda>1 ||
        !isfinite(c->accel_lpf_s) || c->accel_lpf_s<0 ||
        !isfinite(c->motion_lpf_s) || c->motion_lpf_s<0) {
        RmIns_Fail(s,RM_INS_ERROR_CONFIG); return false;
    }
    s->config=*c; return true;
}
void RmIns_Poll(RmIns *s,uint64_t now_us)
{
    if (!s) return;
    if (s->snapshot.state==RM_INS_CALIBRATING &&
        (now_us<s->start_us || now_us-s->start_us>s->config.calibration_timeout_us))
        RmIns_Fail(s,RM_INS_ERROR_TIMEOUT);
    if (s->snapshot.valid && (now_us<s->snapshot.timestamp_us ||
        now_us-s->snapshot.timestamp_us>s->config.freshness_us)) s->snapshot.valid=false;
}
static void rotate(const float q[4],const float v[3],float out[3])
{
    float a=q[0],x=q[1],y=q[2],z=q[3];
    out[0]=(1-2*y*y-2*z*z)*v[0]+2*(x*y-a*z)*v[1]+2*(x*z+a*y)*v[2];
    out[1]=2*(x*y+a*z)*v[0]+(1-2*x*x-2*z*z)*v[1]+2*(y*z-a*x)*v[2];
    out[2]=2*(x*z-a*y)*v[0]+2*(y*z+a*x)*v[1]+(1-2*x*x-2*y*y)*v[2];
}
static bool initialize_filter(RmIns *s)
{
    float acc[3];
    for (unsigned i=0;i<3U;++i) {
        acc[i]=s->accel_sum[i]/(float)s->snapshot.calibration_count;
        s->bias[i]=s->gyro_sum[i]/(float)s->snapshot.calibration_count;
    }
    float n=norm3(acc); if (!isfinite(n) || n<1e-6f) return false;
    for (unsigned i=0;i<3U;++i) acc[i]/=n;
    /* Shortest rotation from measured gravity to earth +Z; yaw is relative. */
    float q[4]={1.0f+acc[2],acc[1],-acc[0],0};
    float qn=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]);
    if (qn<1e-6f) { q[0]=0; q[1]=1; q[2]=0; }
    else for (unsigned i=0;i<4U;++i) q[i]/=qn;
    return IMU_QuaternionEKF_Init(&s->filter,q,s->config.q1,s->config.q2,
        s->config.r,s->config.lambda,s->config.accel_lpf_s);
}
void RmIns_Push(RmIns *s,const RmImuSample *raw,uint64_t now_us)
{
    if (!s || !raw) return;
    RmIns_Poll(s,now_us);
    if (s->snapshot.state==RM_INS_FAILED) return;
    if (!raw->valid) { s->snapshot.valid=false; return; }
    if (s->have_raw && raw->sequence==s->raw_sequence) return;
    if (now_us<raw->gyro_us || now_us<raw->accel_us ||
        now_us-raw->gyro_us>s->config.freshness_us || now_us-raw->accel_us>s->config.freshness_us) {
        s->snapshot.valid=false; return;
    }
    float accel[3],gyro[3];
    for (unsigned i=0;i<3U;++i) {
        int a=s->config.axes[i]; unsigned j=(unsigned)(a<0?-a:a)-1U;
        accel[i]=raw->accel[j]*(a<0?-1.0f:1.0f);
        gyro[i]=raw->gyro[j]*(a<0?-1.0f:1.0f);
        if (!isfinite(accel[i]) || !isfinite(gyro[i])) { RmIns_Fail(s,RM_INS_ERROR_NUMERIC); return; }
    }
    float an=norm3(accel),gn=norm3(gyro);
    if (!isfinite(an) || !isfinite(gn) || an<0.01f || an>120.0f || gn>80.0f ||
        !isfinite(raw->temperature_c)) { RmIns_Fail(s,RM_INS_ERROR_NUMERIC); return; }
    float dt=0;
    if (s->have_raw) {
        if (raw->gyro_us<=s->last_us) { RmIns_Fail(s,RM_INS_ERROR_TIME); return; }
        dt=(float)(raw->gyro_us-s->last_us)*1e-6f;
        if (dt<s->config.min_dt_s || dt>s->config.max_dt_s) {
            if (s->snapshot.state==RM_INS_RUNNING) { RmIns_Fail(s,RM_INS_ERROR_TIME); return; }
            s->snapshot.calibration_count=0;
            memset(s->accel_sum,0,sizeof(s->accel_sum)); memset(s->gyro_sum,0,sizeof(s->gyro_sum));
        }
    }
    s->have_raw=true; s->raw_sequence=raw->sequence; s->last_us=raw->gyro_us;
    RmInsSnapshot *v=&s->snapshot;
    if (v->state==RM_INS_CALIBRATING) {
        if (fabsf(an-9.80665f)>s->config.gravity_tolerance || gn>s->config.stationary_gyro_rad_s) {
            v->calibration_count=0; memset(s->accel_sum,0,sizeof(s->accel_sum)); memset(s->gyro_sum,0,sizeof(s->gyro_sum)); return;
        }
        for (unsigned i=0;i<3U;++i) { s->accel_sum[i]+=accel[i]; s->gyro_sum[i]+=gyro[i]; }
        if (++v->calibration_count<s->config.calibration_samples) return;
        if (!initialize_filter(s)) { RmIns_Fail(s,RM_INS_ERROR_NUMERIC); return; }
        v->state=RM_INS_RUNNING; return; /* First integration uses next sample's dt. */
    }
    for (unsigned i=0;i<3U;++i) gyro[i]-=s->bias[i];
    if (!IMU_QuaternionEKF_Update(&s->filter,gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],dt)) {
        RmIns_Fail(s,RM_INS_ERROR_NUMERIC); return;
    }
    for (unsigned i=0;i<36U;++i) if (!isfinite(s->filter.IMU_QuaternionEKF.P_data[i])) {
        RmIns_Fail(s,RM_INS_ERROR_NUMERIC); return;
    }
    memcpy(v->quaternion,s->filter.q,sizeof(v->quaternion));
    float conjugate[4]={v->quaternion[0],-v->quaternion[1],-v->quaternion[2],-v->quaternion[3]};
    const float gravity[3]={0,0,9.80665f}; float gb[3]; rotate(conjugate,gravity,gb);
    float weight=dt/(s->config.motion_lpf_s+dt);
    for (unsigned i=0;i<3U;++i) {
        v->accel[i]=accel[i]; v->gyro_bias[i]=s->bias[i]+s->filter.GyroBias[i];
        v->gyro[i]=s->filter.Gyro[i];
        v->motion_body[i]+=weight*(accel[i]-gb[i]-v->motion_body[i]);
    }
    rotate(v->quaternion,v->motion_body,v->motion_earth);
    v->yaw_deg=s->filter.Yaw; v->pitch_deg=s->filter.Pitch; v->roll_deg=s->filter.Roll;
    v->yaw_total_deg=s->filter.YawTotalAngle; v->temperature_c=raw->temperature_c;
    v->dt_s=dt; v->timestamp_us=raw->gyro_us; ++v->sequence;
    v->converged=s->filter.ConvergeFlag!=0U; v->valid=true;
}
bool RmIns_Read(const RmIns *s,uint64_t now_us,RmInsSnapshot *out)
{
    if (!s || !out) return false;
    *out=s->snapshot;
    out->valid=out->valid && now_us>=out->timestamp_us && now_us-out->timestamp_us<=s->config.freshness_us;
    return out->valid;
}
