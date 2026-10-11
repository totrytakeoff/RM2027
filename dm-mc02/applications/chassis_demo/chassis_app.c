#include "chassis_app.h"
#include "chassis_config.h"
#include "bringup_app.h"
#include "bringup.h"
#include "output_gate.h"
#include "safety_manager.h"
#include "logging.h"
#include <math.h>
#include <string.h>
static RmRuntimeConfig runtime;
static void (*base_diagnostics)(uint64_t);
static RmChassisController controller;
static RmSafetyManager safety;
static RmChassisSnapshot published;
static uint64_t previous_tick,start_us;
static uint32_t publish_failures;
bool RmChassisApp_Read(RmChassisSnapshot *out)
{
    if (!out) return false;
    RmCriticalState key=RmCritical_Enter(); *out=published; RmCritical_Exit(key);
    return true;
}
static void control(uint64_t now_us)
{
    RmChassisSnapshot next={.timestamp_us=now_us};
    RmChassisIntent intent;
    RmInsSnapshot imu;
    bool remote_ok=RmBringup_TakeRemote(&next.remote) && RmChassis_MapEt08(&next.remote,&intent);
    if (!remote_ok) memset(&intent,0,sizeof(intent));
    bool imu_ok=RmBringup_ReadIns(&imu);
    for (unsigned i=0;i<4U;++i) if (RmBringup_ReadMotor(i,&next.feedback[i])) next.online_mask|=1U<<i;
    RmCanStats bus;
    bool bus_ok=RmCan_GetStats(CHASSIS_DEMO_CAN,&bus) && bus.online;
    uint32_t now_ms=RmTime_NowMs();
    const RmSafetyInputs input={
        .initialization_complete=imu_ok && now_us-start_us>=CHASSIS_DEMO_STARTUP_MS*1000ULL,
        .input_online=remote_ok,.emergency_stop=!remote_ok || RmBoard_KeyPressed(),
        .operator_enable_request=intent.enable,.operator_safe_position=intent.safe_position,
        .operator_arm_event=intent.arm_event,.task_health_ok=RmRuntime_Healthy(now_ms),
        .device_health_ok=bus_ok && (!CHASSIS_DEMO_REQUIRE_ALL_MOTORS || next.online_mask==15U)};
    (void)RmSafety_Update(&safety,&input);
    float dt=previous_tick?(float)(now_us-previous_tick)*1e-6f:0.02f;
    previous_tick=now_us;
    bool permit=RmSafety_OutputPermitted(&safety);
    if (permit) permit=RmChassis_Step(&controller,&intent,&imu,dt,&next.output);
    if (!permit) {
        RmChassis_Reset(&controller); RmOutputGate_Stop(safety.reasons?safety.reasons:1U<<13);
        if (imu_ok) {
            next.output.yaw_rad=imu.yaw_total_deg*0.017453292519943295f*CHASSIS_DEMO_IMU_YAW_SIGN;
            next.output.target_yaw_rad=next.output.yaw_rad;
        }
        next.output.mode=intent.mode;
    } else {
        /* Recheck live inputs immediately before issuing a lease. An I/O/health
           preemption during the calculation must not reopen a stopped gate. */
        RmCriticalState key=RmCritical_Enter();
        RemoteControlState live_remote; RmChassisIntent live_intent; RmInsSnapshot live_imu;
        now_ms=RmTime_NowMs();
        permit=RmBringup_ReadRemote(&live_remote) && RmChassis_MapEt08(&live_remote,&live_intent) &&
            live_intent.enable && live_intent.mode==intent.mode && RmBringup_ReadIns(&live_imu) &&
            RmRuntime_Healthy(now_ms) && !RmBoard_KeyPressed() &&
            RmCan_GetStats(CHASSIS_DEMO_CAN,&bus) && bus.online;
        if (permit) {
            RmOutputGateSnapshot gate; RmOutputGate_Read(&gate);
            uint32_t epoch=gate.permitted && RmOutputGate_Renew(gate.epoch,now_ms,RM_COMMAND_LEASE_MS)?
                gate.epoch:RmOutputGate_Arm(now_ms,RM_COMMAND_LEASE_MS);
            RmDjiCommand commands[4]={0};
            for (unsigned i=0;i<4U;++i) commands[i]=(RmDjiCommand){
                .reference=next.output.wheel_ref_rad_s[i],.epoch=epoch,
                .deadline_ms=now_ms+RM_COMMAND_LEASE_MS,.outer_loop=RM_DJI_LOOP_SPEED,.enabled=true};
            permit=RmBringup_SubmitMotorCommands(commands,4U,epoch);
            if (!permit) ++publish_failures;
        }
        if (!permit) RmOutputGate_Stop(1U<<13);
        RmCritical_Exit(key);
        if (!permit) { RmChassis_Reset(&controller); memset(next.output.wheel_ref_rad_s,0,sizeof(next.output.wheel_ref_rad_s)); }
    }
    next.enabled=permit; next.reasons=safety.reasons;
    if (!permit && !next.reasons) next.reasons=1U<<13;
    next.publish_failures=publish_failures;
    RmCriticalState key=RmCritical_Enter(); published=next; RmCritical_Exit(key);
}
typedef struct { char bytes[64]; size_t used; bool overflow; } Line;
static void text(Line *line,const char *value)
{
    while (*value) {
        if (line->used==sizeof(line->bytes)) { line->overflow=true; return; }
        line->bytes[line->used++]=*value++;
    }
}
static void number(Line *line,int32_t value)
{
    uint32_t magnitude=value<0?(uint32_t)(-(int64_t)value):(uint32_t)value;
    if (value<0) text(line,"-");
    char digits[11]={0}; unsigned n=0;
    do { digits[n++]=(char)('0'+magnitude%10U); magnitude/=10U; } while (magnitude);
    while (n) { char digit[2]={digits[--n],0}; text(line,digit); }
}
static void field(Line *line,const char *name,int32_t value) { text(line,name); number(line,value); }
static int32_t milli(float value)
{
    double scaled=(double)value*1000.0;
    if (!isfinite(scaled)) return 0;
    if (scaled>2147483647.0) return INT32_MAX;
    if (scaled<-2147483648.0) return INT32_MIN;
    return (int32_t)scaled;
}
static void diagnostics(uint64_t now_us)
{
    static uint32_t next_report;
    static unsigned row=8U;
    static RmChassisSnapshot report;
    base_diagnostics(now_us);
    uint32_t now=(uint32_t)(now_us/1000U);
    if (row==8U && (int32_t)(now-next_report)>=0) {
        (void)RmChassisApp_Read(&report); row=0U; next_report=now+200U;
    }
    if (row==8U) return;
    Line line={0};
    if (row==0U) {
        field(&line,"CH t=",(int32_t)(report.timestamp_us/1000U));
        field(&line," mode=",(int32_t)report.output.mode); field(&line," en=",report.enabled);
        field(&line," tx=",RM_ENABLE_MOTION); field(&line," why=",(int32_t)report.reasons);
        field(&line," online=",(int32_t)report.online_mask);
    } else if (row==1U) {
        field(&line,"RC lx=",report.remote.axis_raw[0]); field(&line," ly=",report.remote.axis_raw[1]);
        field(&line," rx=",report.remote.axis_raw[2]); field(&line," ry=",report.remote.axis_raw[3]);
        field(&line," sa=",report.remote.switches[0]); field(&line," sb=",report.remote.switches[1]);
    } else if (row==2U) {
        field(&line,"YAW ref=",milli(report.output.target_yaw_rad));
        field(&line," imu=",milli(report.output.yaw_rad)); field(&line," err=",milli(report.output.yaw_error_rad));
        field(&line," rate=",milli(report.output.yaw_rate_rad_s));
    } else if (row==3U) {
        field(&line,"BODY vx=",milli(report.output.vx_m_s)); field(&line," vy=",milli(report.output.vy_m_s));
        field(&line," wz=",milli(report.output.wz_rad_s)); field(&line," scale=",milli(report.output.translation_scale));
    } else {
        unsigned i=row-4U;
        const RmDjiFeedback *f=&report.feedback[i];
        field(&line,"M",(int32_t)i); field(&line," ref=",milli(report.output.wheel_ref_rad_s[i]));
        field(&line," fdb=",milli(f->speed_rad_s)); field(&line," cur=",f->current_raw);
        field(&line," temp=",f->temperature);
    }
    text(&line,"\r\n");
    if (line.overflow || RmSerial_TrySendCopy(RM_SERIAL_VISION,line.bytes,line.used)==RM_OK) ++row;
}
const RmRuntimeConfig *RmChassisApp_Init(void)
{
    if (!RmChassis_ConfigValid()) RmFault_Panic(0x5002U);
    if (!RmLog_SetOutput(RM_SERIAL_VISION)) RmFault_Panic(0x5001U);
    const RmRuntimeConfig *base=RmBringup_Init(); runtime=*base;
    base_diagnostics=base->tick[RM_TASK_DIAGNOSTICS];
    runtime.tick[RM_TASK_CONTROL]=control; runtime.tick[RM_TASK_DIAGNOSTICS]=diagnostics;
    const RmSafetyPolicy policy={.require_explicit_rearm=CHASSIS_DEMO_REQUIRE_REARM};
    RmSafety_InitWithPolicy(&safety,&policy); RmChassis_Reset(&controller);
    start_us=RmTime_NowUs();
    return &runtime;
}
