#include "mecanum_app.h"
#include "mecanum_config.h"
#include "binding.h"
#include "bringup_app.h"
#include "bringup.h"
#include "output_gate.h"
#include "safety_manager.h"
#include "logging.h"
#include <math.h>
#include <string.h>
static RmRuntimeConfig runtime;
static void (*base_diagnostics)(uint64_t);
static RmMecanumController controller;
static RmSafetyManager safety;
static RmMecanumSnapshot published;
static uint64_t previous_tick,start_us;
static uint32_t publish_failures;
bool RmMecanumApp_Read(RmMecanumSnapshot *out)
{
    if (!out) return false;
    RmCriticalState key=RmCritical_Enter(); *out=published; RmCritical_Exit(key);
    return true;
}
static void control(uint64_t now_us)
{
    RmMecanumSnapshot next={.timestamp_us=now_us};
    RmMecanumIntent intent;
    RmInsSnapshot imu;
    bool remote_ok=RmBringup_TakeRemote(&next.remote) && RmMecanum_MapEt08(&next.remote,&intent);
    if (!remote_ok) memset(&intent,0,sizeof(intent));
    bool imu_ok=RmBringup_ReadIns(&imu);
    for (unsigned i=0;i<4U;++i) if (RmBringup_ReadMotor(i,&next.feedback[i])) next.online_mask|=1U<<i;
    RmCanStats bus;
    bool bus_ok=RmCan_GetStats(MECANUM_DEMO_CAN,&bus) && bus.online;
    uint32_t now_ms=RmTime_NowMs();
    const RmSafetyInputs input={
        .initialization_complete=imu_ok && now_us-start_us>=MECANUM_DEMO_STARTUP_MS*1000ULL,
        .input_online=remote_ok,.emergency_stop=!remote_ok || RmBoard_KeyPressed(),
        .operator_enable_request=intent.enable,.operator_safe_position=intent.safe_position,
        .operator_arm_event=intent.arm_event,.task_health_ok=RmRuntime_Healthy(now_ms),
        .device_health_ok=bus_ok && (!MECANUM_DEMO_REQUIRE_ALL_MOTORS || next.online_mask==15U)};
    (void)RmSafety_Update(&safety,&input);
    float dt=previous_tick?(float)(now_us-previous_tick)*1e-6f:0.02f;
    previous_tick=now_us;
    bool permit=RmSafety_OutputPermitted(&safety);
    if (permit) permit=RmMecanum_Step(&controller,&intent,&imu,dt,&next.output);
    if (!permit) {
        RmMecanum_Reset(&controller); RmOutputGate_Stop(safety.reasons?safety.reasons:1U<<13);
        if (imu_ok) {
            next.output.yaw_rad=imu.yaw_total_deg*0.017453292519943295f*MECANUM_DEMO_IMU_YAW_SIGN;
            next.output.target_yaw_rad=next.output.yaw_rad;
        }
        next.output.mode=intent.mode;
    } else {
        /* Recheck live inputs immediately before issuing a lease. An I/O/health
           preemption during the calculation must not reopen a stopped gate. */
        RmCriticalState key=RmCritical_Enter();
        RemoteControlState live_remote; RmMecanumIntent live_intent; RmInsSnapshot live_imu;
        now_ms=RmTime_NowMs();
        permit=RmBringup_ReadRemote(&live_remote) && RmMecanum_MapEt08(&live_remote,&live_intent) &&
            live_intent.enable && live_intent.mode==intent.mode && RmBringup_ReadIns(&live_imu) &&
            RmRuntime_Healthy(now_ms) && !RmBoard_KeyPressed() &&
            RmCan_GetStats(MECANUM_DEMO_CAN,&bus) && bus.online;
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
        if (!permit) { RmMecanum_Reset(&controller); memset(next.output.wheel_ref_rad_s,0,sizeof(next.output.wheel_ref_rad_s)); }
    }
    next.enabled=permit; next.reasons=safety.reasons;
    if (!permit && !next.reasons) next.reasons=1U<<13;
    next.publish_failures=publish_failures;
    RmCriticalState key=RmCritical_Enter(); published=next; RmCritical_Exit(key);
}
enum { VOFA_CHANNELS=7U, VOFA_FRAME_BYTES=VOFA_CHANNELS*4U+4U };
_Static_assert(sizeof(float)==4U,"JustFloat requires 32-bit floats");
_Static_assert(VOFA_FRAME_BYTES<=64U,"USART1 copied TX slot is 64 bytes");
static void diagnostics(uint64_t now_us)
{
    static uint32_t window_start,window_mask,samples;
    static float current_sum[4],absolute_sum;
    static bool window_active;
    /* Framework text stays on USART10; USART1 carries only JustFloat. */
    base_diagnostics(now_us);
    uint32_t now=(uint32_t)(now_us/1000U);
    float current_by_id[4]={0},absolute_current=0.0f;
    uint32_t valid_mask=0U;
    for (unsigned i=0;i<4U;++i) {
        RmDjiFeedback feedback;
        unsigned id=rm_bringup_binding.motors[i].id;
        if (id<1U || id>4U || !RmBringup_ReadMotor(i,&feedback)) continue;
        float current=(float)feedback.current_raw/MECANUM_DEMO_CURRENT_RAW_PER_AMP;
        current_by_id[id-1U]=current;
        /* Rectify each sample before averaging; forward/reverse must not
           cancel in the power estimate or absolute current total. */
        absolute_current+=fabsf(current);
        valid_mask|=1U<<(id-1U);
    }
    if (!window_active || valid_mask!=window_mask) {
        /* Do not blend old online data into an offline/recovered window. */
        memset(current_sum,0,sizeof(current_sum)); absolute_sum=0.0f; samples=0U;
        window_start=now; window_mask=valid_mask; window_active=true;
    }
    for (unsigned i=0;i<4U;++i) current_sum[i]+=current_by_id[i];
    absolute_sum+=absolute_current; ++samples;
    if ((uint32_t)(now-window_start)<MECANUM_DEMO_VOFA_PERIOD_MS) return;
    float channel[VOFA_CHANNELS]={0};
    float inverse_samples=1.0f/(float)samples;
    channel[1]=absolute_sum*inverse_samples;
    for (unsigned i=0;i<4U;++i) channel[i+2U]=current_sum[i]*inverse_samples;
    channel[0]=channel[1]*MECANUM_DEMO_SUPPLY_VOLTAGE; /* Estimate, watts. */
    channel[6]=(float)valid_mask;
    memset(current_sum,0,sizeof(current_sum)); absolute_sum=0.0f; samples=0U;
    window_start=now;
    uint8_t frame[VOFA_FRAME_BYTES];
    for (unsigned i=0;i<VOFA_CHANNELS;++i) {
        uint32_t bits;
        if (!isfinite(channel[i])) channel[i]=0.0f;
        memcpy(&bits,&channel[i],sizeof(bits));
        frame[i*4U]=(uint8_t)bits;
        frame[i*4U+1U]=(uint8_t)(bits>>8);
        frame[i*4U+2U]=(uint8_t)(bits>>16);
        frame[i*4U+3U]=(uint8_t)(bits>>24);
    }
    const uint8_t tail[4]={0x00U,0x00U,0x80U,0x7fU};
    memcpy(frame+VOFA_CHANNELS*4U,tail,sizeof(tail));
    /* The platform copies the whole frame before returning; no stack DMA.
       Backpressure drops this sample; it never blocks the control tasks. */
    (void)RmSerial_TrySendCopy(RM_SERIAL_VISION,frame,sizeof(frame));
}
const RmRuntimeConfig *RmMecanumApp_Init(void)
{
    if (!RmMecanum_ConfigValid()) RmFault_Panic(0x5002U);
    if (!RmLog_SetOutput(RM_SERIAL_CONSOLE)) RmFault_Panic(0x5001U);
    const RmRuntimeConfig *base=RmBringup_Init(); runtime=*base;
    base_diagnostics=base->tick[RM_TASK_DIAGNOSTICS];
    runtime.tick[RM_TASK_CONTROL]=control; runtime.tick[RM_TASK_DIAGNOSTICS]=diagnostics;
    const RmSafetyPolicy policy={.require_explicit_rearm=MECANUM_DEMO_REQUIRE_REARM};
    RmSafety_InitWithPolicy(&safety,&policy); RmMecanum_Reset(&controller);
    start_us=RmTime_NowUs();
    return &runtime;
}
