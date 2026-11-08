#include "motor_test_device.h"
#if MOTOR_TEST_MODEL == MOTOR_TEST_MODEL_DM4310
#include "dm.h"
#include "output_gate.h"
#include "bringup.h"
#include <math.h>
#include <string.h>
_Static_assert(MOTOR_TEST_ID>0 && MOTOR_TEST_ID<16,"DM feedback ID occupies four bits; use ID 1..15");
_Static_assert(MOTOR_TEST_DM_MASTER_ID<=0x7ff,"DM Master ID must be a standard CAN ID");
/* I/O publishes config and feedback; MOTOR owns PI and link activation. */
static RmDmLimits limits;
static RmDmFeedback feedback;
static uint32_t mask, generation, control_mode, timeout;
static bool seen, continuous;
static float absolute_position, last_position, zero_position;
static RmDjiCommand command;
static PIDInstance speed_pid, position_pid;
static float submitted_torque;
static const uint8_t registers[]={0x15,0x16,0x17,0x0a,0x09};
static void reset_link(uint32_t gen)
{
    mask=0; generation=gen; seen=continuous=false;
    limits=(RmDmLimits){0}; control_mode=timeout=0;
}
static bool ready(void)
{
    return mask==31U && RmDm_LimitsValid(&limits) && control_mode==1U && timeout>0U && timeout<=2000U;
}
uint32_t RmMotorTest_DmConfigMask(void)
{
    RmCriticalState key=RmCritical_Enter(); uint32_t value=mask; RmCritical_Exit(key); return value;
}
uint32_t RmMotorTest_DmFaults(void)
{
    RmCriticalState key=RmCritical_Enter();
    uint32_t faults=(mask!=31U?1U:0U) | (control_mode!=1U?2U:0U) |
        (!timeout || timeout>2000U?4U:0U) | (!RmDm_LimitsValid(&limits)?8U:0U) |
        (seen && feedback.state>1?16U:0U);
    RmCritical_Exit(key); return faults;
}
float RmMotorTest_DmTorque(void)
{
    RmCriticalState key=RmCritical_Enter();float value=submitted_torque;RmCritical_Exit(key);return value;
}
static void receive(const RmCanFrame *frame)
{
    if (frame->bus!=MOTOR_TEST_CAN) return;
    RmCriticalState key=RmCritical_Enter();
    if (frame->generation!=generation) reset_link(frame->generation);
    uint8_t reg; uint32_t bits;
    if (RmDm_RegisterReply(frame,MOTOR_TEST_DM_MASTER_ID,MOTOR_TEST_ID,&reg,&bits)) {
        for (unsigned i=0;i<5;++i) if (reg==registers[i]) {
            if (i<3) {
                float value; memcpy(&value,&bits,sizeof(value));
                if (!isfinite(value) || value<=0) { mask &= ~(1U<<i); break; }
                if (i==0) limits.position=value;
                if (i==1) limits.velocity=value;
                if (i==2) limits.torque=value;
            } else if (i==3) control_mode=bits;
            else timeout=bits;
            mask|=1U<<i; break;
        }
    } else if ((mask&7U)==7U && RmDm_LimitsValid(&limits) &&
               !(frame->length==8 && frame->data[0]==MOTOR_TEST_ID && frame->data[1]==0 && frame->data[2]==0x55)) {
        RmDmFeedback next;
        if (RmDm_Decode(frame,MOTOR_TEST_DM_MASTER_ID,MOTOR_TEST_ID,&limits,&next) &&
            (!seen || next.timestamp_us>=feedback.timestamp_us)) {
            if (!seen) { absolute_position=next.position_rad; continuous=true; }
            else {
                float delta=remainderf(next.position_rad-last_position,2.0f*limits.position);
                if (next.timestamp_us-feedback.timestamp_us>=20000U ||
                    fabsf(next.velocity_rad_s)*(float)(next.timestamp_us-feedback.timestamp_us)*1e-6f>=limits.position)
                    continuous=false;
                absolute_position+=delta;
            }
            last_position=next.position_rad; feedback=next; seen=true;
        }
    }
    RmCritical_Exit(key);
}
bool RmMotorTest_DeviceRead(RmDjiFeedback *out)
{
    if (!out) return false;
    RmCriticalState key=RmCritical_Enter();
    RmCanStats bus; uint64_t now=RmTime_NowUs();
    bool valid=ready() && seen && RmCan_GetStats(MOTOR_TEST_CAN,&bus) && bus.online &&
        bus.generation==generation && now>=feedback.timestamp_us && now-feedback.timestamp_us<20000U && feedback.state<=1;
    *out=(RmDjiFeedback){.timestamp_us=feedback.timestamp_us,.generation=generation,
        .rpm=(int16_t)fmaxf(-32767,fminf(32767,feedback.velocity_rad_s*9.5492965855f)),
        .current_raw=(int16_t)fmaxf(-32767,fminf(32767,feedback.torque_nm*1000.0f)),
        .speed_rad_s=feedback.velocity_rad_s,.angle_rad=absolute_position-zero_position,
        .angle_deg=(absolute_position-zero_position)*57.2957795131f,
        .temperature=feedback.rotor_temperature,.valid=valid,.position_valid=valid && continuous};
    RmCritical_Exit(key); return valid;
}
bool RmMotorTest_DeviceZero(void)
{
    RmCriticalState key=RmCritical_Enter();
    RmOutputGateSnapshot gate; RmOutputGate_Read(&gate); RmDjiFeedback current;
    bool valid=!gate.permitted && RmMotorTest_DeviceRead(&current);
    if (valid) { zero_position=absolute_position; continuous=true; }
    RmCritical_Exit(key); return valid;
}
bool RmMotorTest_DeviceSubmit(const RmDjiCommand *cmd)
{
    if (!cmd) return false;
    RmCriticalState key=RmCritical_Enter(); uint32_t now=RmTime_NowMs();
    bool valid=cmd->enabled && isfinite(cmd->reference) &&
        (cmd->outer_loop==RM_DJI_LOOP_SPEED || cmd->outer_loop==RM_DJI_LOOP_ANGLE) &&
        cmd->deadline_ms-now<=RM_COMMAND_LEASE_MS && (int32_t)(cmd->deadline_ms-now)>0 &&
        RmOutputGate_Allows(cmd->epoch,now);
    if (valid) command=*cmd;
    RmCritical_Exit(key); return valid;
}
static void motor_tick(uint64_t now_us)
{
    static uint64_t last;
    static uint32_t next_service, next_enable, next_timeout_write;
    static unsigned query;
    static bool prepared, active;
    float dt=last?(float)(now_us-last)*1e-6f:0.005f; last=now_us;
    uint32_t now=(uint32_t)(now_us/1000U);
    RmDjiFeedback current; bool online=RmMotorTest_DeviceRead(&current);
    RmCriticalState key=RmCritical_Enter();
    RmCanStats bus;
    if (RmCan_GetStats(MOTOR_TEST_CAN,&bus) && bus.generation!=generation) reset_link(bus.generation);
    RmDjiCommand cmd=command; RmDmLimits range=limits;
    bool configured=ready(); uint8_t state=feedback.state;
    bool repair_timeout=mask==31U && RmDm_LimitsValid(&limits) && control_mode==1U &&
        (!timeout || timeout>2000U) && seen && state==0 && now_us>=feedback.timestamp_us &&
        now_us-feedback.timestamp_us<20000U;
    RmCritical_Exit(key);
    bool permit=RM_ENABLE_MOTION && configured && online && RmOutputGate_Allows(cmd.epoch,now) &&
        (int32_t)(cmd.deadline_ms-now)>0 && cmd.enabled && dt>0 && dt<=0.02f &&
        (cmd.outer_loop!=RM_DJI_LOOP_ANGLE || current.position_valid);
    RmCanFrame frame;
    if (!permit) {
        PIDReset(&speed_pid); PIDReset(&position_pid); active=prepared=false;
        key=RmCritical_Enter(); submitted_torque=0; RmCritical_Exit(key);
        if ((int32_t)(now-next_service)>=0) {
            next_service=now+10U;
            RmDm_SpecialFrame(&frame,MOTOR_TEST_CAN,MOTOR_TEST_ID,false);
            (void)RmCan_SubmitDmServiceFrame(&frame,now+10U);
            bool wrote_timeout=false;
            if (repair_timeout && (int32_t)(now-next_timeout_write)>=0) {
                key=RmCritical_Enter();
                RmOutputGateSnapshot gate; RmOutputGate_Read(&gate);
                if (!gate.permitted) {
                    RmDm_TimeoutFrame(&frame,MOTOR_TEST_CAN,MOTOR_TEST_ID);
                    wrote_timeout=RmCan_SubmitDmTimeoutFrame(&frame,now+10U)==RM_OK;
                    next_timeout_write=now+100U;
                }
                RmCritical_Exit(key);
            }
            if (!configured && !wrote_timeout) {
                RmDm_ReadRegister(&frame,MOTOR_TEST_CAN,MOTOR_TEST_ID,registers[query++%5]);
                (void)RmCan_SubmitDmServiceFrame(&frame,now+10U);
            }
        }
        return;
    }
    uint32_t deadline=cmd.deadline_ms;
    if ((int32_t)(deadline-now)>10) deadline=now+10;
    if (state==0) {
        PIDReset(&speed_pid); PIDReset(&position_pid); active=false;
        key=RmCritical_Enter(); submitted_torque=0; RmCritical_Exit(key);
        if (!prepared) {
            if (RmDm_TorqueFrame(&frame,MOTOR_TEST_CAN,MOTOR_TEST_ID,&range,0) &&
                RmCan_Submit(&frame,cmd.epoch,deadline)==RM_OK) { prepared=true; next_enable=now+5; }
        } else if ((int32_t)(now-next_enable)>=0) {
            RmDm_SpecialFrame(&frame,MOTOR_TEST_CAN,MOTOR_TEST_ID,true);
            (void)RmCan_Submit(&frame,cmd.epoch,deadline); next_enable=now+20;
        }
        return;
    }
    if (!active || cmd.reset_mask) { PIDReset(&speed_pid); PIDReset(&position_pid); }
    active=true; prepared=true;
    float reference=cmd.reference;
    float speed_limit=fminf(MOTOR_TEST_MAX_RAD_S,range.velocity);
    position_pid.MaxOut=speed_limit;
    if (cmd.outer_loop==RM_DJI_LOOP_ANGLE)
        reference=PIDCalculate(&position_pid,current.angle_rad,reference*0.0174532925199f,dt);
    reference=fmaxf(-speed_limit,fminf(speed_limit,reference));
    speed_pid.MaxOut=fminf(MOTOR_TEST_DM_TORQUE_LIMIT,range.torque);
    speed_pid.IntegralLimit=fminf(MOTOR_TEST_INTEGRAL_LIMIT,speed_pid.MaxOut);
    float torque=PIDCalculate(&speed_pid,current.speed_rad_s,reference,dt);
    if (speed_pid.dt<=0 || (cmd.outer_loop==RM_DJI_LOOP_ANGLE && position_pid.dt<=0)) {
        RmOutputGate_Stop(1U<<13); return;
    }
    if (!RmDm_TorqueFrame(&frame,MOTOR_TEST_CAN,MOTOR_TEST_ID,&range,torque) ||
        RmCan_Submit(&frame,cmd.epoch,deadline)!=RM_OK) {
        RmOutputGate_Stop(1U<<13);
        key=RmCritical_Enter(); submitted_torque=0; RmCritical_Exit(key);
    } else {
        key=RmCritical_Enter(); submitted_torque=torque; RmCritical_Exit(key);
    }
}
void RmMotorTest_DeviceInit(RmRuntimeConfig *runtime)
{
    const PID_Init_Config_s pi={.Kp=MOTOR_TEST_SPEED_KP,.Ki=MOTOR_TEST_SPEED_KI,
        .MaxOut=MOTOR_TEST_DM_TORQUE_LIMIT,.IntegralLimit=MOTOR_TEST_INTEGRAL_LIMIT,
        .Improve=PID_Trapezoid_Intergral|PID_Integral_Limit|PID_HoldIntegral};
    PIDInit(&speed_pid,&pi);
    const PID_Init_Config_s position={.Kp=MOTOR_TEST_POSITION_KP*57.2957795131f,
        .MaxOut=MOTOR_TEST_MAX_RAD_S};
    PIDInit(&position_pid,&position);
    RmBringup_SetCanObserver(receive); runtime->tick[RM_TASK_MOTOR]=motor_tick;
}
#endif
