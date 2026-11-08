#include "motor_test_device.h"
#include "dm.h"
#include "output_gate.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint64_t clock_us=100000;
static RmCanStats bus={.online=true,.generation=1};
static void (*observer)(const RmCanFrame *);
static RmCanFrame last_motion;
static unsigned passive, motion, timeout_writes;
RmCriticalState RmCritical_Enter(void) { return 0; }
void RmCritical_Exit(RmCriticalState key) { (void)key; }
uint64_t RmTime_NowUs(void) { return clock_us; }
uint32_t RmTime_NowMs(void) { return (uint32_t)(clock_us/1000); }
void RmBringup_SetCanObserver(void (*fn)(const RmCanFrame *)) { observer=fn; }
bool RmCan_GetStats(RmCanId id,RmCanStats *out) { assert(id==MOTOR_TEST_CAN);*out=bus;return true; }
RmStatus RmCan_Submit(const RmCanFrame *frame,uint32_t epoch,uint32_t deadline)
{
    assert(RmOutputGate_Allows(epoch,RmTime_NowMs())); assert((int32_t)(deadline-RmTime_NowMs())>0);
    last_motion=*frame;++motion;return RM_OK;
}
RmStatus RmCan_SubmitDmServiceFrame(const RmCanFrame *frame,uint32_t deadline)
{
    assert(RmDm_ServiceFrameSafe(frame));assert((int32_t)(deadline-RmTime_NowMs())>0);
    ++passive;return RM_OK;
}
RmStatus RmCan_SubmitDmTimeoutFrame(const RmCanFrame *frame,uint32_t deadline)
{
    RmOutputGateSnapshot gate;RmOutputGate_Read(&gate);assert(!gate.permitted);
    assert(RmDm_TimeoutFrameSafe(frame));assert((int32_t)(deadline-RmTime_NowMs())>0);
    ++timeout_writes;return RM_OK;
}
static void params(void)
{
    const uint8_t regs[]={0x15,0x16,0x17,0x0a,0x09};
    const float values[]={12.5f,30,10};
    for(unsigned i=0;i<5;++i) {
        uint32_t bits=i==3?1:1000;
        if(i<3) memcpy(&bits,&values[i],4);
        RmCanFrame f={.bus=MOTOR_TEST_CAN,.identifier=MOTOR_TEST_DM_MASTER_ID,.length=8,
            .generation=bus.generation,.timestamp_us=clock_us,
            .data={MOTOR_TEST_ID,0,0x33,regs[i],(uint8_t)bits,(uint8_t)(bits>>8),(uint8_t)(bits>>16),(uint8_t)(bits>>24)}};
        observer(&f);
    }
}
static void status(unsigned state)
{
    RmCanFrame f={.bus=MOTOR_TEST_CAN,.identifier=MOTOR_TEST_DM_MASTER_ID,.length=8,
        .generation=bus.generation,.timestamp_us=clock_us,
        .data={(uint8_t)((state<<4)|MOTOR_TEST_ID),0x80,0,0x80,0x08,0,25,26}};
    observer(&f);
}
int main(void)
{
    RmRuntimeConfig runtime={0}; RmOutputGate_Init(); RmMotorTest_DeviceInit(&runtime);
    runtime.tick[RM_TASK_MOTOR](clock_us); assert(passive==2 && !motion);
    RmDjiFeedback f; assert(!RmMotorTest_DeviceRead(&f));
    params(); status(0);assert(RmMotorTest_DeviceRead(&f));
    assert(RmMotorTest_DmConfigMask()==31 && RmMotorTest_DeviceZero());
    RmCanFrame wrong_mode={.bus=MOTOR_TEST_CAN,.identifier=MOTOR_TEST_DM_MASTER_ID,.length=8,
        .generation=bus.generation,.timestamp_us=clock_us,.data={MOTOR_TEST_ID,0,0x33,0x0a,2,0,0,0}};
    observer(&wrong_mode);assert(!RmMotorTest_DeviceRead(&f) && (RmMotorTest_DmFaults()&2U));
    runtime.tick[RM_TASK_MOTOR](clock_us);assert(motion==0);
    params();status(0);assert(RmMotorTest_DeviceRead(&f));
    RmCanFrame no_timeout={.bus=MOTOR_TEST_CAN,.identifier=MOTOR_TEST_DM_MASTER_ID,.length=8,
        .generation=bus.generation,.timestamp_us=clock_us,.data={MOTOR_TEST_ID,0,0x33,0x09,0,0,0,0}};
    observer(&no_timeout);assert(!RmMotorTest_DeviceRead(&f));
    clock_us+=10000;status(0);runtime.tick[RM_TASK_MOTOR](clock_us);
    assert(timeout_writes==1 && motion==0 && !RmMotorTest_DeviceRead(&f));
    /* The write ACK must not be decoded as a motor position sample. */
    RmCanFrame ack;RmDm_TimeoutFrame(&ack,MOTOR_TEST_CAN,MOTOR_TEST_ID);
    ack.identifier=MOTOR_TEST_DM_MASTER_ID;ack.timestamp_us=clock_us;ack.generation=bus.generation;
    observer(&ack);assert(!RmMotorTest_DeviceRead(&f));
    params();status(0);assert(RmMotorTest_DeviceRead(&f));
    uint32_t epoch=RmOutputGate_Arm(RmTime_NowMs(),100);
    assert(!RmMotorTest_DeviceZero());
    RmDjiCommand cmd={.reference=5,.enabled=true,.outer_loop=RM_DJI_LOOP_SPEED,.epoch=epoch,.deadline_ms=RmTime_NowMs()+100};
    assert(RmMotorTest_DeviceSubmit(&cmd));
    clock_us+=5000;runtime.tick[RM_TASK_MOTOR](clock_us);
    assert(motion==1 && last_motion.data[7]!=0xfc); /* zero torque before enable */
    clock_us+=5000;status(0);runtime.tick[RM_TASK_MOTOR](clock_us);
    assert(motion==2 && last_motion.data[7]==0xfc);
    clock_us+=5000;status(1);runtime.tick[RM_TASK_MOTOR](clock_us);
    assert(motion==3 && (((last_motion.data[3]&15U)<<8)|last_motion.data[4])==0);
    assert(((last_motion.data[5]<<4)|(last_motion.data[6]>>4))==0);
    assert((((last_motion.data[6]&15U)<<8)|last_motion.data[7])>2048); /* positive torque */
    cmd.outer_loop=RM_DJI_LOOP_ANGLE;cmd.reference=-180;cmd.reset_mask=7;
    assert(RmMotorTest_DeviceSubmit(&cmd));
    clock_us+=5000;status(1);runtime.tick[RM_TASK_MOTOR](clock_us);
    assert((((last_motion.data[6]&15U)<<8)|last_motion.data[7])<2048);
    unsigned previous=motion;
    RmOutputGate_Stop(1);clock_us+=20000;runtime.tick[RM_TASK_MOTOR](clock_us);
    assert(motion==previous && !RmMotorTest_DeviceSubmit(&cmd));
    /* Reconnect invalidates parameters and cannot keep issuing torque. */
    ++bus.generation;clock_us+=20000;runtime.tick[RM_TASK_MOTOR](clock_us);
    assert(RmMotorTest_DmConfigMask()==0 && !RmMotorTest_DeviceRead(&f));
    params();status(0);assert(RmMotorTest_DeviceRead(&f));
    clock_us+=20001;assert(!RmMotorTest_DeviceRead(&f));
    puts("PASS: DM startup disable, parameters, enable handshake, torque cascade, lease stop, generation and TTL");
}
