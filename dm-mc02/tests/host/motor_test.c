#include "motor_test_control.h"
#include "motor_test_config.h"
#include "binding.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void near(float actual, float expected) { assert(fabsf(actual-expected) < 0.001f); }
int main(void)
{
    RemoteControlState r = {.data_valid=1,.axis_valid_mask=15,.switch_valid_mask=15};
    r.switches[3]=REMOTE_SWITCH_DOWN;
    assert(RmMotorTest_Mode(&r)==MOTOR_TEST_SPEED);
    r.axis_normalized[REMOTE_AXIS_LEFT_Y]=1;
    r.axis_normalized[REMOTE_AXIS_RIGHT_Y]=-1;
    near(RmMotorTest_Reference(&r,MOTOR_TEST_SPEED),MOTOR_TEST_MAX_RAD_S);
    r.switches[3]=REMOTE_SWITCH_UP;
    assert(RmMotorTest_Mode(&r)==MOTOR_TEST_POSITION);
    float end=MOTOR_TEST_POSITION_RANGE_RAD*57.2957795131f*MOTOR_TEST_REDUCTION;
    near(RmMotorTest_Reference(&r,MOTOR_TEST_POSITION),-end);
    r.axis_normalized[REMOTE_AXIS_RIGHT_Y]=1;
    near(RmMotorTest_Reference(&r,MOTOR_TEST_POSITION),end);
    r.axis_normalized[REMOTE_AXIS_RIGHT_Y]=0.1f;
    near(RmMotorTest_Reference(&r,MOTOR_TEST_POSITION),0);
    r.switches[3]=REMOTE_SWITCH_MIDDLE;
    assert(RmMotorTest_Mode(&r)==MOTOR_TEST_OFF);
    r.switches[3]=REMOTE_SWITCH_UP; r.data_valid=0;
    assert(RmMotorTest_Mode(&r)==MOTOR_TEST_OFF);
    RmDjiRegistry reg;
    assert(RmDji_Init(&reg,rm_bringup_binding.motors,1));
    RmCanFrame frame={.bus=MOTOR_TEST_CAN,.identifier=0x201,.length=8,.timestamp_us=1000,.generation=1};
    assert(RmDji_Receive(&reg,&frame));
    RmDjiGroup groups[RM_DJI_MAX_GROUPS];
    RmDjiCommand cmd={.reference=end,.epoch=7,.deadline_ms=100,.outer_loop=RM_DJI_LOOP_ANGLE,.enabled=true};
    assert(RmDji_Step(&reg,&cmd,1,1000,0.005f,true,7,100,groups)==1);
    assert(reg.motors[0].output>0);
    near(reg.motors[0].speed_pid.Ref,end*MOTOR_TEST_POSITION_KP);
    cmd.reference=-end; cmd.reset_mask=7;
    (void)RmDji_Step(&reg,&cmd,1,1000,0.005f,true,7,100,groups);
    assert(reg.motors[0].output<0);
    cmd.reference=0;
    (void)RmDji_Step(&reg,&cmd,1,1000,0.005f,true,7,100,groups);
    near(reg.motors[0].output,0);
    reg.motors[0].feedback.value.position_valid=false;
    cmd.reference=end;
    (void)RmDji_Step(&reg,&cmd,1,1000,0.005f,true,7,100,groups);
    near(reg.motors[0].output,0);
    assert(!reg.motors[0].active);
    puts("PASS: SD modes, independent sticks, output-shaft +/-pi mapping, angle cascade and invalid-position stop");
}
