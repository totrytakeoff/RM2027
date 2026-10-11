#include "chassis_control.h"
#include "chassis_config.h"
#include "binding.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void near(float actual,float expected) { assert(fabsf(actual-expected)<1e-4f); }
int main(void)
{
    RemoteControlState r={.type=REMOTE_CONTROL_TYPE_ET08,.link_online=1,.data_valid=1,
        .axis_valid_mask=15,.switch_valid_mask=15,
        .switches={REMOTE_SWITCH_DOWN,REMOTE_SWITCH_DOWN,REMOTE_SWITCH_DOWN,REMOTE_SWITCH_DOWN}};
    RmChassisIntent intent;
    assert(RmChassis_MapEt08(&r,&intent) && !intent.enable && intent.safe_position);
    r.switches[0]=REMOTE_SWITCH_UP; r.switch_changed_mask=1;
    r.axis_raw[0]=17; r.axis_normalized[0]=17.0f/660.0f;
    assert(RmChassis_MapEt08(&r,&intent) && intent.enable && intent.arm_event);
    near(intent.x,0); near(intent.y,0); near(intent.yaw,0);
    r.axis_normalized[0]=1; r.axis_raw[0]=660;
    assert(RmChassis_MapEt08(&r,&intent)); near(intent.x,-1); assert(!intent.safe_position);
    r.axis_normalized[1]=-1; r.axis_raw[1]=-660;
    assert(RmChassis_MapEt08(&r,&intent)); near(intent.y,-1);
    r.switches[1]=REMOTE_SWITCH_MIDDLE;
    assert(RmChassis_MapEt08(&r,&intent) && intent.mode==RM_CHASSIS_AUTO_AIM);
    r.switches[1]=REMOTE_SWITCH_UP;
    assert(RmChassis_MapEt08(&r,&intent) && intent.mode==RM_CHASSIS_SPIN);
    r.failsafe=1; assert(!RmChassis_MapEt08(&r,&intent) && !intent.enable);
    r.failsafe=0; r.axis_normalized[0]=NAN; assert(!RmChassis_MapEt08(&r,&intent));
    RmInsSnapshot imu={.valid=true};
    RmChassisController c; RmChassis_Reset(&c);
    RmChassisOutput out;
    intent=(RmChassisIntent){.valid=true,.enable=true,.mode=RM_CHASSIS_FOLLOW};
    assert(RmChassis_Step(&c,&intent,&imu,0.02f,&out));
    for (unsigned i=0;i<4;++i) near(out.wheel_ref_rad_s[i],0);
    imu.yaw_total_deg=10;
    assert(RmChassis_Step(&c,&intent,&imu,0.02f,&out) && out.wz_rad_s<0);
    imu.yaw_total_deg=0; imu.gyro[2]=1;
    RmChassis_Reset(&c);
    assert(RmChassis_Step(&c,&intent,&imu,0.02f,&out) && out.wz_rad_s<0);
    imu.gyro[2]=0; intent.yaw=0.5f; RmChassis_Reset(&c);
    assert(RmChassis_Step(&c,&intent,&imu,0.02f,&out) && out.yaw_error_rad>0 && out.wz_rad_s>0);
    intent.yaw=0; c.target_yaw_rad=179.0f*0.017453292519943295f; imu.yaw_total_deg=-179;
    assert(RmChassis_Step(&c,&intent,&imu,0.02f,&out)); near(out.yaw_error_rad,-2*0.017453292519943295f);
    RmChassis_Reset(&c); imu.yaw_total_deg=0; intent.mode=RM_CHASSIS_SPIN; intent.x=intent.y=1;
    for (unsigned n=0;n<20;++n) assert(RmChassis_Step(&c,&intent,&imu,0.02f,&out));
    assert(out.translation_scale>=0 && out.translation_scale<1);
    for (unsigned i=0;i<4;++i) assert(fabsf(out.wheel_ref_rad_s[i])<=CHASSIS_DEMO_MOTOR_MAX_RAD_S+0.001f);
    intent.mode=RM_CHASSIS_FOLLOW; intent.x=intent.y=0; imu.yaw_total_deg=45;
    assert(RmChassis_Step(&c,&intent,&imu,0.02f,&out)); near(out.wz_rad_s,-CHASSIS_DEMO_YAW_MAX_RAD_S);
    imu.gyro[2]=NAN;
    assert(!RmChassis_Step(&c,&intent,&imu,0.02f,&out) && !c.active);
    for (unsigned i=0;i<4;++i) near(out.wheel_ref_rad_s[i],0);
    imu.gyro[2]=0; assert(RmChassis_Step(&c,&intent,&imu,0.02f,&out));
    c.filtered_vx=NAN; assert(!RmChassis_Step(&c,&intent,&imu,0.02f,&out));
    assert(!RmChassis_Step(&c,&intent,&imu,0.1f,&out));
    RmDjiRegistry registry;
    assert(RmDji_Init(&registry,rm_bringup_binding.motors,4));
    RmDjiCommand commands[4]={0};
    for (unsigned i=0;i<4;++i) {
        RmCanFrame frame={.bus=CHASSIS_DEMO_CAN,.identifier=registry.motors[i].feedback_id,
            .generation=1,.timestamp_us=1000,.length=8};
        assert(RmDji_Receive(&registry,&frame));
        commands[i]=(RmDjiCommand){.enabled=true,.outer_loop=RM_DJI_LOOP_SPEED,
            .reference=10.0f,.epoch=7,.deadline_ms=100};
    }
    RmDjiGroup groups[RM_DJI_MAX_GROUPS];
    assert(RmDji_Step(&registry,commands,4,1000,0.005f,true,7,100,groups)==1);
    assert(groups[0].frame.identifier==0x200 && groups[0].frame.bus==RM_CAN_1);
    for (unsigned i=0;i<4;++i) {
        assert(registry.motors[i].active && registry.motors[i].output>800 && registry.motors[i].output<805);
        unsigned slot=registry.motors[i].command_slot*2;
        assert((((unsigned)groups[0].frame.data[slot]<<8)|groups[0].frame.data[slot+1])>800);
    }
    assert(RmDji_Step(&registry,commands,4,1000,0.005f,false,7,100,groups)==1);
    for (unsigned i=0;i<8;++i) assert(groups[0].frame.data[i]==0);
    assert(RmDji_Step(&registry,commands,4,22000,0.005f,true,7,100,groups)==1);
    for (unsigned i=0;i<8;++i) assert(groups[0].frame.data[i]==0);
    puts("PASS: ET08, yaw/wrap, spin, transitions, numerical stop, M3508 cascade/wire/lease");
    return 0;
}
