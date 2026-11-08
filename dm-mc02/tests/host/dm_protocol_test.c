#include "dm.h"
#include "controller.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void near(float a,float b,float tol) { assert(fabsf(a-b)<tol); }
int main(void)
{
    RmDmLimits l={12.5f,30,10}; RmCanFrame f;
    for (int n=-10;n<=10;++n) {
        assert(RmDm_TorqueFrame(&f,RM_CAN_1,1,&l,(float)n));
        assert((((f.data[3]&15U)<<8)|f.data[4])==0); /* Kp */
        assert(((f.data[5]<<4)|(f.data[6]>>4))==0); /* Kd */
        unsigned p=(f.data[0]<<8)|f.data[1],v=(f.data[2]<<4)|(f.data[3]>>4);
        unsigned t=((f.data[6]&15U)<<8)|f.data[7];
        assert(p==32768 && v==2048);
        near((float)t*20/4095-10,(float)n,.003f);
        assert(!RmDm_ServiceFrameSafe(&f));
    }
    assert(!RmDm_TorqueFrame(&f,RM_CAN_1,1,&l,NAN));
    assert(!RmDm_TorqueFrame(&f,RM_CAN_1,0x7ff,&l,0));
    RmDm_SpecialFrame(&f,RM_CAN_1,1,true); assert(!RmDm_ServiceFrameSafe(&f));
    RmDm_SpecialFrame(&f,RM_CAN_1,1,false); assert(RmDm_ServiceFrameSafe(&f));
    f.data[7]=0xfe; assert(!RmDm_ServiceFrameSafe(&f));
    f.data[7]=0xfb; assert(!RmDm_ServiceFrameSafe(&f));
    RmDm_ReadRegister(&f,RM_CAN_1,1,0x15); assert(RmDm_ServiceFrameSafe(&f));
    f.data[2]=0x55; assert(!RmDm_ServiceFrameSafe(&f));
    f.data[2]=0x33; f.data[3]=0x50; assert(!RmDm_ServiceFrameSafe(&f));
    RmDm_ReadRegister(&f,RM_CAN_1,0x7ff,0x15); assert(!RmDm_ServiceFrameSafe(&f));
    f=(RmCanFrame){.identifier=0,.length=8,.data={0x11,0xff,0xff,0xff,0xff,0xff,30,31}};
    RmDmFeedback out;
    assert(RmDm_Decode(&f,0,1,&l,&out));
    near(out.position_rad,12.5f,.0001f); near(out.velocity_rad_s,30,.0001f); near(out.torque_nm,10,.0001f);
    assert(out.state==1 && out.mos_temperature==30 && out.rotor_temperature==31);
    f.identifier=2; assert(!RmDm_Decode(&f,0,1,&l,&out));
    f.identifier=0; f.data[0]=0x12; assert(!RmDm_Decode(&f,0,1,&l,&out));
    RmDm_ReadRegister(&f,RM_CAN_1,1,0x15);f.identifier=0;
    f.data[4]=0;f.data[5]=0;f.data[6]=0x48;f.data[7]=0x41;
    uint8_t reg; uint32_t bits;
    assert(RmDm_RegisterReply(&f,0,1,&reg,&bits) && reg==0x15 && bits==0x41480000);
    f.identifier=0x7ff;assert(RmDm_RegisterReply(&f,0,1,&reg,&bits));
    assert(!RmDm_TorqueFrame(&f,RM_CAN_1,1,&l,10.1f));
    RmDm_TimeoutFrame(&f,RM_CAN_1,1);assert(RmDm_TimeoutFrameSafe(&f));
    assert(!RmDm_ServiceFrameSafe(&f)); /* Ordinary service API still rejects writes. */
    f.data[4]=f.data[5]=0;assert(!RmDm_TimeoutFrameSafe(&f));
    RmDm_TimeoutFrame(&f,RM_CAN_1,1);f.data[3]=0x0a;assert(!RmDm_TimeoutFrameSafe(&f));
    PIDInstance pid;
    const PID_Init_Config_s cfg={.Kp=.2f,.Ki=.5f,.MaxOut=7,.IntegralLimit=7,
        .Improve=PID_Trapezoid_Intergral|PID_Integral_Limit|PID_HoldIntegral};
    PIDInit(&pid,&cfg);
    for(unsigned i=0;i<200;++i) (void)PIDCalculate(&pid,0,1,.005f);
    float load=pid.Iout; assert(load>.49f);
    near(PIDCalculate(&pid,1,1,.005f),load,.00001f);
    PIDReset(&pid);near(PIDCalculate(&pid,1,1,.005f),0,.00001f);
    puts("PASS: DM feedback units, torque-only MIT, service whitelist, read registers and holding torque");
}
