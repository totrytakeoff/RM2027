#include "dm.h"
#include <math.h>
#include <string.h>
bool RmDm_LimitsValid(const RmDmLimits *l)
{
    return l && isfinite(l->position) && l->position>0 && l->position<1000000.0f && isfinite(l->velocity) &&
        l->velocity>0 && l->velocity<1000000.0f && isfinite(l->torque) && l->torque>0 && l->torque<1000000.0f;
}
static float unpack(unsigned v, float limit, unsigned max)
{ return ((float)v/(float)max*2.0f-1.0f)*limit; }
static unsigned pack(float v, float limit, unsigned max)
{ return (unsigned)lroundf((fmaxf(-limit,fminf(limit,v))/limit+1.0f)*0.5f*(float)max); }
bool RmDm_Decode(const RmCanFrame *f,uint16_t master,uint8_t id,const RmDmLimits *l,RmDmFeedback *out)
{
    if (!f || !out || f->extended || f->length!=8 || f->identifier!=master ||
        (f->data[0]&15U)!=id || !RmDm_LimitsValid(l)) return false;
    *out=(RmDmFeedback){.timestamp_us=f->timestamp_us,.generation=f->generation,
        .state=f->data[0]>>4,.mos_temperature=f->data[6],.rotor_temperature=f->data[7],
        .position_rad=unpack((f->data[1]<<8)|f->data[2],l->position,65535),
        .velocity_rad_s=unpack((f->data[3]<<4)|(f->data[4]>>4),l->velocity,4095),
        .torque_nm=unpack(((f->data[4]&15U)<<8)|f->data[5],l->torque,4095)};
    return true;
}
bool RmDm_TorqueFrame(RmCanFrame *f,RmCanId bus,uint16_t id,const RmDmLimits *l,float torque)
{
    if (!f || !id || id>=0x7ff || (unsigned)bus>=RM_CAN_COUNT || !RmDm_LimitsValid(l) || !isfinite(torque) || fabsf(torque)>l->torque) return false;
    unsigned p=pack(0,l->position,65535),v=pack(0,l->velocity,4095),t=pack(torque,l->torque,4095);
    *f=(RmCanFrame){.bus=bus,.identifier=id,.length=8};
    f->data[0]=(uint8_t)(p>>8); f->data[1]=(uint8_t)p; f->data[2]=(uint8_t)(v>>4);
    f->data[3]=(uint8_t)(v<<4); /* Kp=0, Kd=0: outer loops command torque only. */
    f->data[6]=(uint8_t)(t>>8); f->data[7]=(uint8_t)t;
    return true;
}
void RmDm_SpecialFrame(RmCanFrame *f,RmCanId bus,uint16_t id,bool enable)
{
    *f=(RmCanFrame){.bus=bus,.identifier=id,.length=8};
    memset(f->data,255,7); f->data[7]=enable?0xfc:0xfd;
}
void RmDm_ReadRegister(RmCanFrame *f,RmCanId bus,uint16_t id,uint8_t reg)
{
    *f=(RmCanFrame){.bus=bus,.identifier=0x7ff,.length=8,
        .data={(uint8_t)id,(uint8_t)(id>>8),0x33,reg,0,0,0,0}};
}
bool RmDm_RegisterReply(const RmCanFrame *f,uint16_t master,uint16_t id,uint8_t *reg,uint32_t *bits)
{
    if (!f || !reg || !bits || f->extended || f->length!=8 || (f->identifier!=master && f->identifier!=0x7ff) ||
        f->data[0]!=(uint8_t)id || f->data[1]!=(uint8_t)(id>>8) || f->data[2]!=0x33) return false;
    *reg=f->data[3]; *bits=(uint32_t)f->data[4]|((uint32_t)f->data[5]<<8)|
        ((uint32_t)f->data[6]<<16)|((uint32_t)f->data[7]<<24);
    return true;
}
bool RmDm_ServiceFrameSafe(const RmCanFrame *f)
{
    if (!f || f->extended || f->length!=8 || (unsigned)f->bus>=RM_CAN_COUNT || f->identifier>0x7ff) return false;
    bool disable=f->identifier>0 && f->identifier<0x7ff && f->data[7]==0xfd;
    for (unsigned i=0;i<7;++i) disable=disable && f->data[i]==0xff;
    if (disable) return true;
    if (f->identifier!=0x7ff || f->data[2]!=0x33 ||
        !(f->data[0] || f->data[1]) || f->data[1]>7 || (f->data[0]==0xff && f->data[1]==7)) return false;
    for (unsigned i=4;i<8;++i) if (f->data[i]) return false;
    return f->data[3]==0x09 || f->data[3]==0x0a ||
        f->data[3]==0x15 || f->data[3]==0x16 || f->data[3]==0x17;
}

void RmDm_TimeoutFrame(RmCanFrame *f,RmCanId bus,uint16_t id)
{
    *f=(RmCanFrame){.bus=bus,.identifier=0x7ff,.length=8,
        .data={(uint8_t)id,(uint8_t)(id>>8),0x55,0x09,0xe8,0x03,0,0}};
}
bool RmDm_TimeoutFrameSafe(const RmCanFrame *f)
{
    return f && !f->extended && f->length==8 && (unsigned)f->bus<RM_CAN_COUNT &&
        f->identifier==0x7ff && (f->data[0] || f->data[1]) && f->data[1]<8 &&
        !(f->data[0]==0xff && f->data[1]==7) && f->data[2]==0x55 && f->data[3]==0x09 &&
        f->data[4]==0xe8 && f->data[5]==0x03 && f->data[6]==0 && f->data[7]==0;
}
