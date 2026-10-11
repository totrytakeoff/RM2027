/* Channel packing and ET08 mappings follow RM2026 et08/dt7 backends.
   Stream synchronization, receive-time health and explicit edge consumption
   replace platform UART callbacks and daemon coupling. */
#include "remote.h"
#include <string.h>
#include <stdlib.h>
static uint16_t bits11(const uint8_t *b,unsigned bit)
{
    unsigned byte=bit/8U,shift=bit%8U;
    uint32_t word=b[byte]|((uint32_t)b[byte+1U]<<8);
    if (shift>5U) word|=(uint32_t)b[byte+2U]<<16;
    return (uint16_t)((word>>shift)&0x7ffU);
}
static void axis(RemoteControlState *s,unsigned id,int raw,uint16_t span)
{
    s->axis_raw[id]=(int16_t)raw;
    float v=(float)raw/(float)span;
    s->axis_normalized[id]=v>1?1:v < -1?-1:v; s->axis_valid_mask|=1U<<id;
}
bool RmRemote_Init(RmRemote *s,const RmRemoteConfig *c)
{
    if (!s) return false;
    memset(s,0,sizeof(*s));
    if (!c || (unsigned)c->protocol>RM_REMOTE_DBUS || !c->timeout_ms || c->timeout_ms>10000U ||
        !c->span || c->span>2047U || c->center>2047U) return false;
    const uint8_t channels[]={c->right_x,c->right_y,c->left_x,c->left_y,c->combined_sa_sb,
        c->combined_sd_sc,c->knob_left,c->knob_right};
    for (unsigned i=0;i<sizeof(channels);++i) if (channels[i]>=16U) return false;
    s->config=*c; s->initialized=true; return true;
}
void RmRemote_Discontinuity(RmRemote *s,uint32_t generation)
{
    if (!s) return;
    s->generation=generation; s->used=0; s->good_streak=0; s->locked=false;
    s->have_good=false; memset(&s->state,0,sizeof(s->state)); ++s->stats.discontinuities;
}
static unsigned switch_index(const RmRemoteConfig *c,uint16_t raw)
{
    unsigned best=6U,diff=65535U;
    for (unsigned i=0;i<6U;++i) {
        unsigned d=(unsigned)abs((int)raw-(int)c->switch_levels[i]);
        if (d<diff) { diff=d; best=i; }
    }
    return diff<=c->switch_tolerance?best:6U;
}
static void switches(RemoteControlState *v,unsigned upper,unsigned lower,unsigned combined)
{
    if (combined>=6U) return;
    v->switches[upper]=combined<=2U?REMOTE_SWITCH_UP:REMOTE_SWITCH_DOWN;
    v->switches[lower]=(RemoteControlSwitchPosition)(REMOTE_SWITCH_UP+combined%3U);
    v->switch_valid_mask|=(1U<<upper)|(1U<<lower);
}
static bool decode(RmRemote *s,RemoteControlState *v)
{
    const uint8_t *b=s->buffer; const RmRemoteConfig *c=&s->config;
    memset(v,0,sizeof(*v));
    if (c->protocol==RM_REMOTE_SBUS) {
        if (b[0]!=0x0fU || (b[23]&0xf0U)!=0U ||
            !(b[24]==0U || (b[24]&0xcfU)==0x04U)) return false;
        for (unsigned i=0;i<16U;++i) s->raw_channels[i]=bits11(b+1U,i*11U);
        v->type=REMOTE_CONTROL_TYPE_ET08;
        v->capabilities=REMOTE_CAPABILITY_STICKS|REMOTE_CAPABILITY_SWITCHES|REMOTE_CAPABILITY_KNOBS|REMOTE_CAPABILITY_AUX_AXES;
        v->frame_lost=(b[23]>>2)&1U; v->failsafe=(b[23]>>3)&1U;
        const unsigned target[]={REMOTE_AXIS_RIGHT_X,REMOTE_AXIS_RIGHT_Y,REMOTE_AXIS_LEFT_X,
            REMOTE_AXIS_LEFT_Y,REMOTE_AXIS_KNOB_LEFT,REMOTE_AXIS_KNOB_RIGHT};
        const uint8_t source[]={c->right_x,c->right_y,c->left_x,c->left_y,c->knob_left,c->knob_right};
        for (unsigned i=0;i<6U;++i) axis(v,target[i],(int)s->raw_channels[source[i]]-c->center,c->span);
        for (unsigned i=0;i<8U;++i) axis(v,REMOTE_AXIS_AUX_0+i,(int)s->raw_channels[i+8U]-c->center,c->span);
        switches(v,0,1,switch_index(c,s->raw_channels[c->combined_sa_sb]));
        switches(v,3,2,switch_index(c,s->raw_channels[c->combined_sd_sc]));
    } else {
        uint16_t ch[5];
        for (unsigned i=0;i<4U;++i) ch[i]=bits11(b,i*11U);
        ch[4]=(uint16_t)(b[16]|((uint16_t)b[17]<<8));
        unsigned sw1=(b[5]>>4)&3U,sw2=(b[5]>>6)&3U;
        if (!sw1 || !sw2 || b[12]>1U || b[13]>1U) return false;
        for (unsigned i=0;i<5U;++i) if (abs((int)ch[i]-1024)>660) return false;
        v->type=REMOTE_CONTROL_TYPE_DT7;
        v->capabilities=REMOTE_CAPABILITY_STICKS|REMOTE_CAPABILITY_SWITCHES|REMOTE_CAPABILITY_DIAL|
            REMOTE_CAPABILITY_KEYBOARD|REMOTE_CAPABILITY_MOUSE|REMOTE_CAPABILITY_BUTTONS;
        const unsigned target[]={REMOTE_AXIS_RIGHT_X,REMOTE_AXIS_RIGHT_Y,REMOTE_AXIS_LEFT_X,REMOTE_AXIS_LEFT_Y,REMOTE_AXIS_DIAL};
        for (unsigned i=0;i<5U;++i) axis(v,target[i],(int)ch[i]-1024,660);
        v->switches[0]=sw1==1?REMOTE_SWITCH_UP:sw1==2?REMOTE_SWITCH_DOWN:REMOTE_SWITCH_MIDDLE;
        v->switches[1]=sw2==1?REMOTE_SWITCH_UP:sw2==2?REMOTE_SWITCH_DOWN:REMOTE_SWITCH_MIDDLE;
        v->switch_valid_mask=3U;
        v->mouse_x=(int16_t)(b[6]|((uint16_t)b[7]<<8)); v->mouse_y=(int16_t)(b[8]|((uint16_t)b[9]<<8));
        v->mouse_z=(int16_t)(b[10]|((uint16_t)b[11]<<8)); v->keys_down=b[14]|((uint16_t)b[15]<<8);
        v->buttons_supported=REMOTE_BUTTON_MOUSE_LEFT|REMOTE_BUTTON_MOUSE_RIGHT;
        v->buttons_down=(b[12]?REMOTE_BUTTON_MOUSE_LEFT:0U)|(b[13]?REMOTE_BUTTON_MOUSE_RIGHT:0U);
    }
    return true;
}
void RmRemote_Push(RmRemote *s,const uint8_t *data,size_t size,uint32_t sample_ms,uint32_t generation)
{
    if (!s || !s->initialized || !data) return;
    if (generation!=s->generation) RmRemote_Discontinuity(s,generation);
    if (s->have_good && sample_ms-s->last_good_ms>=s->config.timeout_ms) RmRemote_Discontinuity(s,generation);
    unsigned frame_size=s->config.protocol==RM_REMOTE_SBUS?25U:18U;
    for (size_t i=0;i<size;++i) {
        if (s->config.protocol==RM_REMOTE_SBUS && !s->used && data[i]!=0x0fU) continue;
        s->buffer[s->used++]=data[i];
        if (s->used<frame_size) continue;
        RemoteControlState next;
        if (!decode(s,&next)) {
            ++s->stats.bad_frames; s->locked=false; s->good_streak=0;
            memmove(s->buffer,s->buffer+1U,--s->used); continue;
        }
        s->used=0; ++s->stats.frames;
        if (next.failsafe) {
            ++s->stats.failsafe; s->state.failsafe=1U; s->state.data_valid=0;
            s->have_good=false; s->locked=false; s->good_streak=0; continue;
        }
        if (next.frame_lost) { ++s->stats.frame_lost; continue; }
        if (s->good_streak<2U) ++s->good_streak;
        if (!s->locked && s->good_streak<2U) continue;
        bool was_good=s->have_good; s->locked=true; s->have_good=true;
        next.sample_sequence=s->stats.good_frames+1U; next.sample_time_ms=sample_ms;
        next.link_online=next.data_valid=1U;
        if (was_good) {
            next.keys_pressed=s->state.keys_pressed|(next.keys_down&~s->state.keys_down);
            next.keys_released=s->state.keys_released|(s->state.keys_down&~next.keys_down);
            next.buttons_pressed=s->state.buttons_pressed|(next.buttons_down&~s->state.buttons_down);
            next.buttons_released=s->state.buttons_released|(s->state.buttons_down&~next.buttons_down);
            next.switch_changed_mask=s->state.switch_changed_mask;
            for (unsigned j=0;j<REMOTE_CONTROL_SWITCH_COUNT;++j) if (next.switches[j]!=s->state.switches[j]) next.switch_changed_mask|=1U<<j;
        }
        s->state=next; s->last_good_ms=sample_ms; ++s->stats.good_frames;
    }
}
bool RmRemote_Read(const RmRemote *s,uint32_t now_ms,RemoteControlState *out)
{
    if (!s || !out) return false;
    *out=s->state;
    bool good=s->initialized && s->have_good && !out->failsafe && now_ms-s->last_good_ms<s->config.timeout_ms;
    if (!good) {
        RemoteControlType type=out->type; uint32_t caps=out->capabilities; uint8_t fs=out->failsafe;
        memset(out,0,sizeof(*out)); out->type=type; out->capabilities=caps; out->failsafe=fs;
    }
    return good;
}
bool RmRemote_Take(RmRemote *s,uint32_t now_ms,RemoteControlState *out)
{
    bool valid=RmRemote_Read(s,now_ms,out);
    if (s) {
        s->state.keys_pressed=s->state.keys_released=s->state.buttons_pressed=s->state.buttons_released=0;
        s->state.switch_changed_mask=0;
    }
    return valid;
}
