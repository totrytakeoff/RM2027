/* DJI mapping, filtering and cascade order adapted from RM2026 dji_motor.c.
   Transport handles, live feedback pointers and automatic angle-sign inference
   are replaced by explicit binding and copied inputs. Wire output is ESC units. */
#include "dji.h"
#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
static int16_t be16(const uint8_t *p) { return (int16_t)(((uint16_t)p[0]<<8)|p[1]); }
bool RmDji_Decode(RmDjiFeedbackState *s,const RmCanFrame *f,uint32_t max_gap_ms)
{
    if (!s || !f || f->extended || f->length!=8U || !max_gap_ms) return false;
    uint16_t encoder=(uint16_t)be16(f->data); if (encoder>=8192U) return false;
    RmDjiFeedback next=s->value;
    if (s->initialized && f->timestamp_us<next.timestamp_us) return false;
    bool discontinuity=s->initialized && (f->generation!=next.generation ||
        f->timestamp_us-next.timestamp_us>=(uint64_t)max_gap_ms*1000U);
    int32_t delta=(int32_t)encoder-(int32_t)next.encoder;
    bool ambiguous=delta==4096 || delta == -4096;
    if (delta>4096) delta-=8192; else if (delta < -4096) delta+=8192;
    if (!s->initialized) { next.encoder_total=encoder; next.position_valid=true; }
    else if (discontinuity || ambiguous) next.position_valid=false;
    else {
        if ((delta>0 && next.encoder_total>INT64_MAX-delta) ||
            (delta<0 && next.encoder_total<INT64_MIN-delta)) return false;
        next.encoder_total+=delta;
    }
    next.encoder=encoder; next.rpm=be16(f->data+2); next.current_raw=be16(f->data+4);
    if (next.rpm==INT16_MIN) return false;
    /* Half-revolution aliasing cannot be reconstructed after a slow/gapped RX. */
    if (s->initialized && (double)abs((int)next.rpm)*(double)(f->timestamp_us-next.timestamp_us)/60000000.0>=0.45)
        next.position_valid=false;
    next.speed_deg_s=0.15f*next.speed_deg_s+0.85f*(float)next.rpm*6.0f;
    next.speed_rad_s=0.15f*next.speed_rad_s+0.85f*(float)next.rpm*0.10471975512f;
    next.current_filtered=(int16_t)(0.1f*(float)next.current_filtered+0.9f*(float)next.current_raw);
    next.temperature=f->data[6]; next.angle_deg=(float)((double)next.encoder_total*(360.0/8192.0));
    next.angle_rad=(float)((double)next.encoder_total*(6.283185307179586/8192.0));
    next.timestamp_us=f->timestamp_us; next.generation=f->generation; ++next.sequence; next.valid=true;
    s->value=next; s->initialized=true; return true;
}
static bool sign_valid(int s) { return s==1 || s==-1; }
static bool pid_valid(const PID_Init_Config_s *p)
{
    return isfinite(p->Kp) && isfinite(p->Ki) && isfinite(p->Kd) &&
        isfinite(p->MaxOut) && p->MaxOut>0 && isfinite(p->DeadBand) && p->DeadBand>=0 &&
        isfinite(p->IntegralLimit) && p->IntegralLimit>=0 &&
        isfinite(p->CoefA) && isfinite(p->CoefB) && p->CoefB>=0 &&
        (!(p->Improve&PID_ChangingIntegrationRate) || p->CoefA>0) &&
        isfinite(p->Output_LPF_RC) && p->Output_LPF_RC>=0 &&
        isfinite(p->Derivative_LPF_RC) && p->Derivative_LPF_RC>=0;
}
bool RmDji_Init(RmDjiRegistry *r,const RmDjiConfig *configs,unsigned count)
{
    if (!r) return false;
    memset(r,0,sizeof(*r));
    if (count>RM_DJI_MAX_MOTORS || (count && !configs)) return false;
    for (unsigned i=0;i<count;++i) {
        const RmDjiConfig *c=&configs[i]; RmDjiMotor *m=&r->motors[i];
        if ((unsigned)c->bus>=RM_CAN_COUNT || (unsigned)c->model>RM_DJI_GM6020_VOLTAGE ||
            !c->id || c->id>(c->model==RM_DJI_GM6020_VOLTAGE?7U:8U) || c->loops>7U ||
            !sign_valid(c->reference_sign) || !sign_valid(c->output_sign) ||
            !sign_valid(c->angle_sign) || !sign_valid(c->speed_sign) ||
            !c->feedback_timeout_ms || c->feedback_timeout_ms>1000U ||
            !c->command_timeout_ms || c->command_timeout_ms>1000U ||
            ((c->loops&RM_DJI_LOOP_ANGLE) && !pid_valid(&c->angle_pid)) ||
            ((c->loops&RM_DJI_LOOP_SPEED) && !pid_valid(&c->speed_pid)) ||
            ((c->loops&RM_DJI_LOOP_CURRENT) && !pid_valid(&c->current_pid))) goto invalid;
        m->config=*c;
        m->feedback_id=(c->model==RM_DJI_GM6020_VOLTAGE?0x204U:0x200U)+c->id;
        m->command_id=c->model==RM_DJI_GM6020_VOLTAGE?(c->id<=4U?0x1ffU:0x2ffU):(c->id<=4U?0x200U:0x1ffU);
        m->command_slot=(c->id-1U)%4U;
        for (unsigned j=0;j<i;++j) {
            const RmDjiMotor *other=&r->motors[j];
            if (other->config.bus==c->bus && (other->feedback_id==m->feedback_id ||
                (other->command_id==m->command_id && other->command_slot==m->command_slot))) goto invalid;
        }
        PIDInit(&m->angle_pid,&c->angle_pid); PIDInit(&m->speed_pid,&c->speed_pid); PIDInit(&m->current_pid,&c->current_pid);
    }
    r->count=count; return true;
invalid:
    memset(r,0,sizeof(*r)); return false;
}
bool RmDji_Receive(RmDjiRegistry *r,const RmCanFrame *f)
{
    if (!r || !f) return false;
    for (unsigned i=0;i<r->count;++i) {
        RmDjiMotor *m=&r->motors[i];
        if (!f->extended && f->bus==m->config.bus && f->identifier==m->feedback_id) {
            if (RmDji_Decode(&m->feedback,f,m->config.feedback_timeout_ms)) return true;
            ++r->bad_feedback; return false;
        }
    }
    return false;
}
bool RmDji_Read(const RmDjiRegistry *r,unsigned i,uint64_t now,RmDjiFeedback *out)
{
    if (!r || !out || i>=r->count) return false;
    const RmDjiMotor *m=&r->motors[i]; *out=m->feedback.value;
    out->valid=out->valid && now>=out->timestamp_us && now-out->timestamp_us<(uint64_t)m->config.feedback_timeout_ms*1000U;
    if (!out->valid) out->position_valid=false;
    return out->valid;
}
bool RmDji_AllOnline(const RmDjiRegistry *r,uint64_t now)
{
    if (!r || !r->count) return false;
    for (unsigned i=0;i<r->count;++i) { RmDjiFeedback f; if (!RmDji_Read(r,i,now,&f)) return false; }
    return true;
}
bool RmDji_RebasePosition(RmDjiRegistry *r,unsigned i,int64_t total,uint64_t now)
{
    RmDjiFeedback current;
    if (!r || i>=r->count || r->motors[i].active || !RmDji_Read(r,i,now,&current)) return false;
    RmDjiMotor *m=&r->motors[i];
    m->feedback.value.encoder_total=total;
    m->feedback.value.angle_deg=(float)((double)total*(360.0/8192.0));
    m->feedback.value.angle_rad=(float)((double)total*(6.283185307179586/8192.0));
    m->feedback.value.position_valid=true;
    PIDReset(&m->angle_pid); PIDReset(&m->speed_pid); PIDReset(&m->current_pid);
    return true;
}
static void reset(RmDjiMotor *m,uint8_t mask)
{
    if (mask&RM_DJI_LOOP_ANGLE) PIDReset(&m->angle_pid);
    if (mask&RM_DJI_LOOP_SPEED) PIDReset(&m->speed_pid);
    if (mask&RM_DJI_LOOP_CURRENT) PIDReset(&m->current_pid);
}
static float output(RmDjiMotor *m,const RmDjiCommand *c,uint64_t now,float dt,bool permit,uint32_t epoch)
{
    RmDjiFeedback f;
    f=m->feedback.value; uint32_t ms=(uint32_t)(now/1000U);
    uint8_t loop=c?c->outer_loop:0U;
    bool online=f.valid && now>=f.timestamp_us && now-f.timestamp_us<(uint64_t)m->config.feedback_timeout_ms*1000U;
    bool command_valid=c && c->enabled && c->epoch==epoch && (int32_t)(c->deadline_ms-ms)>0 &&
        (uint32_t)(c->deadline_ms-ms)<=m->config.command_timeout_ms &&
        (loop==0U || loop==1U || loop==2U || loop==4U) && (loop&m->config.loops)==loop &&
        (loop!=0U || m->config.allow_direct) && c->external_mask<=3U && c->reset_mask<=7U &&
        isfinite(c->reference) && isfinite(c->speed_ff) && isfinite(c->current_ff) &&
        isfinite(c->external_angle) && isfinite(c->external_speed) &&
        (!c->external_mask || ms-c->external_time_ms<m->config.feedback_timeout_ms) &&
        (loop!=RM_DJI_LOOP_ANGLE || (c->external_mask&RM_DJI_EXTERNAL_ANGLE) || f.position_valid);
    if (!permit || !online || !command_valid || !isfinite(dt) || dt<=0 || dt>0.02f) {
        reset(m,7U); m->active=false; return 0;
    }
    if (!m->active) reset(m,7U);
    m->active=true; reset(m,c->reset_mask);
    float ref=c->reference*(float)m->config.reference_sign;
    if (loop==RM_DJI_LOOP_ANGLE) {
        ref=PIDCalculate(&m->angle_pid,
            (c->external_mask&RM_DJI_EXTERNAL_ANGLE)?c->external_angle:f.angle_deg*(float)m->config.angle_sign,ref,dt);
        if (m->angle_pid.dt<=0) { reset(m,7U); m->active=false; return 0; }
    }
    if ((loop==RM_DJI_LOOP_ANGLE || loop==RM_DJI_LOOP_SPEED) && (m->config.loops&RM_DJI_LOOP_SPEED)) {
        float measure=(c->external_mask&RM_DJI_EXTERNAL_SPEED)?c->external_speed:
            (m->config.speed_in_rad_s?f.speed_rad_s:f.speed_deg_s)*(float)m->config.speed_sign;
        ref=PIDCalculate(&m->speed_pid,measure,ref+c->speed_ff,dt);
        if (m->speed_pid.dt<=0) { reset(m,7U); m->active=false; return 0; }
    }
    ref+=c->current_ff;
    if (loop && (m->config.loops&RM_DJI_LOOP_CURRENT)) {
        ref=PIDCalculate(&m->current_pid,(float)f.current_filtered,ref,dt);
        if (m->current_pid.dt<=0) { reset(m,7U); m->active=false; return 0; }
    }
    ref*=(float)m->config.output_sign;
    float limit=m->config.model==RM_DJI_M2006?10000.0f:m->config.model==RM_DJI_M3508?16384.0f:30000.0f;
    if (!isfinite(ref)) { reset(m,7U); m->active=false; return 0; }
    return fmaxf(-limit,fminf(limit,ref));
}
unsigned RmDji_Step(RmDjiRegistry *r,const RmDjiCommand *commands,unsigned count,
    uint64_t now,float dt,bool permit,uint32_t epoch,uint32_t deadline,RmDjiGroup groups[RM_DJI_MAX_GROUPS])
{
    if (!r || !groups) return 0;
    memset(groups,0,RM_DJI_MAX_GROUPS*sizeof(*groups)); unsigned n=0;
    for (unsigned i=0;i<r->count;++i) {
        RmDjiMotor *m=&r->motors[i];
        m->output=output(m,commands && i<count?&commands[i]:NULL,now,dt,permit,epoch);
        unsigned g;
        for (g=0;g<n;++g) if (groups[g].frame.bus==m->config.bus && groups[g].frame.identifier==m->command_id) break;
        if (g==n) {
            if (n==RM_DJI_MAX_GROUPS) return 0;
            groups[n]=(RmDjiGroup){.frame={.bus=m->config.bus,.identifier=m->command_id,.length=8},.epoch=epoch,.deadline_ms=deadline}; ++n;
        }
        if (m->active) {
            const RmDjiCommand *c=&commands[i];
            uint32_t now_ms=(uint32_t)(now/1000U);
            const uint32_t deadlines[]={c->deadline_ms,
                (uint32_t)(m->feedback.value.timestamp_us/1000U)+m->config.feedback_timeout_ms,
                c->external_mask?c->external_time_ms+m->config.feedback_timeout_ms:groups[g].deadline_ms};
            for (unsigned j=0;j<3U;++j) if ((int32_t)(deadlines[j]-now_ms)<(int32_t)(groups[g].deadline_ms-now_ms))
                groups[g].deadline_ms=deadlines[j];
        }
        int16_t set=(int16_t)m->output; unsigned slot=m->command_slot*2U;
        groups[g].frame.data[slot]=(uint8_t)((uint16_t)set>>8); groups[g].frame.data[slot+1U]=(uint8_t)set;
    }
    return n;
}
