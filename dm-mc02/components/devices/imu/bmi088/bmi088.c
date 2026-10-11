/* Register setup/scales follow RM2026 BMI088driver and Bosch BMI088 datasheet.
 * Reimplemented as an explicit, bounded bringup state machine with injected SPI.
 * Author attribution for the original register driver: Wang Hongxi. */
#include "bmi088.h"
#include <string.h>
#include <math.h>
typedef struct { RmImuChip chip; uint8_t reg,value; uint32_t wait_us; } Setup;
static const Setup setup[]={
    {RM_IMU_ACCEL,0x7D,0x04,50000}, {RM_IMU_ACCEL,0x7C,0x00,50000},
    {RM_IMU_ACCEL,0x40,0xAC,2000}, {RM_IMU_ACCEL,0x41,0x01,2000},
    {RM_IMU_GYRO,0x0F,0x00,2000}, {RM_IMU_GYRO,0x10,0x81,2000},
    {RM_IMU_GYRO,0x11,0x00,30000}
};
static bool read_regs(RmBmi088 *s,RmImuChip chip,uint8_t reg,uint8_t *data,unsigned n)
{
    uint8_t tx[16]={0},rx[16]={0};
    unsigned skip=chip==RM_IMU_ACCEL?2U:1U;
    if (n+skip>sizeof(tx)) return false;
    tx[0]=reg|0x80U;
    if (s->bus->transfer(s->bus->context,chip,tx,rx,n+skip)!=RM_OK) return false;
    memcpy(data,rx+skip,n); return true;
}
static bool write_reg(RmBmi088 *s,RmImuChip chip,uint8_t reg,uint8_t value)
{
    uint8_t tx[2]={reg,value},rx[2];
    return s->bus->transfer(s->bus->context,chip,tx,rx,2U)==RM_OK;
}
static void fail(RmBmi088 *s) { s->state=RM_BMI_FAILED; s->sample.valid=false; ++s->errors; }
void RmBmi088_Init(RmBmi088 *s,const RmSensorBus *bus,uint32_t timeout_us)
{
    if (!s) return;
    memset(s,0,sizeof(*s)); s->bus=bus; s->timeout_us=timeout_us;
    if (!bus || !bus->transfer || !bus->now_us || timeout_us==0U) { fail(s); return; }
    s->start_us=bus->now_us(bus->context); s->next_us=s->start_us;
}
static int16_t signed_le(const uint8_t *p) { return (int16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); }
void RmBmi088_Tick(RmBmi088 *s,uint64_t now_us)
{
    if (!s || s->state==RM_BMI_FAILED || now_us<s->next_us) return;
    if (s->state!=RM_BMI_READY && now_us-s->start_us>s->timeout_us) { fail(s); return; }
    if (s->state==RM_BMI_READY) {
        uint8_t accel[6],gyro[6],temp[2];
        RmImuSample next={0};
        if (!read_regs(s,RM_IMU_ACCEL,0x12,accel,6U)) { fail(s); return; }
        next.accel_us=s->bus->now_us(s->bus->context);
        if (!read_regs(s,RM_IMU_GYRO,0x02,gyro,6U)) { fail(s); return; }
        next.gyro_us=s->bus->now_us(s->bus->context);
        if (!read_regs(s,RM_IMU_ACCEL,0x22,temp,2U)) { fail(s); return; }
        for (unsigned i=0;i<3U;++i) {
            next.accel[i]=(float)signed_le(&accel[i*2U])*(6.0f*9.80665f/32768.0f);
            next.gyro[i]=(float)signed_le(&gyro[i*2U])*(2000.0f*0.01745329251994f/32768.0f);
        }
        int32_t t=((int32_t)temp[0]<<3)|(temp[1]>>5);
        if (t>1023) t-=2048;
        next.temperature_c=23.0f+(float)t*0.125f;
        next.valid=isfinite(next.temperature_c) && next.temperature_c>-40.0f && next.temperature_c<85.0f;
        if (!next.valid) { fail(s); return; }
        next.sequence=s->sample.sequence+1U;
        s->sample=next; return;
    }
    s->state=RM_BMI_CONFIGURING;
    switch (s->phase) {
    case 0: /* First accel SPI access switches the chip out of I2C mode. */
        if (!read_regs(s,RM_IMU_ACCEL,0x00,&s->accel_id,1U)) { fail(s); return; }
        s->next_us=now_us+2000U; ++s->phase; break;
    case 1:
        if (!write_reg(s,RM_IMU_ACCEL,0x7E,0xB6)) { fail(s); return; }
        s->next_us=now_us+80000U; ++s->phase; break;
    case 2:
        if (!read_regs(s,RM_IMU_ACCEL,0x00,&s->accel_id,1U)) { fail(s); return; }
        s->next_us=now_us+2000U; ++s->phase; break;
    case 3:
        if (!read_regs(s,RM_IMU_ACCEL,0x00,&s->accel_id,1U) || s->accel_id!=0x1EU ||
            !write_reg(s,RM_IMU_GYRO,0x14,0xB6)) { fail(s); return; }
        s->next_us=now_us+80000U; ++s->phase; break;
    case 4:
        if (!read_regs(s,RM_IMU_GYRO,0x00,&s->gyro_id,1U) || s->gyro_id!=0x0FU) { fail(s); return; }
        ++s->phase; break;
    default:
        if (s->register_index>=sizeof(setup)/sizeof(setup[0])) { s->state=RM_BMI_READY; return; }
        const Setup *r=&setup[s->register_index];
        if (!s->verify) {
            if (!write_reg(s,r->chip,r->reg,r->value)) { fail(s); return; }
            s->verify=true; s->next_us=now_us+r->wait_us;
        } else {
            uint8_t value;
            if (!read_regs(s,r->chip,r->reg,&value,1U) || value!=r->value) { fail(s); return; }
            s->verify=false; ++s->register_index; s->next_us=now_us+2000U;
        }
        break;
    }
}
