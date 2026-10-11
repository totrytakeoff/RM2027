#ifndef RM_BMI088_H
#define RM_BMI088_H
#include "rm_platform.h"
typedef enum { RM_BMI_BOOT, RM_BMI_CONFIGURING, RM_BMI_READY, RM_BMI_FAILED } RmBmiState;
typedef struct {
    float accel[3], gyro[3], temperature_c;
    uint64_t accel_us, gyro_us;
    uint32_t sequence;
    bool valid;
} RmImuSample;
typedef struct {
    const RmSensorBus *bus;
    RmBmiState state;
    RmImuSample sample;
    uint64_t start_us, next_us;
    uint32_t timeout_us, errors;
    uint8_t phase, register_index, accel_id, gyro_id;
    bool verify;
} RmBmi088;
/* Call from the sensor owner task. Init performs no SPI waits. */
void RmBmi088_Init(RmBmi088 *sensor,const RmSensorBus *bus,uint32_t timeout_us);
/* One bounded initialization operation or sample batch per tick. */
void RmBmi088_Tick(RmBmi088 *sensor,uint64_t now_us);
#endif
