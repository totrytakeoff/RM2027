#ifndef ROBOT_TUNING_TELEMETRY_H
#define ROBOT_TUNING_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the dedicated tuning UART without enabling generic logs. */
bool TuningTelemetry_Init(void);

/** Build and start one non-blocking VOFA+ JustFloat DMA frame. */
void TuningTelemetry_Publish(uint32_t now_ms);

/* Fill one self-contained field set and return its actual channel count. */
uint16_t TuningTelemetry_GetYawChannels(float channels[]);
uint16_t TuningTelemetry_GetPitchChannels(float channels[]);
uint16_t TuningTelemetry_GetLoaderChannels(float channels[]);
uint16_t TuningTelemetry_GetFrictionLeftChannels(float channels[]);
uint16_t TuningTelemetry_GetFrictionRightChannels(float channels[]);
uint16_t TuningTelemetry_GetShootStateChannels(float channels[]);

uint32_t TuningTelemetry_GetSentCount(void);
uint32_t TuningTelemetry_GetDroppedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_TUNING_TELEMETRY_H */
