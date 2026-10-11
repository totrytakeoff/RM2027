#ifndef RM_LOGGING_H
#define RM_LOGGING_H
#include <stdint.h>
#include "rm_platform.h"
typedef enum {
    RM_LOG_BOOT=1, RM_LOG_CLOCKS, RM_LOG_IMU_READY, RM_LOG_IMU_FAILED,
    RM_LOG_TASK_FAULT, RM_LOG_HEARTBEAT, RM_LOG_SERIAL_STATS, RM_LOG_CAN_STATS,
    RM_LOG_IMU_SAMPLE, RM_LOG_RESET_FLAGS, RM_LOG_PREVIOUS_FAULT,
    RM_LOG_TASK_TIMING, RM_LOG_TASK_STACK, RM_LOG_INS_STATE, RM_LOG_INS_SAMPLE,
    RM_LOG_REMOTE_SAMPLE, RM_LOG_DJI_CANDIDATES, RM_LOG_BINDING, RM_LOG_MOTOR_IO,
    RM_LOG_SERIAL_RECOVERY
} RmLogEvent;
void RmLog_Publish(RmLogEvent event, uint32_t a, uint32_t b);
void RmLog_Drain(unsigned budget);
uint32_t RmLog_Dropped(void);
/* Assembly only, before starting tasks. Each formatted record fits 64 bytes. */
bool RmLog_SetOutput(RmSerialId serial);
#endif
