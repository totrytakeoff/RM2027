#ifndef RM_CHASSIS_APP_H
#define RM_CHASSIS_APP_H
#include "runtime.h"
#include "chassis_control.h"
#include "dji.h"
typedef struct {
    uint64_t timestamp_us;
    RmChassisOutput output;
    RemoteControlState remote;
    RmDjiFeedback feedback[4];
    uint32_t reasons,online_mask,publish_failures;
    bool enabled;
} RmChassisSnapshot;
const RmRuntimeConfig *RmChassisApp_Init(void);
bool RmChassisApp_Read(RmChassisSnapshot *);
#endif
