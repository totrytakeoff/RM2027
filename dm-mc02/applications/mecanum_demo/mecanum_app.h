#ifndef RM_MECANUM_APP_H
#define RM_MECANUM_APP_H
#include "runtime.h"
#include "mecanum_control.h"
#include "dji.h"
typedef struct {
    uint64_t timestamp_us;
    RmMecanumOutput output;
    RemoteControlState remote;
    RmDjiFeedback feedback[4];
    uint32_t reasons,online_mask,publish_failures;
    bool enabled;
} RmMecanumSnapshot;
const RmRuntimeConfig *RmMecanumApp_Init(void);
bool RmMecanumApp_Read(RmMecanumSnapshot *);
#endif
