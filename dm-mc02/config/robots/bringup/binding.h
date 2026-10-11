#ifndef RM_BRINGUP_BINDING_H
#define RM_BRINGUP_BINDING_H
#include "ins.h"
#include "remote.h"
#include "dji.h"
typedef struct {
    RmInsConfig ins;
    RmRemoteConfig remote;
    const RmDjiConfig *motors;
    unsigned motor_count;
} RmBringupBinding;
extern const RmBringupBinding rm_bringup_binding;
#endif
