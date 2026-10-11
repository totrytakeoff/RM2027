#include "shoot_trigger.h"

#include <stddef.h>

void SingleShotTrigger_Init(SingleShotTrigger *trigger)
{
    if (trigger == NULL) {
        return;
    }
    trigger->consumed = 0U;
    trigger->activation_count = 0U;
}

bool SingleShotTrigger_Update(SingleShotTrigger *trigger,
                                     bool single_mode,
                                     bool trigger_down)
{
    if (trigger == NULL) {
        return false;
    }

    if (!single_mode || !trigger_down) {
        trigger->consumed = 0U;
        return false;
    }
    if (trigger->consumed != 0U) {
        return false;
    }

    trigger->consumed = 1U;
    if (trigger->activation_count < UINT32_MAX) {
        trigger->activation_count++;
    }
    return true;
}
