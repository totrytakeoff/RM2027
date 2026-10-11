#ifndef RM_CRITICAL_H
#define RM_CRITICAL_H

#include <stdint.h>

#include "stm32f4xx.h"

typedef uint32_t RmCriticalState;

/** Enter a short single-core critical section from task or interrupt context. */
static inline RmCriticalState RmCritical_Enter(void)
{
    RmCriticalState state = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return state;
}

/** Restore the interrupt mask captured by RmCritical_Enter(). */
static inline void RmCritical_Exit(RmCriticalState state)
{
    __DMB();
    __set_PRIMASK(state);
}

#endif /* RM_CRITICAL_H */
