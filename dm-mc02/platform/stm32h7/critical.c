#include "h7_private.h"
RmCriticalState RmCritical_Enter(void)
{
    uint32_t prior = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return prior;
}
void RmCritical_Exit(RmCriticalState prior)
{
    __DMB();
    __set_PRIMASK(prior);
}
