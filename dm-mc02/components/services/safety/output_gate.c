#include "output_gate.h"
#include "rm_platform.h"
static RmOutputGateSnapshot gate;
static void (*stop_hook)(void);
void RmOutputGate_Init(void)
{
    RmCriticalState key = RmCritical_Enter();
    gate = (RmOutputGateSnapshot){.epoch=1U, .reasons=1U};
    stop_hook=NULL;
    RmCritical_Exit(key);
}
void RmOutputGate_SetStopHook(void (*cancel_motion)(void))
{
    RmCriticalState key=RmCritical_Enter();
    if (!gate.permitted) stop_hook=cancel_motion;
    RmCritical_Exit(key);
}
void RmOutputGate_Stop(uint32_t reasons)
{
    RmCriticalState key = RmCritical_Enter();
    bool cancel=gate.permitted;
    if (cancel) { if (++gate.epoch == 0U) ++gate.epoch; }
    gate.permitted = false; gate.reasons |= reasons;
    if (cancel && stop_hook) stop_hook(); /* Register-only cancellation; no waits. */
    RmCritical_Exit(key);
}
uint32_t RmOutputGate_Arm(uint32_t now_ms, uint32_t lease_ms)
{
    if (lease_ms == 0U || lease_ms >= INT32_MAX) return 0U;
    RmCriticalState key = RmCritical_Enter();
    if (stop_hook) stop_hook(); /* Discard earlier epochs before issuing a new one. */
    if (++gate.epoch == 0U) ++gate.epoch;
    gate.reasons = 0U; gate.deadline_ms = now_ms + lease_ms; gate.permitted = true;
    uint32_t epoch = gate.epoch;
    RmCritical_Exit(key);
    return epoch;
}
bool RmOutputGate_Allows(uint32_t epoch, uint32_t now_ms)
{
    RmCriticalState key = RmCritical_Enter();
    bool ok = gate.permitted && epoch != 0U && epoch == gate.epoch &&
              (int32_t)(gate.deadline_ms - now_ms) > 0;
    RmCritical_Exit(key);
    return ok;
}
bool RmOutputGate_Renew(uint32_t epoch, uint32_t now_ms, uint32_t lease_ms)
{
    if (lease_ms == 0U || lease_ms >= INT32_MAX) return false;
    RmCriticalState key = RmCritical_Enter();
    bool ok = RmOutputGate_Allows(epoch, now_ms);
    if (ok) gate.deadline_ms = now_ms + lease_ms;
    RmCritical_Exit(key);
    return ok;
}
void RmOutputGate_Read(RmOutputGateSnapshot *snapshot)
{
    if (!snapshot) return;
    RmCriticalState key = RmCritical_Enter(); *snapshot = gate; RmCritical_Exit(key);
}
