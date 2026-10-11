#ifndef RM_OUTPUT_GATE_H
#define RM_OUTPUT_GATE_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint32_t epoch, reasons, deadline_ms; bool permitted; } RmOutputGateSnapshot;
void RmOutputGate_Init(void);
void RmOutputGate_SetStopHook(void (*cancel_motion)(void));
void RmOutputGate_Stop(uint32_t reasons);
/* Only the application rearm state machine may issue/renew a permit. */
uint32_t RmOutputGate_Arm(uint32_t now_ms, uint32_t lease_ms);
bool RmOutputGate_Renew(uint32_t epoch, uint32_t now_ms, uint32_t lease_ms);
bool RmOutputGate_Allows(uint32_t epoch, uint32_t now_ms);
void RmOutputGate_Read(RmOutputGateSnapshot *snapshot);
#endif
