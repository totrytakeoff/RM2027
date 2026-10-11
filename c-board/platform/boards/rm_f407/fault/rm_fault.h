#ifndef RM_FAULT_H
#define RM_FAULT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RM_FAULT_RECORD_MAGIC   0x524D4641UL
#define RM_FAULT_RECORD_VERSION 1UL

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t active_vector;
    uint32_t exc_return;
    uint32_t msp;
    uint32_t psp;
    uint32_t stack_frame_valid;
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t afsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t shcsr;
    uint32_t icsr;
} RmFaultRecord;

/* Retained in .noinit so a debugger can inspect the last crash after reset. */
extern volatile RmFaultRecord g_rm_fault_record;

void RmFault_CaptureHardFault(uint32_t *stack_frame,
                              uint32_t exc_return) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif
