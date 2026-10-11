#include "rm_fault.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"

enum {
    RM_EXCEPTION_FRAME_WORDS = 8U,
    RM_INVALID_REGISTER_VALUE = 0xFFFFFFFFUL,
};

volatile RmFaultRecord g_rm_fault_record
    __attribute__((section(".noinit.rm_fault"), used, aligned(4)));

static bool RmFault_IsFrameReadable(const uint32_t *stack_frame)
{
    uintptr_t first = (uintptr_t)stack_frame;
    uintptr_t last = first + (RM_EXCEPTION_FRAME_WORDS * sizeof(uint32_t));
    bool in_main_sram =
        (first >= 0x20000000UL) && (last <= 0x20020000UL) && (last > first);
    bool in_ccm_sram =
        (first >= 0x10000000UL) && (last <= 0x10010000UL) && (last > first);

    return ((first & (sizeof(uint32_t) - 1U)) == 0U) &&
           (in_main_sram || in_ccm_sram);
}

void RmFault_CaptureHardFault(uint32_t *stack_frame, uint32_t exc_return)
{
    volatile RmFaultRecord *record = &g_rm_fault_record;
    bool frame_valid;

    __disable_irq();
    record->magic = 0U;
    __DSB();

    frame_valid = RmFault_IsFrameReadable(stack_frame);
    record->version = RM_FAULT_RECORD_VERSION;
    record->active_vector = SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk;
    record->exc_return = exc_return;
    record->msp = __get_MSP();
    record->psp = __get_PSP();
    record->stack_frame_valid = frame_valid ? 1U : 0U;

    if (frame_valid) {
        record->r0 = stack_frame[0];
        record->r1 = stack_frame[1];
        record->r2 = stack_frame[2];
        record->r3 = stack_frame[3];
        record->r12 = stack_frame[4];
        record->lr = stack_frame[5];
        record->pc = stack_frame[6];
        record->xpsr = stack_frame[7];
    } else {
        record->r0 = RM_INVALID_REGISTER_VALUE;
        record->r1 = RM_INVALID_REGISTER_VALUE;
        record->r2 = RM_INVALID_REGISTER_VALUE;
        record->r3 = RM_INVALID_REGISTER_VALUE;
        record->r12 = RM_INVALID_REGISTER_VALUE;
        record->lr = RM_INVALID_REGISTER_VALUE;
        record->pc = RM_INVALID_REGISTER_VALUE;
        record->xpsr = RM_INVALID_REGISTER_VALUE;
    }

    record->cfsr = SCB->CFSR;
    record->hfsr = SCB->HFSR;
    record->dfsr = SCB->DFSR;
    record->afsr = SCB->AFSR;
    record->mmfar = SCB->MMFAR;
    record->bfar = SCB->BFAR;
    record->shcsr = SCB->SHCSR;
    record->icsr = SCB->ICSR;
    __DSB();
    record->magic = RM_FAULT_RECORD_MAGIC;
    __DSB();

    /* If IWDG is running it will reset the board; otherwise stay debuggable. */
    for (;;) {
        __NOP();
    }
}
