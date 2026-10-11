#include "h7_private.h"
#define FAULT_MAGIC 0x524D4641U
typedef struct {
    uint32_t magic, version, reason, ipsr, cfsr, hfsr, mmfar, bfar;
    uint32_t frame[8], checksum;
} RmFaultRecord;
volatile RmFaultRecord rm_fault_record __attribute__((section(".noinit_fault"), aligned(32)));
static uint32_t checksum(const RmFaultRecord *r)
{
    uint32_t sum=0xC1A0F00DU ^ r->version ^ r->reason ^ r->ipsr ^ r->cfsr ^
        r->hfsr ^ r->mmfar ^ r->bfar;
    for (unsigned i=0;i<8U;++i) sum^=r->frame[i];
    return sum;
}
bool RmFault_ReadPrevious(RmFaultInfo *out)
{
    if (!out) return false;
    RmCriticalState key=RmCritical_Enter();
    RmFaultRecord previous=rm_fault_record;
    rm_fault_record.magic=0U;
    RmCritical_Exit(key);
    if (previous.magic!=FAULT_MAGIC || previous.version!=1U ||
        previous.checksum!=checksum(&previous)) return false;
    *out=(RmFaultInfo){.reason=previous.reason,.ipsr=previous.ipsr,
        .cfsr=previous.cfsr,.hfsr=previous.hfsr,.mmfar=previous.mmfar,.bfar=previous.bfar};
    for (unsigned i=0;i<8U;++i) out->frame[i]=previous.frame[i];
    return true;
}
static void stop_outputs(void)
{
    /* No HAL handles, initialized RAM, polling or new transmissions are needed.
       Abort outstanding requests and inhibit retransmission before waiting for
       reset. A frame already on the wire cannot be withdrawn. */
    if ((RCC->APB1HENR & RCC_APB1HENR_FDCANEN)!=0U) {
        FDCAN_GlobalTypeDef *const controllers[]={FDCAN1,FDCAN2,FDCAN3};
        for (unsigned i=0;i<3U;++i) {
            controllers[i]->IE=0U; controllers[i]->ILE=0U;
            controllers[i]->TXBCR=controllers[i]->TXBRP;
            controllers[i]->CCCR |= FDCAN_CCCR_DAR | FDCAN_CCCR_INIT;
        }
        __DSB();
    }
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIOB->BSRR = GPIO_PIN_1 << 16U;
    GPIOC->BSRR = (GPIO_PIN_13 | GPIO_PIN_14) << 16U;
}
static _Noreturn void record(uint32_t reason, const uint32_t *frame)
{
    __disable_irq(); stop_outputs();
    rm_fault_record.magic = 0U;
    rm_fault_record.version = 1U; rm_fault_record.reason = reason;
    rm_fault_record.ipsr = __get_IPSR(); rm_fault_record.cfsr = SCB->CFSR;
    rm_fault_record.hfsr = SCB->HFSR; rm_fault_record.mmfar = SCB->MMFAR;
    rm_fault_record.bfar = SCB->BFAR;
    uintptr_t p = (uintptr_t)frame;
    bool readable = (p >= 0x20000000U && p <= 0x20020000U - 32U) ||
                    (p >= 0x24000000U && p <= 0x24050000U - 32U);
    if ((SCB->CFSR & (SCB_CFSR_STKERR_Msk | SCB_CFSR_MSTKERR_Msk)) != 0U) readable = false;
    for (unsigned i=0; i<8U; ++i) rm_fault_record.frame[i] = readable ? frame[i] : 0U;
    RmFaultRecord current=rm_fault_record;
    rm_fault_record.checksum = checksum(&current);
    __DSB(); rm_fault_record.magic = FAULT_MAGIC; __DSB();
    for (;;) __WFI(); /* The watchdog, if running, resets; debugger can inspect retained record. */
}
_Noreturn void RmFault_Panic(uint32_t reason) { record(reason, NULL); }
_Noreturn void RmFault_FromException(uint32_t *frame, uint32_t exc_return)
{
    /* ARM DUI 0646C, Figure 2-2: r0..xPSR start at SP for both frame types;
       floating-point storage follows the eight core-register words. */
    (void)exc_return;
    record(0xF000U | __get_IPSR(), frame);
}
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile("tst lr, #4\n ite eq\n mrseq r0, msp\n mrsne r0, psp\n mov r1, lr\n b RmFault_FromException");
}
void MemManage_Handler(void) { RmFault_Panic(0xF004U); }
void BusFault_Handler(void) { RmFault_Panic(0xF005U); }
void UsageFault_Handler(void) { RmFault_Panic(0xF006U); }
void NMI_Handler(void) { RmFault_Panic(0xF002U); }
