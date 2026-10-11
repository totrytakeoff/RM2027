#include "FreeRTOS.h"
#include "task.h"
#include "rm_platform.h"
static StaticTask_t idle_tcb;
static StackType_t idle_stack[configMINIMAL_STACK_SIZE] __attribute__((section(".dtcm_bss"),aligned(8)));
void vApplicationGetIdleTaskMemory(StaticTask_t **tcb,StackType_t **stack,uint32_t *size)
{
    *tcb=&idle_tcb; *stack=idle_stack; *size=configMINIMAL_STACK_SIZE;
}
void vApplicationStackOverflowHook(TaskHandle_t task,char *name)
{
    (void)task; (void)name; RmFault_Panic(0x300BU);
}
