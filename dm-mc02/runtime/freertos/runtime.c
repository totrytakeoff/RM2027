#include "runtime.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bringup.h"
#include <string.h>
/* DTCM stacks cannot accidentally be passed as DMA storage. Sizes are initial
   budgets; retain high-water marks for hardware tuning. */
#define TASK_STACK __attribute__((section(".dtcm_bss"),aligned(8)))
static StackType_t ins_stack[1024] TASK_STACK, io_stack[768] TASK_STACK;
static StackType_t motor_stack[512] TASK_STACK, health_stack[512] TASK_STACK;
static StackType_t control_stack[768] TASK_STACK, diagnostics_stack[768] TASK_STACK;
static StaticTask_t tcbs[RM_TASK_COUNT];
static TaskHandle_t handles[RM_TASK_COUNT];
static RmTaskStats stats[RM_TASK_COUNT];
static uint32_t last_start[RM_TASK_COUNT];
static const uint32_t periods[]={RM_INS_PERIOD_MS,RM_IO_PERIOD_MS,RM_MOTOR_PERIOD_MS,
    RM_HEALTH_PERIOD_MS,RM_CONTROL_PERIOD_MS,RM_DIAGNOSTICS_PERIOD_MS};
static RmRuntimeConfig callbacks;
static bool watchdog_started, startup_complete;
static uint32_t runtime_start_ms;
static void run(void *argument)
{
    unsigned id=(unsigned)(uintptr_t)argument;
    TickType_t wake=xTaskGetTickCount();
    for (;;) {
        uint64_t now_us=RmTime_NowUs();
        uint32_t begin=RmTime_Cycles();
        uint32_t start_us=(uint32_t)now_us;
        bool release_late=stats[id].run_count>0U && start_us-last_start[id]>periods[id]*1250U;
        last_start[id]=start_us;
        if (callbacks.tick[id]) callbacks.tick[id](now_us);
        uint32_t elapsed=RmTime_CyclesToUs(RmTime_Cycles()-begin);
        RmCriticalState key=RmCritical_Enter();
        RmTaskStats *s=&stats[id];
        ++s->run_count; s->last_finish_ms=RmTime_NowMs();
        if (elapsed>s->max_runtime_us) s->max_runtime_us=elapsed;
        if (release_late || elapsed>=periods[id]*1000U) {
            ++s->deadline_misses; ++s->consecutive_overruns;
        } else s->consecutive_overruns=0U;
        RmCritical_Exit(key);
        if (s->run_count%100U==0U) {
            uint32_t free_words=uxTaskGetStackHighWaterMark(NULL);
            key=RmCritical_Enter(); s->stack_free_words=free_words; RmCritical_Exit(key);
        }
        if (id==RM_TASK_HEALTH) {
            if (!startup_complete && callbacks.bootstrap_complete && callbacks.bootstrap_complete()) startup_complete=true;
            bool boot_within_deadline=startup_complete || RmTime_NowMs()-runtime_start_ms<RM_BOOTSTRAP_TIMEOUT_MS;
            if (RmRuntime_Healthy(RmTime_NowMs()) && boot_within_deadline) {
                if (watchdog_started) RmWatchdog_Refresh();
            } else callbacks.fault(1U<<11);
        }
        /* Absolute periodic release; catch-up work remains bounded by the callback. */
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(periods[id]));
    }
}
bool RmRuntime_ReadTask(RmTaskId id,RmTaskStats *out)
{
    if ((unsigned)id>=RM_TASK_COUNT || !out) return false;
    RmCriticalState key=RmCritical_Enter(); *out=stats[id]; RmCritical_Exit(key); return true;
}
bool RmRuntime_Healthy(uint32_t now_ms)
{
    RmCriticalState key=RmCritical_Enter();
    bool ok=true;
    if (now_ms-runtime_start_ms>RM_TASK_STARTUP_GRACE_MS) for (unsigned i=0;i<RM_TASK_DIAGNOSTICS;++i) {
        const RmTaskStats *s=&stats[i];
        if (s->run_count==0U || now_ms-s->last_finish_ms>periods[i]*RM_TASK_HEARTBEAT_PERIODS ||
            s->consecutive_overruns>=3U || s->stack_free_words<RM_TASK_STACK_MARGIN_WORDS) ok=false;
    }
    RmCritical_Exit(key); return ok;
}
_Noreturn void RmRuntime_Start(const RmRuntimeConfig *config)
{
    if (!config || !config->bootstrap_complete || !config->fault) RmFault_Panic(0x3001U);
    for (unsigned i=0;i<RM_TASK_COUNT;++i) if (!config->tick[i]) RmFault_Panic(0x3002U);
    callbacks=*config; runtime_start_ms=RmTime_NowMs();
    /* Reset assembly has already supervised board/assembly initialization.
       Switch to the runtime timeout; health owns bounded-bootstrap refresh. */
    watchdog_started=RmWatchdog_Start();
    if (!watchdog_started) RmFault_Panic(0x300CU);
    StackType_t *stacks[]={ins_stack,io_stack,motor_stack,health_stack,control_stack,diagnostics_stack};
    const uint32_t sizes[]={1024,768,512,512,768,768};
    const UBaseType_t priorities[]={5,4,3,4,2,1};
    const char *names[]={"ins","io","motor","health","control","diagnostics"};
    for (unsigned i=0;i<RM_TASK_COUNT;++i) {
        stats[i].stack_free_words=sizes[i];
        handles[i]=xTaskCreateStatic(run,names[i],sizes[i],(void *)(uintptr_t)i,priorities[i],stacks[i],&tcbs[i]);
        if (!handles[i]) RmFault_Panic(0x3003U+i);
    }
    vTaskStartScheduler();
    RmFault_Panic(0x300AU);
}
