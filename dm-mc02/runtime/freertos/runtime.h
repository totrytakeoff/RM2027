#ifndef RM_RUNTIME_H
#define RM_RUNTIME_H
#include "rm_platform.h"
typedef enum { RM_TASK_INS,RM_TASK_IO,RM_TASK_MOTOR,RM_TASK_HEALTH,RM_TASK_CONTROL,RM_TASK_DIAGNOSTICS,RM_TASK_COUNT } RmTaskId;
typedef struct {
    uint32_t run_count, last_finish_ms, max_runtime_us, deadline_misses;
    uint32_t consecutive_overruns, stack_free_words;
} RmTaskStats;
typedef struct {
    void (*tick[RM_TASK_COUNT])(uint64_t now_us);
    bool (*bootstrap_complete)(void);
    void (*fault)(uint32_t reasons);
} RmRuntimeConfig;
_Noreturn void RmRuntime_Start(const RmRuntimeConfig *config);
bool RmRuntime_ReadTask(RmTaskId id,RmTaskStats *stats);
bool RmRuntime_Healthy(uint32_t now_ms);
#endif
