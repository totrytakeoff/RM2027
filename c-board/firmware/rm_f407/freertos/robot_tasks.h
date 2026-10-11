#ifndef ROBOT_TASKS_H
#define ROBOT_TASKS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ROBOT_TASK_INS = 0,
    ROBOT_TASK_MOTOR,
    ROBOT_TASK_CONTROL,
    ROBOT_TASK_HEALTH,
    ROBOT_TASK_DIAGNOSTICS,
    ROBOT_TASK_TUNING_TELEMETRY,
    ROBOT_TASK_COUNT,
} RobotTaskId;

typedef struct {
    uint32_t period_us;
    uint32_t run_count;
    uint32_t deadline_miss_count;
    uint32_t consecutive_deadline_misses;
    uint32_t last_start_interval_us;
    uint32_t max_start_interval_us;
    uint32_t last_execution_us;
    uint32_t max_execution_us;
    uint32_t stack_high_water_words;
    uint32_t last_run_tick;
} RobotTaskRuntime;

typedef struct {
    RobotTaskRuntime tasks[ROBOT_TASK_COUNT];
    uint32_t unhealthy_mask;
} RobotTaskHealthSnapshot;

/** Create all application tasks using caller-owned static storage. */
bool RobotTasks_Create(void);

/** Copy the latest runtime counters for telemetry or debugger inspection. */
bool RobotTasks_GetHealthSnapshot(RobotTaskHealthSnapshot *snapshot);

#endif /* ROBOT_TASKS_H */
