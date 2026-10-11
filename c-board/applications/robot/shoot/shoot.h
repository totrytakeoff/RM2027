/**
 * @file shoot.h
 * @brief 发射控制模块
 */

#ifndef ROBOT_SHOOT_H
#define ROBOT_SHOOT_H

#include <stdbool.h>

#include "dji_motor.h"
#include "robot_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * 发射状态枚举
 *============================================================================*/
typedef enum {
    SHOOT_OFF = 0,          // 关闭
    SHOOT_FRICTION_ON,      // 仅摩擦轮
    SHOOT_SINGLE,           // 单发
    SHOOT_CONTINUOUS,       // 连发
} ShootState_e;

typedef enum {
    SHOOT_MOTOR_LOADER = 0,
    SHOOT_MOTOR_FRICTION_LEFT,
    SHOOT_MOTOR_FRICTION_RIGHT,
} ShootMotor_e;

typedef enum {
    SHOOT_LOADER_JAM_IDLE = 0,
    SHOOT_LOADER_JAM_REVERSING,
    SHOOT_LOADER_JAM_LOCKED,
} ShootLoaderJamState_e;

typedef struct {
    RobotFireMode_e input_fire_mode;
    uint8_t fire_trigger_down;
    uint8_t fire_trigger_pressed;
    uint8_t single_trigger_consumed;
    uint32_t single_trigger_activation_count;
    ShootState_e shoot_state;
    uint8_t friction_ready;
    uint8_t single_active;
    uint8_t pending_shots;
    uint32_t single_start_count;
    uint32_t single_timeout_count;
    ShootLoaderJamState_e loader_jam_state;
    uint8_t loader_jam_retry_count;
    uint32_t loader_jam_fault_count;
} ShootTuningSnapshot;

/**
 * @brief 发射初始化
 */
bool Shoot_Init(void);

/**
 * @brief 发射更新
 * @param input 输入数据
 */
void Shoot_Update(Input_Data_t *input);

/**
 * @brief 发射停止
 */
void Shoot_Stop(void);
bool Shoot_IsHealthy(void);
ShootState_e Shoot_GetState(void);
float Shoot_GetLoaderRef(void);
float Shoot_GetLoaderFeedback(void);
uint8_t Shoot_IsSingleActive(void);
uint8_t Shoot_GetPendingShots(void);
bool Shoot_GetTuningSnapshot(ShootTuningSnapshot *snapshot);
bool Shoot_GetMotorTuningSnapshot(ShootMotor_e selection,
                                  DJIMotorTuningSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_SHOOT_H */
