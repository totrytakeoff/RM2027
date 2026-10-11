#ifndef RM_COMPONENTS_DM8009P_H
#define RM_COMPONENTS_DM8009P_H

/**
 * @file dm8009p.h
 * @brief 兼容层：DM8009P（DM‑J8009P‑2EC）默认参数/便捷封装
 *
 * 说明：
 * - MIT 协议/模式定义/打包发送等“通用能力”已经上移到 `dmmotor.h/.c`。
 * - 本文件仅保留“型号相关的默认值”，并提供薄封装，便于旧代码过渡。
 */

#include "dmmotor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DM8009P 常用默认量程（与上位机截图一致） */
#define DM8009P_DEFAULT_P_RANGE 12.5f
#define DM8009P_DEFAULT_V_RANGE 45.0f
#define DM8009P_DEFAULT_T_RANGE 54.0f
#define DM8009P_DEFAULT_KP_MAX 500.0f
#define DM8009P_DEFAULT_KD_MAX 5.0f

/* 兼容旧类型名 */
typedef DMMotor_Mode DM8009P_Mode;
typedef DMMotor_InitConfig DM8009P_InitConfig;
typedef DMMotor_Feedback DM8009P_Feedback;
typedef DMMotor_Handle DM8009P_Handle;

static inline void DM8009P_FillDefaultConfig(DM8009P_InitConfig *cfg)
{
    if (cfg == NULL)
        return;
    cfg->position_range = DM8009P_DEFAULT_P_RANGE;
    cfg->velocity_range = DM8009P_DEFAULT_V_RANGE;
    cfg->torque_range = DM8009P_DEFAULT_T_RANGE;
    cfg->kp_max = DM8009P_DEFAULT_KP_MAX;
    cfg->kd_max = DM8009P_DEFAULT_KD_MAX;
}

/* 兼容旧 API：实际调用通用 DMMotor_* */
static inline DM8009P_Handle *DM8009P_Init(const DM8009P_InitConfig *config) { return DMMotor_Init(config); }
static inline void DM8009P_DeInit(DM8009P_Handle *motor) { DMMotor_DeInit(motor); }
static inline const DM8009P_Feedback *DM8009P_GetFeedback(const DM8009P_Handle *motor) { return DMMotor_GetFeedback(motor); }
static inline void DM8009P_ClearError(DM8009P_Handle *motor, DM8009P_Mode mode) { DMMotor_ClearError(motor, mode); }
static inline void DM8009P_Enable(DM8009P_Handle *motor, DM8009P_Mode mode) { DMMotor_Enable(motor, mode); }
static inline void DM8009P_Disable(DM8009P_Handle *motor, DM8009P_Mode mode) { DMMotor_Disable(motor, mode); }
static inline void DM8009P_SaveZero(DM8009P_Handle *motor, DM8009P_Mode mode) { DMMotor_SaveZero(motor, mode); }
static inline void DM8009P_SendMITCommand(DM8009P_Handle *motor, float p, float v, float kp, float kd, float t)
{
    DMMotor_SendMIT(motor, p, v, kp, kd, t);
}
static inline void DM8009P_SendSpeedCommand(DM8009P_Handle *motor, float speed_rad_s) { DMMotor_SendSpeed(motor, speed_rad_s); }
static inline void DM8009P_SendPositionCommand(DM8009P_Handle *motor, float position_rad, float max_speed_rad_s)
{
    DMMotor_SendPosition(motor, position_rad, max_speed_rad_s);
}
static inline void DM8009P_SendMixedCommand(DM8009P_Handle *motor, float position_rad, float velocity_rad_s, float current)
{
    DMMotor_SendMixed(motor, position_rad, velocity_rad_s, current);
}
static inline void DM8009P_RequestRegister(DM8009P_Handle *motor, uint8_t reg) { DMMotor_RequestRegister(motor, reg); }
static inline void DM8009P_WriteRegister(DM8009P_Handle *motor, uint8_t reg, const uint8_t value[4])
{
    DMMotor_WriteRegister(motor, reg, value);
}
static inline void DM8009P_SaveRegisters(DM8009P_Handle *motor) { DMMotor_SaveRegisters(motor); }

#ifdef __cplusplus
}
#endif

#endif
