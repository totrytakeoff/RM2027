#ifndef RM_COMPONENTS_DM_MOTOR_H
#define RM_COMPONENTS_DM_MOTOR_H

#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DM 电机通用控制模式（与官方协议一致）
 * @note 这些值会直接参与 CAN 标准帧 ID 组帧：StdId = motor_id + mode
 */
typedef enum
{
    DM_MODE_MIT = 0x000,
    DM_MODE_POSITION = 0x100,
    DM_MODE_SPEED = 0x200,
    DM_MODE_MIXED = 0x300,
} DMMotor_Mode;

/**
 * @brief DM 电机通用初始化配置
 *
 * @note
 * - motor_id：电机拨码设定的 CAN ID（用于发送控制帧/使能帧）
 * - master_id：上位机显示的 Master ID（4bit），或“完整反馈 StdId”
 *   - 若 use_shared_feedback_id = false 且 master_id <= 0x0F：反馈 StdId 计算为 (master<<4)|(motor&0x0F)
 *   - 若 use_shared_feedback_id = true：反馈 StdId 使用 master_id（多电机共用同一反馈 ID）
 *   - 若 master_id > 0x0F：视为你已给出完整反馈 StdId（用于适配框架严格 CAN 过滤器）
 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint16_t motor_id;
    uint16_t master_id;

    float position_range; /* PMAX */
    float velocity_range; /* VMAX */
    float torque_range;   /* TMAX */
    float kp_max;
    float kd_max;

    bool auto_clear_error;
    bool auto_enable_mit;
    bool auto_zero_position;
    bool use_shared_feedback_id;
} DMMotor_InitConfig;

typedef struct
{
    uint8_t motor_id;
    uint8_t error_state;
    float position_rad;
    float velocity_rad_s;
    float torque;
    float mos_temp;
    float rotor_temp;
} DMMotor_Feedback;

typedef struct DMMotor_Handle DMMotor_Handle;

/* -------------------- 工具函数：角度/弧度互转 -------------------- */
static inline float DM_DegToRad(float deg)
{
    return deg * 0.01745329251994329576923690768489f;
}
static inline float DM_RadToDeg(float rad)
{
    return rad * 57.295779513082320876798154814105f;
}

/* -------------------- 生命周期 -------------------- */
DMMotor_Handle *DMMotor_Init(const DMMotor_InitConfig *config);
void DMMotor_DeInit(DMMotor_Handle *motor);

/* -------------------- 反馈 -------------------- */
const DMMotor_Feedback *DMMotor_GetFeedback(const DMMotor_Handle *motor);

/* -------------------- 模式控制/指令 -------------------- */
void DMMotor_ClearError(DMMotor_Handle *motor, DMMotor_Mode mode);
void DMMotor_Enable(DMMotor_Handle *motor, DMMotor_Mode mode);
void DMMotor_Disable(DMMotor_Handle *motor, DMMotor_Mode mode);
void DMMotor_SaveZero(DMMotor_Handle *motor, DMMotor_Mode mode);

/**
 * @brief 发送 MIT 标准控制帧（p, v, kp, kd, t）
 * @note 这是“协议直通接口”，不包含任何控制策略/环路；调参时外部 1kHz 调用即可。
 */
void DMMotor_SendMIT(DMMotor_Handle *motor,
                     float position_rad,
                     float velocity_rad_s,
                     float kp,
                     float kd,
                     float torque);

void DMMotor_SendPosition(DMMotor_Handle *motor, float position_rad, float max_speed_rad_s);
void DMMotor_SendSpeed(DMMotor_Handle *motor, float speed_rad_s);
void DMMotor_SendMixed(DMMotor_Handle *motor, float position_rad, float velocity_rad_s, float current);

/* -------------------- 寄存器（上位机同款 0x7FF 通道） -------------------- */
void DMMotor_RequestRegister(DMMotor_Handle *motor, uint8_t reg);
void DMMotor_WriteRegister(DMMotor_Handle *motor, uint8_t reg, const uint8_t value[4]);
void DMMotor_SaveRegisters(DMMotor_Handle *motor);

#ifdef __cplusplus
}
#endif

#endif
