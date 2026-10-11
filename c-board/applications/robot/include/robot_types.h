/**
 * @file robot_types.h
 * @brief 最小框架类型定义
 */

#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>
#include "referee_types.h"

/*============================================================================
 * 控制模式枚举
 *============================================================================*/
typedef enum {
    MODE_ZERO_FORCE = 0,    // 停止
    MODE_NORMAL,            // 正常工作
} WorkMode_e;

typedef enum {
    CHASSIS_FOLLOW = 0,     // 云台跟随底盘
    CHASSIS_NO_FOLLOW,      // 底盘独立
} ChassisMode_e;

typedef enum {
    ROBOT_CONTROL_FOLLOW = 0,
    ROBOT_CONTROL_AUTO_AIM,
    ROBOT_CONTROL_SPIN,
} RobotControlMode_e;

typedef enum {
    FRICTION_OFF = 0,
    FRICTION_ON,
} FrictionMode_e;

typedef enum {
    LOADER_STOP = 0,
    LOADER_SINGLE,
    LOADER_DOUBLE,
    LOADER_CONTINUOUS,
} LoaderMode_e;

typedef enum {
    ROBOT_FIRE_DISABLED = 0,
    ROBOT_FIRE_SINGLE,
    ROBOT_FIRE_CONTINUOUS,
} RobotFireMode_e;

typedef enum {
    CTRL_ZERO_FORCE = 0,
    CTRL_ENABLE,
} ControlMode_e;

typedef enum {
    REF_SPEED = 0,
    REF_ANGLE,
} RefType_e;

typedef enum {
    AXIS_CTRL_ANGLE = 0,
    AXIS_CTRL_SPEED,
    AXIS_CTRL_BRAKE,
} AxisCtrlMode_e;

/*============================================================================
 * 机器人操作意图。这里只描述期望动作，不包含任何具体遥控协议字段。
 *============================================================================*/
typedef struct {
    /*
     * 无量纲操作意图，范围统一为 [-1, 1]。输入层不得在这里引入
     * m/s、rad/s、deg/s、电机转速或机械限位。
     */
    float chassis_x_intent;       // 云台坐标系横移：左为正
    float chassis_y_intent;       // 云台坐标系前进：前为正
    float chassis_rotate_intent;  // 底盘旋转意图
    float gimbal_yaw_intent;      // 云台 Yaw 速度意图
    float gimbal_pitch_intent;    // 云台 Pitch 速度意图
    uint8_t yaw_control_active;
    uint8_t pitch_control_active;
    
    /* 发射操作意图；实际电机状态由发射执行层维护。 */
    RobotFireMode_e fire_mode;
    uint8_t fire_trigger_down;
    uint8_t fire_trigger_pressed;
    uint8_t fire_trigger_released;
    
    /* 模式控制 */
    RobotControlMode_e control_mode;
    uint8_t emergency_stop;     // 急停

    /* 遥控链路与操作者安全门请求，由安全层决定最终是否允许输出。 */
    uint8_t online;
    uint8_t data_valid;
    uint8_t operator_enable_request;
    uint8_t operator_safe_position; /* 安全门以外的控制均处于安全姿态 */
    uint8_t operator_arm_event;
} Input_Data_t;

/*============================================================================
 * 底盘控制数据
 *============================================================================*/
typedef struct {
    float vx;               // 车体坐标系横移速度 (m/s)
    float vy;               // 车体坐标系前进速度 (m/s)
    float wz;               // 车体旋转角速度 (rad/s)
    float vx_cmd;           // 云台坐标系横移速度指令
    float vy_cmd;           // 云台坐标系前进速度指令
    float yaw_offset_rad;   // 云台相对底盘逻辑夹角(rad)
    ChassisMode_e mode;
    uint8_t spin_enable;
    ControlMode_e control_mode;
    RefType_e ref_type;
} Chassis_Cmd_t;

/*============================================================================
 * 云台控制数据
 *============================================================================*/
typedef struct {
    float yaw_speed;        // Yaw角速度指令
    float pitch_speed;      // Pitch角度目标推进速率
    float yaw_angle;        // Yaw目标角度(分离模式)
    float pitch_angle;      // Pitch目标角度
    RobotControlMode_e mode;
    AxisCtrlMode_e yaw_ctrl_mode;
    uint8_t manual_pitch;   // Pitch操作者输入是否活动
    AxisCtrlMode_e pitch_ctrl_mode;
    ControlMode_e control_mode;
    RefType_e ref_type;
} Gimbal_Cmd_t;

/*============================================================================
 * 发射控制数据
 *============================================================================*/
typedef struct {
    FrictionMode_e friction;
    LoaderMode_e loader;
    ControlMode_e control_mode;
    RefType_e ref_type;
} Shoot_Cmd_t;

/*============================================================================
 * 全局控制上下文 - 所有模块共享
 *============================================================================*/
typedef struct {
    Input_Data_t input;
    Chassis_Cmd_t chassis;
    Gimbal_Cmd_t gimbal;
    Shoot_Cmd_t shoot;
    
    uint8_t emergency_stop; // 紧急停止标志
    uint8_t initialized;    // 初始化完成标志
    MinimalRefereeData_t referee;
} Robot_Context_t;

/* 全局上下文声明 */
#ifdef __cplusplus
extern "C" {
#endif

extern Robot_Context_t g_robot;

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_TYPES_H */
