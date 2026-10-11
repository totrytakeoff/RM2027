/**
 * @file dji_motor.h
 * @brief DJI智能电机头文件
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  1. 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
          2. 为M2006和M3508增加开环的零位校准函数,并在初始化时调用(根据用户配置决定是否调用)

 *
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdbool.h>

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define DJI_MOTOR_CNT 12

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度
    float speed_aps;          // 历史角速度接口，单位 deg/s
    float speed_rad_s;        // 转子角速度，单位 rad/s
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度 Celsius

    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} DJI_Motor_Measure_s;

/** One-shot PID runtime reset requests consumed by the motor task. */
typedef enum
{
    DJI_MOTOR_PID_RESET_NONE = 0U,
    DJI_MOTOR_PID_RESET_CURRENT = (1U << 0),
    DJI_MOTOR_PID_RESET_SPEED = (1U << 1),
    DJI_MOTOR_PID_RESET_ANGLE = (1U << 2),
    DJI_MOTOR_PID_RESET_ALL = DJI_MOTOR_PID_RESET_CURRENT |
                              DJI_MOTOR_PID_RESET_SPEED |
                              DJI_MOTOR_PID_RESET_ANGLE,
} DJIMotorPidReset_e;

/** Values embedded in a command instead of sampled through legacy pointers. */
typedef enum
{
    DJI_MOTOR_EXTERNAL_NONE = 0U,
    DJI_MOTOR_EXTERNAL_ANGLE = (1U << 0),
    DJI_MOTOR_EXTERNAL_SPEED = (1U << 1),
    DJI_MOTOR_EXTERNAL_SPEED_FF = (1U << 2),
    DJI_MOTOR_EXTERNAL_CURRENT_FF = (1U << 3),
} DJIMotorExternalInput_e;

/**
 * Complete task-to-task motor command.
 *
 * pid_reset_mask is an edge-triggered request: publishing it latches the bits
 * until the motor task consumes them, while reading the mailbox always returns
 * zero for that field.
 */
typedef struct
{
    Motor_Control_Setting_s settings;
    float reference;
    float other_angle_feedback;
    float other_speed_feedback;
    float speed_feedforward;
    float current_feedforward;
    Motor_Working_Type_e working_state;
    uint8_t external_input_mask;
    uint8_t pid_reset_mask;
} DJIMotorCommand;

/** Narrow, task-safe view of one PID runtime for diagnostics and tuning. */
typedef struct
{
    float reference;
    float measure;
    float error;
    float p_output;
    float i_output;
    float d_output;
    float output;
    float max_output;
} DJIMotorPidSnapshot;

/** Coherent view of the control chain most recently executed by the motor task. */
typedef struct
{
    Motor_Control_Setting_s settings;
    float command_reference;
    DJIMotorPidSnapshot angle;
    DJIMotorPidSnapshot speed;
    DJIMotorPidSnapshot current;
    float final_output;
    uint8_t output_active;
} DJIMotorControlSnapshot;

/** Generic diagnostic view shared by every DJI motor application. */
typedef struct
{
    DJI_Motor_Measure_s measure;
    DJIMotorControlSnapshot control;
} DJIMotorTuningSnapshot;

/**
 * @brief DJI intelligent motor typedef
 *
 */
typedef struct
{
    DJI_Motor_Measure_s measure;            // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器

    CANInstance *motor_can_instance; // 电机CAN实例
    // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;

    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志

    DaemonInstance* daemon;
    uint32_t feed_cnt;
    float dt;
    float last_total_angle;
    int8_t angle_feedback_sign;
    uint8_t angle_feedback_locked;
    uint8_t feedback_initialized;
    uint8_t output_active;
    float final_output;

    /* Published by application tasks and consumed only by DJIMotorControl(). */
    DJIMotorCommand command_mailbox;
    uint32_t command_last_publish_ms;
    uint32_t command_generation;
    uint8_t pending_pid_reset_mask;
    uint8_t command_published;

    /* Published only after one complete motor-control calculation. */
    DJIMotorControlSnapshot control_snapshot;
    uint8_t control_snapshot_valid;
} DJIMotorInstance;

/**
 * @brief 调用此函数注册一个DJI智能电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小DJIMotorControl()任务的运行频率.
 *
 * @attention M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return DJIMotorInstance*
 */
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);

/** Copy one coherent feedback snapshot for task-level consumers. */
bool DJIMotorGetMeasure(const DJIMotorInstance *motor,
                        DJI_Motor_Measure_s *measure);

/** Copy the latest persistent command (one-shot reset bits are returned clear). */
bool DJIMotorGetCommand(const DJIMotorInstance *motor,
                        DJIMotorCommand *command);

/** Copy the last PID runtime values without exposing a torn task-level read. */
bool DJIMotorGetControlSnapshot(const DJIMotorInstance *motor,
                                DJIMotorControlSnapshot *snapshot);

/** Copy feedback and the latest executed control chain as one diagnostic view. */
bool DJIMotorGetTuningSnapshot(const DJIMotorInstance *motor,
                               DJIMotorTuningSnapshot *snapshot);

/** Atomically publish one complete command for the motor task. */
bool DJIMotorPublishCommand(DJIMotorInstance *motor,
                            const DJIMotorCommand *command);

/**
 * Configure the maximum age of a command before motor output is forced to
 * zero. A value of zero disables the lease for legacy single-loop demos.
 */
bool DJIMotorSetCommandTimeout(uint32_t timeout_ms);

/** Return whether this motor has valid feedback inside its health deadline. */
bool DJIMotorIsOnline(const DJIMotorInstance *motor);

/** Return true only when every registered DJI motor is online. */
bool DJIMotorAllOnline(void);

/** Global safety gate checked by the motor task before every CAN transmission. */
void DJIMotorSetGlobalOutputEnabled(bool enabled);
bool DJIMotorGlobalOutputEnabled(void);

/**
 * @brief 被application层的应用调用,给电机设定参考值.
 *        对于应用,可以将电机视为传递函数为1的设备,不需要关心底层的闭环
 *
 * @param motor 要设置的电机
 * @param ref 设定参考值
 */
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);

/**
 * @brief 切换反馈的目标来源,如将角速度和角度的来源换为IMU(小陀螺模式常用)
 *
 * @param motor 要切换反馈数据来源的电机
 * @param loop  要切换反馈数据来源的控制闭环
 * @param type  目标反馈模式
 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void DJIMotorControl();

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 *
 */
void DJIMotorStop(DJIMotorInstance *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 *
 */
void DJIMotorEnable(DJIMotorInstance *motor);

/**
 * @brief 修改电机闭环目标(外层闭环)
 *
 * @param motor  要修改的电机实例指针
 * @param outer_loop 外层闭环类型
 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop);

#endif // !DJI_MOTOR_H
