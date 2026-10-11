/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 十字全向轮底盘遥控控制演示
 ******************************************************************************
 * @attention
 *
 * 本测试固件整合了遥控接收和电机控制，实现十字全向轮底盘的遥控控制。
 * 遥控器左摇杆控制底盘前进/后退和左转/右转，右摇杆控制底盘左平移/右平移。
 *
 * 电机布局（俯视）：
 *       2
 *       |
 *   1---+---3
 *       |
 *       4
 *
 * 1：右前轮，2：左前轮，3：右后轮，4：左后轮
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "cmsis_os.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "bsp_init.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dt7_remote.h"
#include "dji_motor.h"
#include "user_lib.h"

/* Private define ------------------------------------------------------------*/
#define CHASSIS_MOTOR_COUNT 4U
#define CHASSIS_UPDATE_INTERVAL_MS 20U  // 50Hz更新频率
#define CHASSIS_WHEEL_RADIUS 0.075f    // 轮子半径(m)
#define CHASSIS_WHEEL_BASE 0.34f        // 轮距(m)  轴距 25cm, 轮距 34cm
#define CHASSIS_MAX_VEL 20.0f          // 提高最大速度(m/s)
#define CHASSIS_MAX_ROTATE 47.0f       // 提高最大旋转速度(rad/s)
#define CHASSIS_BOOST_VEL 30.0f        // 提高冲刺速度(m/s)
#define CHASSIS_BRAKE_FACTOR 0.2f      // 刹车因子，0表示完全停止，1表示不刹车

#define M3508_SPEED_MAX 30000.0f    // deg/s, ~83.3 rps
#define M3508_SPEED_MIN (-M3508_SPEED_MAX)

// 速度倍增因子，用于提高整体响应速度
#define CHASSIS_SPEED_MULTIPLIER 1.5f    // 提高速度响应的倍数

// 死区阈值，防止微小抖动 - 使用不同大小的死区平衡稳定性和响应性
#define CHASSIS_DEADZONE_VX 0.15f    // X方向速度死区(m/s)
#define CHASSIS_DEADZONE_VY 0.15f    // Y方向速度死区(m/s)
#define CHASSIS_DEADZONE_WZ 0.2f     // 旋转速度死区(rad/s)
#define CHASSIS_SPEED_DEADZONE 120.0f  // 增大电机速度死区(deg/s)，提高稳定性

// 电机稳定延迟，上电后等待电机稳定
#define MOTOR_STABILIZE_TIME_MS 2000U  // 2秒稳定时间

/* 电机ID定义 - 与实际硬件连接对应 */
#define MOTOR_FRONT_RIGHT  1U
#define MOTOR_FRONT_LEFT   2U
#define MOTOR_BACK_RIGHT   3U
#define MOTOR_BACK_LEFT    4U

/* Private variables ---------------------------------------------------------*/
static RC_ctrl_t *rc_data = NULL;
static DJIMotorInstance *chassis_motors[CHASSIS_MOTOR_COUNT] = {NULL};

/* 底盘运动变量 */
static float chassis_vx = 0.0f;   // X轴速度(m/s)
static float chassis_vy = 0.0f;   // Y轴速度(m/s)
static float chassis_wz = 0.0f;   // Z轴旋转速度(rad/s)
static float boost_factor = 1.0f; // 冲刺因子，1.0表示正常速度，>1.0表示冲刺
static float brake_factor = 1.0f;  // 刹车因子，1.0表示不刹车，<1.0表示刹车

/* 运动学解算结果 - 四个轮子的速度 */
static float wheel_speeds[CHASSIS_MOTOR_COUNT] = {0.0f};

/* 电机稳定标志 - 上电后等待一段时间再允许控制 */
static uint8_t motors_stabilized = 0;
static uint32_t system_start_time = 0;

/* 零速控制标志 - 在未收到遥控器指令时保持电机零速 */
static uint8_t zero_speed_control = 1;

/* 速度平滑滤波变量 */
static float filtered_vx = 0.0f;
static float filtered_vy = 0.0f;
static float filtered_wz = 0.0f;

/* 平滑滤波系数 - 越小越平滑，但响应越慢 */
#define SPEED_FILTER_COEF 0.65f  // 0-1之间，增加以提高稳定性，但保持响应速度

/* 上次控制时间戳，用于计算时间差 */
static uint32_t last_control_time = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Debug_DisableWatchdogs(void);
static void ChassisMotorsInit(void);
static void UpdateChassisKinematics(void);
static void ProcessRemoteControl(void);
static void SendChassisInfo(void);
static void SetAllMotorsZero(void);
static void ForceMotorZero(void);
static uint8_t DetectAndSuppressOscillation(void);
static uint8_t IsRCValueReasonable(int16_t value);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 在调试模式下冻结 IWDG/WWDG，避免单步调试时复位
 */
static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

/**
 * @brief 初始化底盘电机
 */
static void ChassisMotorsInit(void)
{
    const uint8_t motor_ids[CHASSIS_MOTOR_COUNT] = {
        MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT, MOTOR_BACK_RIGHT, MOTOR_BACK_LEFT
    };
    
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        Motor_Init_Config_s config = {
            .can_init_config = {
                .can_handle = &hcan1,
                .tx_id = motor_ids[i],
            },
            .controller_param_init_config = {
                .angle_PID = {
                    .Kp = 5.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .MaxOut = M3508_SPEED_MAX,
                    .IntegralLimit = 500.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                },
                .speed_PID = {
                    .Kp = 4.0f,  // 稍微减小Kp，提高稳定性
                    .Ki = 0.04f, // 减小Ki，防止积分饱和
                    .Kd = 0.0f, // 增大Kd，更好地抑制振荡
                    .IntegralLimit = 1000.0f, // 减小积分限制，防止累积误差
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .MaxOut = 10000.0f, // 适中的输出限制，平衡速度和稳定性
                },
                .current_PID = {
                    .Kp = 0.35f, // 减小Kp，提高稳定性
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .IntegralLimit = 1000.0f, // 减小积分限制，防止累积误差
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                    .MaxOut = 8000.0f, // 适中的输出限制，防止过冲
                },
            },
            .controller_setting_init_config = {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type = SPEED_LOOP,
                .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
                .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            },
            .motor_type = M3508,
        };
        
        chassis_motors[i] = DJIMotorInit(&config);
        LOGINFO("[chassis] Motor %d (ID %u) initialized", i, motor_ids[i]);
        
        if (chassis_motors[i] != NULL) {
            // 设置速度控制模式
            DJIMotorOuterLoop(chassis_motors[i], SPEED_LOOP);
            // 先禁用电机，等待稳定期后再启用
            DJIMotorStop(chassis_motors[i]);
        }
    }
}

/**
 * @brief 十字全向轮运动学解算
 * 
 * 输入：底盘期望速度 (vx, vy, wz)
 * 输出：四个轮子的速度 [右前, 左前, 右后, 左后]
 * 
 * 十字全向轮布局（俯视）：
 *       2 (前)
 *       |
 *   1---+---3
 *       |
 *       4 (后)
 * 
 * 十字全向轮正确运动学模型：
 * v1 = vy - wx + (L * wz)      // 右前轮
 * v2 = vx + wy + (L * wz)      // 左前轮  
 * v3 = -vy + wx + (L * wz)     // 右后轮
 * v4 = -vx - wy + (L * wz)     // 左后轮
 * 
 * 其中：vx为前进方向，vy为左移方向，wz为旋转角速度
 * L为底盘中心到轮子的距离
 */
static void UpdateChassisKinematics(void)
{
    // 检查电机是否已经稳定
    if (!motors_stabilized) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - system_start_time >= MOTOR_STABILIZE_TIME_MS) {
            // 第一次达到稳定时间，启用所有电机
            if (!motors_stabilized) {
                for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
                    if (chassis_motors[i] != NULL) {
                        DJIMotorEnable(chassis_motors[i]);
                    }
                }
                // 初始化滤波变量
                filtered_vx = 0.0f;
                filtered_vy = 0.0f;
                filtered_wz = 0.0f;
                last_control_time = current_time;
            }
            motors_stabilized = 1;
            LOGINFO("[chassis] Motors stabilized, ready for control");
        } else {
            // 电机未稳定，不进行控制
            SetAllMotorsZero();
            return;
        }
    }
    
    // 如果处于零速控制模式，直接设置所有电机速度为0
    if (zero_speed_control) {
        SetAllMotorsZero();
        // 重置滤波器状态
        filtered_vx = 0.0f;
        filtered_vy = 0.0f;
        filtered_wz = 0.0f;
        return;
    }
    
    // 应用低通滤波器平滑速度变化
    filtered_vx = filtered_vx * SPEED_FILTER_COEF + chassis_vx * (1.0f - SPEED_FILTER_COEF);
    filtered_vy = filtered_vy * SPEED_FILTER_COEF + chassis_vy * (1.0f - SPEED_FILTER_COEF);
    filtered_wz = filtered_wz * SPEED_FILTER_COEF + chassis_wz * (1.0f - SPEED_FILTER_COEF);
    
    // 运动学解算 - 修正十字全向轮公式
    float L = CHASSIS_WHEEL_BASE / 2.0f;
    
    // 十字全向轮的运动学解算
    // 根据README中的标准运动学模型进行实现
    // 注意：这里假设vy为前进方向，vx为左移方向，wz为旋转角速度
    
    // 十字全向轮标准运动学模型
    // vx: 左右平移速度 (左为正，右为负)
    // vy: 前进后退速度 (前为正，后为负)
    // wz: 旋转角速度 (顺时针为正，逆时针为负)
    float v1 = filtered_vy - filtered_vx - (L * filtered_wz);      // 右前轮
    float v2 = filtered_vy + filtered_vx - (L * filtered_wz);      // 左前轮
    float v3 = -filtered_vy + filtered_vx - (L * filtered_wz);     // 右后轮
    float v4 = -filtered_vy - filtered_vx - (L * filtered_wz);     // 左后轮
    
    // 将线速度转换为角速度 (rad/s) 并应用速度倍增因子
    wheel_speeds[0] = v1 / CHASSIS_WHEEL_RADIUS * CHASSIS_SPEED_MULTIPLIER; // 右前轮
    wheel_speeds[1] = v2 / CHASSIS_WHEEL_RADIUS * CHASSIS_SPEED_MULTIPLIER; // 左前轮
    wheel_speeds[2] = v3 / CHASSIS_WHEEL_RADIUS * CHASSIS_SPEED_MULTIPLIER; // 右后轮
    wheel_speeds[3] = v4 / CHASSIS_WHEEL_RADIUS * CHASSIS_SPEED_MULTIPLIER; // 左后轮
    
    // 转换为度/秒，因为DJI电机控制期望速度单位为度/秒
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        wheel_speeds[i] = wheel_speeds[i] * 180.0f / PI;
        
        // 应用速度死区，防止微小抖动
        if (fabs(wheel_speeds[i]) < CHASSIS_SPEED_DEADZONE) {
            wheel_speeds[i] = 0.0f;
        }
        
        // 应用动态速度限制 - 根据输入大小调整最大输出
        float input_magnitude = sqrtf(chassis_vx*chassis_vx + chassis_vy*chassis_vy + chassis_wz*chassis_wz);
        float dynamic_max_speed = M3508_SPEED_MAX;
        
        // 如果输入很小，限制最大输出速度，防止抖动
        if (input_magnitude < 1.0f) {
            dynamic_max_speed = M3508_SPEED_MAX * 0.6f;  // 小输入时降低最大速度
        } else if (input_magnitude < 3.0f) {
            dynamic_max_speed = M3508_SPEED_MAX * 0.8f;  // 中等输入时适度降低最大速度
        }
        
        // 限制在电机最大速度范围内
        wheel_speeds[i] = float_constrain(wheel_speeds[i], M3508_SPEED_MIN, dynamic_max_speed);
        
        // 设置电机速度
        if (chassis_motors[i] != NULL) {
            DJIMotorSetRef(chassis_motors[i], wheel_speeds[i]);
        }
    }
    
    // 更新控制时间戳
    last_control_time = HAL_GetTick();
}

/**
 * @brief 处理遥控器输入，转换为底盘速度命令
 * 
 * 摇杆控制映射：
 * - 左摇杆：前后左右控制底盘平动（上下为前进后退，左右为左右平移）
 * - 右摇杆左右：控制底盘旋转
 * - 右摇杆上下：控制冲刺和刹车
 */
static void ProcessRemoteControl(void)
{
    if (rc_data == NULL || !RemoteControlIsOnline()) {
        // 遥控器未连接，停止底盘
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        chassis_wz = 0.0f;
        boost_factor = 1.0f;
        brake_factor = 1.0f;
        // 保持零速控制模式
        zero_speed_control = 1;
        return;
    }
    
    const RC_ctrl_t *rc = &rc_data[TEMP];
    
    // 检查摇杆值是否合理，防止干扰噪声
    if (!IsRCValueReasonable(rc->rc.rocker_l_) || 
        !IsRCValueReasonable(rc->rc.rocker_l1) || 
        !IsRCValueReasonable(rc->rc.rocker_r_) || 
        !IsRCValueReasonable(rc->rc.rocker_r1)) {
        // 摇杆值异常，停止底盘
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        chassis_wz = 0.0f;
        boost_factor = 1.0f;
        brake_factor = 1.0f;
        // 保持零速控制模式
        zero_speed_control = 1;
        return;
    }
    
    // 检查是否所有摇杆都在死区内 - 减小死区范围，提高控制灵敏度
    uint8_t all_in_deadzone = (fabs(rc->rc.rocker_l_) < 50) && 
                             (fabs(rc->rc.rocker_l1) < 50) && 
                             (fabs(rc->rc.rocker_r_) < 50) && 
                             (fabs(rc->rc.rocker_r1) < 50);
    
    if (all_in_deadzone && !rc->key[KEY_PRESS].w && !rc->key[KEY_PRESS].s && 
        !rc->key[KEY_PRESS].a && !rc->key[KEY_PRESS].d && 
        !rc->key[KEY_PRESS].q && !rc->key[KEY_PRESS].e) {
        // 所有摇杆在死区内且没有按键按下，保持零速控制模式
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        chassis_wz = 0.0f;
        boost_factor = 1.0f;
        brake_factor = 1.0f;
        zero_speed_control = 1;
        return;
    }
    
    // 有控制输入，退出零速控制模式
    zero_speed_control = 0;
    
    // 左摇杆控制底盘平动
    // 左右：rocker_l_，范围-660~660，映射到-CHASSIS_MAX_VEL~CHASSIS_MAX_VEL（控制vx左右平移）
    // 修正方向：左推左移，右推右移
    chassis_vx = rc->rc.rocker_l_ / 660.0f * CHASSIS_MAX_VEL;
    
    // 前后：rocker_l1，范围-660~660，映射到-CHASSIS_MAX_VEL~CHASSIS_MAX_VEL（控制vy前进后退）
    // 修正方向：上推前进，下推后退
    chassis_vy = -rc->rc.rocker_l1 / 660.0f * CHASSIS_MAX_VEL;
    
    // 右摇杆左右控制底盘旋转
    // 旋转：rocker_r_，范围-660~660，映射到-CHASSIS_MAX_ROTATE~CHASSIS_MAX_ROTATE
    // 根据README：wz为旋转角速度 (顺时针为正，逆时针为负)
    // 右推是顺时针（正），左推是逆时针（负）
    chassis_wz = -rc->rc.rocker_r_ / 660.0f * CHASSIS_MAX_ROTATE;
    
    // 右摇杆上下控制冲刺和刹车
    // rocker_r1范围-660~660，正值向上推（冲刺），负值向下推（刹车）
    if (rc->rc.rocker_r1 > 50) {  // 向上推，冲刺 - 减小阈值
        // 计算冲刺因子，从1.0到(CHASSIS_BOOST_VEL/CHASSIS_MAX_VEL)
        boost_factor = 1.0f + (rc->rc.rocker_r1 / 660.0f) * (CHASSIS_BOOST_VEL/CHASSIS_MAX_VEL - 1.0f);
        brake_factor = 1.0f;  // 不刹车
    } else if (rc->rc.rocker_r1 < -50) {  // 向下推，刹车 - 减小阈值
        // 计算刹车因子，从1.0到CHASSIS_BRAKE_FACTOR
        brake_factor = 1.0f + (rc->rc.rocker_r1 / 660.0f) * (1.0f - CHASSIS_BRAKE_FACTOR);
        boost_factor = 1.0f;  // 不冲刺
    } else {
        // 中间位置，不冲刺不刹车
        boost_factor = 1.0f;
        brake_factor = 1.0f;
    }
    
    // 如果按键按下，可以执行特定动作（可选）
    if (rc->key[KEY_PRESS].w) {
        // W键按下：向前
        chassis_vy = CHASSIS_MAX_VEL;
    } else if (rc->key[KEY_PRESS].s) {
        // S键按下：向后
        chassis_vy = -CHASSIS_MAX_VEL;
    }
    
    if (rc->key[KEY_PRESS].a) {
        // A键按下：向左平移
        chassis_vx = -CHASSIS_MAX_VEL;
    } else if (rc->key[KEY_PRESS].d) {
        // D键按下：向右平移
        chassis_vx = CHASSIS_MAX_VEL;
    }
    
    if (rc->key[KEY_PRESS].q) {
        // Q键按下：逆时针旋转（负值）
        chassis_wz = CHASSIS_MAX_ROTATE;
    } else if (rc->key[KEY_PRESS].e) {
        // E键按下：顺时针旋转（正值）
        chassis_wz = -CHASSIS_MAX_ROTATE;
    }
    
    // 应用冲刺和刹车因子
    chassis_vx *= boost_factor * brake_factor;
    chassis_vy *= boost_factor * brake_factor;
    
    // 速度死区处理，避免微小抖动
    chassis_vx = float_deadband(chassis_vx, -CHASSIS_DEADZONE_VX, CHASSIS_DEADZONE_VX);
    chassis_vy = float_deadband(chassis_vy, -CHASSIS_DEADZONE_VY, CHASSIS_DEADZONE_VY);
    chassis_wz = float_deadband(chassis_wz, -CHASSIS_DEADZONE_WZ, CHASSIS_DEADZONE_WZ);
}

/**
 * @brief 设置所有电机速度为0（用于稳定期）
 */
static void SetAllMotorsZero(void)
{
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (chassis_motors[i] != NULL) {
            DJIMotorSetRef(chassis_motors[i], 0.0f);
        }
    }
    
    // 同时重置底盘速度变量，防止累积误差
    chassis_vx = 0.0f;
    chassis_vy = 0.0f;
    chassis_wz = 0.0f;
    boost_factor = 1.0f;
    brake_factor = 1.0f;
}

/**
 * @brief 检测和抑制电机微小抖动
 * @return 1:检测到抖动，0:正常
 */
static uint8_t DetectAndSuppressOscillation(void)
{
    // 如果处于零速控制模式，不检测
    if (zero_speed_control) {
        return 0;
    }
    
    // 检查所有电机的期望速度是否接近0（使用wheel_speeds数组作为参考）
    uint8_t oscillation_detected = 0;
    static uint8_t oscillation_count = 0;
    
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        // 如果设定值接近0但底盘速度不为0，可能是抖动
        if (fabs(wheel_speeds[i]) < CHASSIS_SPEED_DEADZONE && 
            (fabs(chassis_vx) > 0.1f || fabs(chassis_vy) > 0.1f || fabs(chassis_wz) > 0.1f)) {
            oscillation_detected = 1;
            break;
        }
    }
    
    // 另一种检测方法：检查摇杆是否在死区内，但底盘速度不为0
    if (rc_data != NULL && RemoteControlIsOnline()) {
        const RC_ctrl_t *rc = &rc_data[TEMP];
        uint8_t all_in_deadzone = (fabs(rc->rc.rocker_l_) < 50) && 
                                 (fabs(rc->rc.rocker_l1) < 50) && 
                                 (fabs(rc->rc.rocker_r_) < 50) && 
                                 (fabs(rc->rc.rocker_r1) < 50) &&
                                 !rc->key[KEY_PRESS].w && !rc->key[KEY_PRESS].s && 
                                 !rc->key[KEY_PRESS].a && !rc->key[KEY_PRESS].d && 
                                 !rc->key[KEY_PRESS].q && !rc->key[KEY_PRESS].e;
        
        // 更严格的检测 - 只有在摇杆完全在死区内且底盘有持续运动时才认为是抖动
        if (all_in_deadzone && (fabs(chassis_vx) > 0.1f || fabs(chassis_vy) > 0.1f || fabs(chassis_wz) > 0.1f)) {
            oscillation_detected = 1;
        }
    }
    
    if (oscillation_detected) {
        oscillation_count++;
        // 连续检测到抖动，强制清零
        if (oscillation_count > 3) {
            SetAllMotorsZero();
            zero_speed_control = 1;
            oscillation_count = 0;
            LOGWARNING("[chassis] Motor oscillation detected, forcing zero speed");
            return 1;
        }
    } else {
        oscillation_count = 0;
    }
    
    return 0;
}

/**
 * @brief 强制清零电机状态，用于初始化和防止抖动
 */
static void ForceMotorZero(void)
{
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (chassis_motors[i] != NULL) {
            // 先禁用电机
            DJIMotorStop(chassis_motors[i]);
            // 清零参考值
            DJIMotorSetRef(chassis_motors[i], 0.0f);
            // 短暂延时确保命令发送
            HAL_Delay(10);
            // 重新启用电机
            DJIMotorEnable(chassis_motors[i]);
            // 再次清零参考值
            DJIMotorSetRef(chassis_motors[i], 0.0f);
        }
    }
    
    // 重置所有控制变量
    chassis_vx = 0.0f;
    chassis_vy = 0.0f;
    chassis_wz = 0.0f;
    boost_factor = 1.0f;
    brake_factor = 1.0f;
    zero_speed_control = 1;
    
    // 重置滤波变量
    filtered_vx = 0.0f;
    filtered_vy = 0.0f;
    filtered_wz = 0.0f;
}

/**
 * @brief 检查遥控器摇杆值是否合理，防止干扰噪声
 * @param value 摇杆原始值
 * @return 1:值合理，0:值异常
 */
static uint8_t IsRCValueReasonable(int16_t value)
{
    // 检查值是否在合理范围内（-660到660）
    if (value < -660 || value > 660) {
        return 0;
    }
    return 1;
}

/**
 * @brief 发送底盘信息（调试用）
 */
static void SendChassisInfo(void)
{
    static uint32_t last_send_tick = 0;
    uint32_t now = HAL_GetTick();
    
    // 每隔500ms发送一次底盘状态
    if (now - last_send_tick >= 500U) {
        last_send_tick = now;
        
        // 通过USART6发送底盘状态（如果有遥测串口）
        char buffer[256];
        int len = snprintf(buffer, sizeof(buffer),
            "[chassis] vx=%.2f vy=%.2f wz=%.2f\r\n"
            "Boost=%.2f Brake=%.2f\r\n"
            "Wheels[0]=%.1f [1]=%.1f [2]=%.1f [3]=%.1f\r\n",
            chassis_vx, chassis_vy, chassis_wz, boost_factor, brake_factor,
            wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3]);
        
        if (len > 0) {
            // 这里可以选择性地通过某个串口发送遥测数据
            // USARTSend(telemetry_usart, (uint8_t *)buffer, (uint16_t)len, USART_TRANSFER_BLOCKING);
        }
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    Debug_DisableWatchdogs();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_SPI1_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_USART3_UART_Init();
    MX_RNG_Init();
    MX_RTC_Init();
    MX_TIM1_Init();
    MX_TIM10_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_TIM8_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_SPI2_Init();
    MX_CRC_Init();
    MX_DAC_Init();

    BSPInit();
    
    // 初始化底盘电机
    ChassisMotorsInit();
    
    // 强制清零电机状态，防止上电抖动
    ForceMotorZero();
    
    // 初始化遥控器
    rc_data = RemoteControlInit(&huart3);

    // 记录系统启动时间，用于电机稳定判断
    system_start_time = HAL_GetTick();

    LOGINFO("[chassis] Omni-directional chassis demo initialized");
    LOGINFO("[chassis] Using left stick for translation (up/down for forward/back, left/right for strafe)");
    LOGINFO("[chassis] Using right stick left/right for rotation, up/down for boost/brake");
    LOGINFO("[chassis] Keys: W/S for forward/back, A/D for strafe, Q/E for rotation");
    LOGINFO("[chassis] Motors will stabilize for %d ms before control", MOTOR_STABILIZE_TIME_MS);
    LOGINFO("[chassis] Anti-jitter measures applied - motors will remain at zero speed until input detected");

    uint32_t last_update_tick = 0;

    while (1)
    {
        // 更新后台任务（包括电机控制）
        DaemonTask();
        DJIMotorControl();
        
        uint32_t now = HAL_GetTick();
        
        // 每隔CHASSIS_UPDATE_INTERVAL_MS更新一次底盘控制
        if (now - last_update_tick >= CHASSIS_UPDATE_INTERVAL_MS) {
            last_update_tick = now;
            
            // 处理遥控器输入
            ProcessRemoteControl();
            
            // 更新运动学解算
            UpdateChassisKinematics();
            
            // 检测和抑制电机抖动
            DetectAndSuppressOscillation();
            
            // 发送底盘状态
            SendChassisInfo();
        }

        HAL_Delay(5);
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/**
 * @brief TIM14 1ms 中断回调，用于累加 HAL 的系统节拍
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14)
    {
        HAL_IncTick();
    }
}
/* USER CODE END 4 */
