                                        /**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 云台遥控控制演示
 ******************************************************************************
 * @attention
 *
 * 本测试固件整合了遥控接收和电机控制，实现云台的遥控速度控制。
 * 右摇杆左右控制yaw轴，右摇杆上下控制pitch轴。
 *
 * 电机配置：
 * - yaw 轴 GM6020: CAN1, ID 2
 * - pitch 轴 GM6020: CAN2, ID 5
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
#include "daemon.h"
#include "dt7_remote.h"
#include "dji_motor.h"
#include "user_lib.h"

/* Private define ------------------------------------------------------------*/
#define GIMBAL_UPDATE_INTERVAL_MS 20U  // 50Hz更新频率

#define GM6020_SPEED_MAX 3600.0f
#define GM6020_SPEED_MIN (-GM6020_SPEED_MAX)
#define GM6020_SPEED_DEADZONE 30.0f

#define RC_DEADZONE 50

/* 电机ID定义 - 与实际硬件连接对应 */
#define YAW_MOTOR_ID   2U
#define PITCH_MOTOR_ID 5U

/* Private variables ---------------------------------------------------------*/
static RC_ctrl_t *rc_data = NULL;
static DJIMotorInstance *yaw_motor = NULL;
static DJIMotorInstance *pitch_motor = NULL;

static float yaw_speed_ref = 0.0f;   // deg/s
static float pitch_speed_ref = 0.0f; // deg/s

static uint8_t zero_speed_control = 1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Debug_DisableWatchdogs(void);
static void GimbalMotorsInit(void);
static void ProcessRemoteControl(void);
static void UpdateGimbalControl(void);
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
 * @brief 初始化云台电机
 */
static void GimbalMotorsInit(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = YAW_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .DeadBand = 0.1f,
                .IntegralLimit = 100.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = 500.0f,
            },
            .speed_PID = {
                .Kp = 10.0f,
                .Ki = 40.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = 20000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };

    yaw_motor = DJIMotorInit(&config);
    LOGINFO("[gimbal] Yaw GM6020 registered on CAN1 id %u", (unsigned)YAW_MOTOR_ID);

    config.can_init_config.can_handle = &hcan2;
    config.can_init_config.tx_id = PITCH_MOTOR_ID;
    pitch_motor = DJIMotorInit(&config);
    LOGINFO("[gimbal] Pitch GM6020 registered on CAN2 id %u", (unsigned)PITCH_MOTOR_ID);

    if (yaw_motor != NULL) {
        DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
        DJIMotorStop(yaw_motor);
    }

    if (pitch_motor != NULL) {
        DJIMotorOuterLoop(pitch_motor, SPEED_LOOP);
        DJIMotorStop(pitch_motor);
    }
}

/**
 * @brief 处理遥控器输入，转换为云台速度命令
 *
 * 摇杆控制映射：
 * - 右摇杆左右：yaw 轴转动速度
 * - 右摇杆上下：pitch 轴转动速度
 */
static void ProcessRemoteControl(void)
{
    if (rc_data == NULL || !RemoteControlIsOnline()) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        zero_speed_control = 1;
        return;
    }

    const RC_ctrl_t *rc = &rc_data[TEMP];

    if (!IsRCValueReasonable(rc->rc.rocker_r_) ||
        !IsRCValueReasonable(rc->rc.rocker_r1)) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        zero_speed_control = 1;
        return;
    }

    uint8_t all_in_deadzone = (fabs(rc->rc.rocker_r_) < RC_DEADZONE) &&
                              (fabs(rc->rc.rocker_r1) < RC_DEADZONE);

    if (all_in_deadzone) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        zero_speed_control = 1;
        return;
    }

    zero_speed_control = 0;

    // 右摇杆左右控制yaw，右摇杆上下控制pitch
    yaw_speed_ref = rc->rc.rocker_r_ / 660.0f * GM6020_SPEED_MAX;
    // 默认上推为俯仰上抬，如方向不符可调整正负号
    pitch_speed_ref = -rc->rc.rocker_r1 / 660.0f * GM6020_SPEED_MAX;

    yaw_speed_ref = float_deadband(yaw_speed_ref, -GM6020_SPEED_DEADZONE, GM6020_SPEED_DEADZONE);
    pitch_speed_ref = float_deadband(pitch_speed_ref, -GM6020_SPEED_DEADZONE, GM6020_SPEED_DEADZONE);
}

/**
 * @brief 更新云台电机速度控制
 */
static void UpdateGimbalControl(void)
{
    if (zero_speed_control) {
        if (yaw_motor != NULL) {
            DJIMotorSetRef(yaw_motor, 0.0f);
        }
        if (pitch_motor != NULL) {
            DJIMotorSetRef(pitch_motor, 0.0f);
        }
        return;
    }

    yaw_speed_ref = float_constrain(yaw_speed_ref, GM6020_SPEED_MIN, GM6020_SPEED_MAX);
    pitch_speed_ref = float_constrain(pitch_speed_ref, GM6020_SPEED_MIN, GM6020_SPEED_MAX);

    if (yaw_motor != NULL) {
        DJIMotorEnable(yaw_motor);
        DJIMotorSetRef(yaw_motor, yaw_speed_ref);
    }

    if (pitch_motor != NULL) {
        DJIMotorEnable(pitch_motor);
        DJIMotorSetRef(pitch_motor, pitch_speed_ref);
    }
}

/**
 * @brief 检查遥控器摇杆值是否合理，防止干扰噪声
 * @param value 摇杆原始值
 * @return 1:值合理，0:值异常
 */
static uint8_t IsRCValueReasonable(int16_t value)
{
    if (value < -660 || value > 660) {
        return 0;
    }
    return 1;
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
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

    GimbalMotorsInit();

    rc_data = RemoteControlInit(&huart3);

    LOGINFO("[gimbal] Gimbal demo initialized");
    LOGINFO("[gimbal] Right stick left/right -> yaw speed, up/down -> pitch speed");

    uint32_t last_update_tick = 0;

    while (1)
    {
        DaemonTask();
        DJIMotorControl();

        uint32_t now = HAL_GetTick();
        if (now - last_update_tick >= GIMBAL_UPDATE_INTERVAL_MS) {
            last_update_tick = now;

            ProcessRemoteControl();
            UpdateGimbalControl();
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
