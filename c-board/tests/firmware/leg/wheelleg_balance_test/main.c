/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Wheel-leg balance test (fixed joint pose + IMU pitch)
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"

#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include <math.h>
#include <string.h>

#include "bsp_init.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dji_motor.h"
#include "dm_imu.h"
#include "dmmotor.h"
#include "et08_remote.h"
#include "utils.h"

#define CONTROL_INTERVAL_MS 5U
#define MIT_SEND_INTERVAL_MS 2U
#define ENABLE_INTERVAL_MS 100U
#define TELEMETRY_INTERVAL_MS 50U

#define IMU_REQUEST_INTERVAL_MS 20U
#define IMU_TIMEOUT_MS 200U

#define IMU_ID 0x01
#define IMU_MASTER_ID 0x00
#define IMU_RX_ID (IMU_MASTER_ID)
#define IMU_VALID_ACCEL (1U << 0)
#define IMU_VALID_GYRO (1U << 1)
#define IMU_VALID_EULER (1U << 2)

#define FRONT_LEFT_ID 1U
#define REAR_LEFT_ID 2U
#define FRONT_RIGHT_ID 3U
#define REAR_RIGHT_ID 4U
#define DM_MASTER_ID 0x00U

#define DM_P_RANGE 12.5f
#define DM_V_RANGE 45.0f
#define DM_T_RANGE 54.0f

/* Stand pose (rad) from loaded stable stance.
 * Update these from "joint pos" logs when the robot is stable on the ground. */

// 电机逆时针转为正方向
#define HIP_TARGET_L -0.10f // 下摆 顺转
#define KNEE_TARGET_L 1.35f // 收起顺转 伸展逆转

#define HIP_TARGET_R -0.25f // 下摆 逆转
#define KNEE_TARGET_R -0.65f //收起逆转 伸展顺转
// #define HIP_TARGET_L 0.92f
// #define HIP_TARGET_R -0.84f
// #define KNEE_TARGET_L 0.05f
// #define KNEE_TARGET_R -0.08f

/* Joint stiffness (per motor).
 * Note: MIT KD is clamped by the driver (<= 5.0). Keep KD in [0..5]. */
//left
#define FL_KP 50.0f
#define FL_KD 1.5f
#define RL_KP 60.0f
#define RL_KD 2.5f

//right
#define FR_KP 50.0f
#define FR_KD 1.5f
#define RR_KP 60.0f
#define RR_KD 1.5f

/* Torque feedforward (TFF) to fight gravity.
 * Use the sign/magnitude from "tq[]" logs at stable stance, then tune in small steps. */
#define HIP_TFF_L -6.0f
#define KNEE_TFF_L 8.0f

#define HIP_TFF_R 6.0f
#define KNEE_TFF_R -8.0f
/* Hip TFF sharing between the two motors on the same leg.
 * rear motor also contributes to hip due to series transmission (rear ~= hip+knee),
 * so put part of hip TFF on rear to reduce "motor fighting" / backlash buzz. */
#define HIP_TFF_SHARE 0.5f /* 0..1: front gets share, rear gets (1-share) */

/* Balance controller (deg-based) */
#define BALANCE_PITCH_KP 950.0f
#define BALANCE_PITCH_KD 70.0f
#define BALANCE_PITCH_KI 0.00f
#define BALANCE_PITCH_I_LIMIT 50.0f
#define BALANCE_OUT_MAX 20000.0f
#define BALANCE_TILT_MAX_DEG 40.0f
#define BALANCE_PITCH_SIGN 1.0f
#define BALANCE_REF_FIXED_DEG -4.70f

/* Wheel command limits */
#define WHEEL_SPEED_MAX 25000.0f
#define WHEEL_SPEED_MIN (-WHEEL_SPEED_MAX)

/* RC mapping */
#define RC_STICK_MAX 660.0f
#define RC_DEADZONE 30
#define RC_FWD_MAX 3000.0f
#define RC_YAW_MAX 2000.0f

typedef struct
{
    DMMotor_Handle *handle;
    float target_p;
    float kp;
    float kd;
    float t_ff;
} JointMotor;

typedef enum
{
    BALANCE_MODE_DISABLE = 0,
    BALANCE_MODE_ACTIVE = 1,
} BalanceMode;

static USARTInstance *telemetry_usart = NULL;
static ET08_Ctrl_t *et08_ctrl = NULL;
static dm_imu_t dm_imu;

static DJIMotorInstance *wheel_left = NULL;
static DJIMotorInstance *wheel_right = NULL;

static JointMotor front_left;
static JointMotor rear_left;
static JointMotor front_right;
static JointMotor rear_right;

static float pitch_deg = 0.0f;
static float pitch_rate_deg_s = 0.0f;
static uint32_t imu_last_ms = 0;
static uint8_t imu_valid = 0;

static float pitch_ref_deg = BALANCE_REF_FIXED_DEG;
static float balance_i = 0.0f;
static uint8_t balance_active = 0;
static float balance_err_deg = 0.0f;
static float balance_out = 0.0f;
static float forward_cmd = 0.0f;
static float yaw_cmd = 0.0f;
static uint8_t imu_alive = 0;

static float wheel_left_ref = 0.0f;
static float wheel_right_ref = 0.0f;
static uint8_t wheel_enable = 0;

static BalanceMode balance_mode = BALANCE_MODE_DISABLE;
static uint32_t last_enable_tick = 0;

void SystemClock_Config(void);
void Error_Handler(void);

static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

static void TelemetryInit(void)
{
    USART_Init_Config_s config = {
        .module_callback = NULL,
        .recv_buff_size = 1,
        .usart_handle = &huart6,
    };
    telemetry_usart = USARTRegister(&config);
}

static void TelemetrySend(const char *msg)
{
    if (!telemetry_usart || !msg)
        return;
    USARTSend(telemetry_usart, (uint8_t *)msg, (uint16_t)strlen(msg), USART_TRANSFER_BLOCKING);
}

static float ClampFloat(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static int16_t ApplyDeadzone(int16_t value, int16_t deadzone)
{
    if (value > deadzone)
        return value - deadzone;
    if (value < -deadzone)
        return value + deadzone;
    return 0;
}

static void WheelMotorsInit(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 5.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = WHEEL_SPEED_MAX,
                .IntegralLimit = 500.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
            },
            .speed_PID = {
                .Kp = 5.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = 20000.0f,
            },
            .current_PID = {
                .Kp = 0.4f,
                .Ki = 0.0f,
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
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };

    wheel_left = DJIMotorInit(&config);
    LOGINFO("[wl_balance] wheel left id 1 ready");

    config.can_init_config.tx_id = 2;
    wheel_right = DJIMotorInit(&config);
    LOGINFO("[wl_balance] wheel right id 2 ready");
}

static void WheelMotorsUpdate(void)
{
    if (wheel_left)
    {
        DJIMotorOuterLoop(wheel_left, SPEED_LOOP);
        if (wheel_enable)
            DJIMotorEnable(wheel_left);
        else
            DJIMotorStop(wheel_left);
        DJIMotorSetRef(wheel_left, wheel_left_ref);
    }
    if (wheel_right)
    {
        DJIMotorOuterLoop(wheel_right, SPEED_LOOP);
        if (wheel_enable)
            DJIMotorEnable(wheel_right);
        else
            DJIMotorStop(wheel_right);
        DJIMotorSetRef(wheel_right, wheel_right_ref);
    }
}

static void JointMotorInit(JointMotor *joint, uint8_t motor_id, float kp, float kd, float t_ff, float target_p)
{
    if (!joint)
        return;

    DMMotor_InitConfig config = {
        .can_handle = &hcan1,
        .motor_id = motor_id,
        .master_id = DM_MASTER_ID,
        .auto_clear_error = true,
        .auto_enable_mit = false,
        .auto_zero_position = false,
        .use_shared_feedback_id = true,
        .position_range = DM_P_RANGE,
        .velocity_range = DM_V_RANGE,
        .torque_range = DM_T_RANGE,
    };

    joint->handle = DMMotor_Init(&config);
    joint->target_p = ClampFloat(target_p, -DM_P_RANGE, DM_P_RANGE);
    joint->kp = kp;
    joint->kd = kd;
    joint->t_ff = t_ff;
}

static void JointMotorsInit(void)
{
    const float hip_front_tff_l = HIP_TFF_L * HIP_TFF_SHARE;
    const float hip_rear_tff_l = HIP_TFF_L * (1.0f - HIP_TFF_SHARE);
    const float hip_front_tff_r = HIP_TFF_R * HIP_TFF_SHARE;
    const float hip_rear_tff_r = HIP_TFF_R * (1.0f - HIP_TFF_SHARE);

    JointMotorInit(&front_left, FRONT_LEFT_ID, FL_KP, FL_KD, hip_front_tff_l, HIP_TARGET_L);
    JointMotorInit(&front_right, FRONT_RIGHT_ID, FR_KP, FR_KD, hip_front_tff_r, HIP_TARGET_R);

    JointMotorInit(&rear_left, REAR_LEFT_ID, RL_KP, RL_KD, KNEE_TFF_L + hip_rear_tff_l,
                   HIP_TARGET_L + KNEE_TARGET_L);
    JointMotorInit(&rear_right, REAR_RIGHT_ID, RR_KP, RR_KD, KNEE_TFF_R + hip_rear_tff_r,
                   HIP_TARGET_R + KNEE_TARGET_R);
}

static void JointMotorsEnable(void)
{
    if (front_left.handle) {
        DMMotor_ClearError(front_left.handle, DM_MODE_MIT);
        DMMotor_Enable(front_left.handle, DM_MODE_MIT);
    }
    if (rear_left.handle) {
        DMMotor_ClearError(rear_left.handle, DM_MODE_MIT);
        DMMotor_Enable(rear_left.handle, DM_MODE_MIT);
    }
    if (front_right.handle) {
        DMMotor_ClearError(front_right.handle, DM_MODE_MIT);
        DMMotor_Enable(front_right.handle, DM_MODE_MIT);
    }
    if (rear_right.handle) {
        DMMotor_ClearError(rear_right.handle, DM_MODE_MIT);
        DMMotor_Enable(rear_right.handle, DM_MODE_MIT);
    }
}

static void JointMotorsSendMIT(void)
{
    if (front_left.handle)
        DMMotor_SendMIT(front_left.handle, front_left.target_p, 0.0f, front_left.kp, front_left.kd, front_left.t_ff);
    if (rear_left.handle)
        DMMotor_SendMIT(rear_left.handle, rear_left.target_p, 0.0f, rear_left.kp, rear_left.kd, rear_left.t_ff);
    if (front_right.handle)
        DMMotor_SendMIT(front_right.handle, front_right.target_p, 0.0f, front_right.kp, front_right.kd, front_right.t_ff);
    if (rear_right.handle)
        DMMotor_SendMIT(rear_right.handle, rear_right.target_p, 0.0f, rear_right.kp, rear_right.kd, rear_right.t_ff);
}

static void ImuInit(void)
{
    dm_imu_can_config_t can_cfg = {
        .can_handle = &hcan2,
        .can_id = IMU_ID,
        .mst_id = IMU_RX_ID,
    };
    dm_imu_init_can(&dm_imu, &can_cfg);
    dm_imu_can_set_active(&dm_imu, false);
}

static void ImuRequestOnce(void)
{
    dm_imu_can_request_accel(&dm_imu);
    dm_imu_can_request_gyro(&dm_imu);
    dm_imu_can_request_euler(&dm_imu);
}

static void ImuUpdate(void)
{
    dm_imu_data_t data;
    if (!dm_imu_get_data(&dm_imu, &data))
        return;
    if ((data.valid_mask & IMU_VALID_EULER) == 0)
        return;

    pitch_deg = data.euler[1];
    if (data.valid_mask & IMU_VALID_GYRO)
    {
        /* DM-IMU gyro range is ~[-34.88, 34.88]; treat it as rad/s and convert to deg/s. */
        pitch_rate_deg_s = data.gyro[1] * 57.2957795f;
    }
    else
    {
        pitch_rate_deg_s = 0.0f;
    }
    imu_last_ms = HAL_GetTick();
    imu_valid = 1;
}

static uint8_t IsSaDown(const ET08_Ctrl_t *ctrl)
{
    if (!ctrl)
        return 0;
    if (ctrl->switch_sa_sb_state != 0xFF)
        return (ctrl->switch_sa_sb_state >= 3U) ? 1U : 0U;
    return (ctrl->switch_sa_sb_centered < -200) ? 1U : 0U;
}

static uint8_t IsSaUp(const ET08_Ctrl_t *ctrl)
{
    if (!ctrl)
        return 0;
    if (ctrl->switch_sa_sb_state != 0xFF)
        return (ctrl->switch_sa_sb_state <= 2U) ? 1U : 0U;
    return (ctrl->switch_sa_sb_centered > 200) ? 1U : 0U;
}

static BalanceMode GetBalanceMode(const ET08_Ctrl_t *ctrl)
{
    if (!ctrl)
        return BALANCE_MODE_DISABLE;
    if (IsSaDown(ctrl))
        return BALANCE_MODE_DISABLE;
    if (IsSaUp(ctrl))
        return BALANCE_MODE_ACTIVE;
    return BALANCE_MODE_DISABLE;
}

static void UpdateControl(float dt_sec)
{
    if (!ET08_IsOnline() || et08_ctrl == NULL)
        balance_mode = BALANCE_MODE_DISABLE;
    else
        balance_mode = GetBalanceMode(et08_ctrl);

    imu_alive = imu_valid && ((HAL_GetTick() - imu_last_ms) <= IMU_TIMEOUT_MS);
    if (imu_alive)
        balance_err_deg = (pitch_deg - pitch_ref_deg) * BALANCE_PITCH_SIGN;
    else
        balance_err_deg = 0.0f;

    if (balance_mode == BALANCE_MODE_DISABLE)
    {
        wheel_enable = 0;
        wheel_left_ref = 0.0f;
        wheel_right_ref = 0.0f;
        balance_active = 0;
        balance_i = 0.0f;
        balance_out = 0.0f;
        forward_cmd = 0.0f;
        yaw_cmd = 0.0f;
        return;
    }

    wheel_enable = 1;

    if (!balance_active && balance_mode == BALANCE_MODE_ACTIVE)
    {
        pitch_ref_deg = BALANCE_REF_FIXED_DEG;
        balance_i = 0.0f;
        balance_active = 1;
    }

    int16_t right_y = ApplyDeadzone(et08_ctrl->right.y, RC_DEADZONE);
    int16_t left_x = ApplyDeadzone(et08_ctrl->left.x, RC_DEADZONE);

    forward_cmd = (float)right_y / RC_STICK_MAX * RC_FWD_MAX;
    yaw_cmd = (float)left_x / RC_STICK_MAX * RC_YAW_MAX;

    balance_out = 0.0f;
    if (balance_mode == BALANCE_MODE_ACTIVE && imu_alive)
    {
        if (fabsf(balance_err_deg) <= BALANCE_TILT_MAX_DEG)
        {
            if (BALANCE_PITCH_KI != 0.0f)
            {
                balance_i += balance_err_deg * dt_sec;
                balance_i = ClampFloat(balance_i, -BALANCE_PITCH_I_LIMIT, BALANCE_PITCH_I_LIMIT);
            }
            else
            {
                balance_i = 0.0f;
            }
            balance_out = BALANCE_PITCH_KP * balance_err_deg +
                          BALANCE_PITCH_KD * pitch_rate_deg_s +
                          BALANCE_PITCH_KI * balance_i;
            balance_out = ClampFloat(balance_out, -BALANCE_OUT_MAX, BALANCE_OUT_MAX);
        }
        else
        {
            balance_i = 0.0f;
            balance_out = 0.0f;
        }
    }
    else
    {
        balance_i = 0.0f;
        balance_out = 0.0f;
    }

    float base_speed = balance_out + forward_cmd;
    wheel_left_ref = -(base_speed + yaw_cmd);
    wheel_right_ref = (base_speed - yaw_cmd);

    wheel_left_ref = ClampFloat(wheel_left_ref, WHEEL_SPEED_MIN, WHEEL_SPEED_MAX);
    wheel_right_ref = ClampFloat(wheel_right_ref, WHEEL_SPEED_MIN, WHEEL_SPEED_MAX);
}

int main(void)
{
    HAL_Init();
    Debug_DisableWatchdogs();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();

    BSPInit();

    TelemetryInit();
    WheelMotorsInit();
    JointMotorsInit();

    et08_ctrl = ET08_Init(&huart3);
    ImuInit();

    TelemetrySend("wheelleg_balance: start\r\n");

    uint32_t last_control_tick = 0;
    uint32_t last_mit_tick = 0;
    uint32_t last_imu_req_tick = 0;
    uint32_t last_telemetry_tick = 0;

    while (1)
    {
        DaemonTask();
        DJIMotorControl();

        uint32_t now = HAL_GetTick();

        if (now - last_imu_req_tick >= IMU_REQUEST_INTERVAL_MS)
        {
            last_imu_req_tick = now;
            ImuRequestOnce();
        }
        ImuUpdate();

        if (now - last_enable_tick >= ENABLE_INTERVAL_MS)
        {
            last_enable_tick = now;
            JointMotorsEnable();
        }

        if (now - last_control_tick >= CONTROL_INTERVAL_MS)
        {
            float dt = (now - last_control_tick) * 0.001f;
            last_control_tick = now;
            UpdateControl(dt);
        }

        if (now - last_mit_tick >= MIT_SEND_INTERVAL_MS)
        {
            last_mit_tick = now;
            JointMotorsSendMIT();
        }

        WheelMotorsUpdate();

        if (now - last_telemetry_tick >= TELEMETRY_INTERVAL_MS)
        {
            last_telemetry_tick = now;
            char buffer[200];
            safe_snprintf(buffer, sizeof(buffer),
                          "BAL,mode=%u,rc=%u,imu=%u,pitch_deg=%.2f,ref_deg=%.2f,err_deg=%.2f,rate_dps=%.2f,out=%.0f,i=%.2f,cmdF=%.0f,cmdY=%.0f,wL=%.0f,wR=%.0f\r\n",
                          (unsigned)balance_mode,
                          (unsigned)ET08_IsOnline(),
                          (unsigned)imu_alive,
                          pitch_deg,
                          pitch_ref_deg,
                          balance_err_deg,
                          pitch_rate_deg_s,
                          balance_out,
                          balance_i,
                          forward_cmd,
                          yaw_cmd,
                          wheel_left_ref,
                          wheel_right_ref);
            TelemetrySend(buffer);

            const DMMotor_Feedback *fl = front_left.handle ? DMMotor_GetFeedback(front_left.handle) : NULL;
            const DMMotor_Feedback *rl = rear_left.handle ? DMMotor_GetFeedback(rear_left.handle) : NULL;
            const DMMotor_Feedback *fr = front_right.handle ? DMMotor_GetFeedback(front_right.handle) : NULL;
            const DMMotor_Feedback *rr = rear_right.handle ? DMMotor_GetFeedback(rear_right.handle) : NULL;

            float fl_pos = fl ? fl->position_rad : 0.0f;
            float rl_pos = rl ? rl->position_rad : 0.0f;
            float fr_pos = fr ? fr->position_rad : 0.0f;
            float rr_pos = rr ? rr->position_rad : 0.0f;

            float fl_tq = fl ? fl->torque : 0.0f;
            float rl_tq = rl ? rl->torque : 0.0f;
            float fr_tq = fr ? fr->torque : 0.0f;
            float rr_tq = rr ? rr->torque : 0.0f;

            uint8_t fl_err = fl ? fl->error_state : 0;
            uint8_t rl_err = rl ? rl->error_state : 0;
            uint8_t fr_err = fr ? fr->error_state : 0;
            uint8_t rr_err = rr ? rr->error_state : 0;

            char jbuf[240];
            safe_snprintf(jbuf, sizeof(jbuf),
                          "JNT,hipL=%.3f,kneeL=%.3f,hipR=%.3f,kneeR=%.3f,posFL=%.3f,posRL=%.3f,posFR=%.3f,posRR=%.3f,tqFL=%.2f,tqRL=%.2f,tqFR=%.2f,tqRR=%.2f,errFL=%u,errRL=%u,errFR=%u,errRR=%u\r\n",
                          fl_pos, (rl_pos - fl_pos),
                          fr_pos, (rr_pos - fr_pos),
                          fl_pos, rl_pos, fr_pos, rr_pos,
                          fl_tq, rl_tq, fr_tq, rr_tq,
                          (unsigned)fl_err, (unsigned)rl_err,
                          (unsigned)fr_err, (unsigned)rr_err);
            TelemetrySend(jbuf);
        }

        HAL_Delay(2);
    }
}

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

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14)
    {
        HAL_IncTick();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
