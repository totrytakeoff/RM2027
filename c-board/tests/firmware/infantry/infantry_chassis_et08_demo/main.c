/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Infantry ET08 chassis-only demo
 ******************************************************************************
 * @attention
 *
 * 目的:
 * - 回退到纯底盘最小链路
 * - 排除云台/IMU/坐标变换带来的干扰
 * - 仅验证 ET08 输入、十字全向轮逆解和电机物理映射
 *
 * 控制定义:
 * - 左摇杆 X: 横移, 左为正
 * - 左摇杆 Y: 前后, 前为正
 * - 右摇杆 X: 底盘自转
 * - 右摇杆 Y: 未使用
 *
 * 固定轮组物理布局(以前向看):
 *   2 3
 *   1 4
 *
 * 即:
 * - FL = 2
 * - FR = 3
 * - BL = 1
 * - BR = 4
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
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dji_motor.h"
#include "et08_remote.h"
#include "minimal_config.h"

/* Private define ------------------------------------------------------------*/
#define CHASSIS_MOTOR_COUNT 4U
#define TELEMETRY_RX_DUMMY 32U
#define TELEMETRY_PERIOD_MS 100U
#define VOFA_PERIOD_MS 20U
#define DAEMON_TASK_PERIOD_MS 1U
#define MOTOR_CONTROL_PERIOD_MS 2U
#define RC_INPUT_DEADZONE 50
#define WHEEL_SPEED_LIMIT_DPS 30000.0f
#define DEMO_RC_MAPPING_MODE 0U

#if DEMO_RC_MAPPING_MODE == 0
#define ET08_LEFT_X_CH ET08_CH4
#define ET08_LEFT_Y_CH ET08_CH3
#define ET08_RIGHT_X_CH ET08_CH1
#define ET08_RIGHT_Y_CH ET08_CH2
#else
#define ET08_LEFT_X_CH ET08_CH1
#define ET08_LEFT_Y_CH ET08_CH2
#define ET08_RIGHT_X_CH ET08_CH4
#define ET08_RIGHT_Y_CH ET08_CH3
#endif

/*
 * 固定采用用户现场确认的物理布局:
 *   2 3
 *   1 4
 *
 * 逆解输出顺序为 FR, FL, BR, BL,
 * 因此下发 ID 需要映射为:
 * - FR -> 3
 * - FL -> 2
 * - BR -> 4
 * - BL -> 1
 */
#define CHASSIS_MOTOR_ID_FR 3U
#define CHASSIS_MOTOR_ID_FL 2U
#define CHASSIS_MOTOR_ID_BR 4U
#define CHASSIS_MOTOR_ID_BL 1U

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    float vx_cmd;
    float vy_cmd;
    float wz_cmd;
} ChassisCmd_t;

/* Private variables ---------------------------------------------------------*/
static ET08_Ctrl_t *et08_ctrl = NULL;
static USARTInstance *telemetry_usart = NULL;

static DJIMotorInstance *motor_fr = NULL;
static DJIMotorInstance *motor_fl = NULL;
static DJIMotorInstance *motor_br = NULL;
static DJIMotorInstance *motor_bl = NULL;

static uint8_t chassis_enabled = 0U;
static uint8_t last_online_state = 0U;
static float wheel_speed_ref[CHASSIS_MOTOR_COUNT] = {0.0f};
static ChassisCmd_t last_cmd = {0.0f};
static float filtered_vx = 0.0f;
static float filtered_vy = 0.0f;
static float filtered_wz = 0.0f;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

static void Debug_DisableWatchdogs(void);
static void InitTelemetryUsart(void);
static void TelemetrySendBuffer(const uint8_t *buffer, uint16_t len);
static void TelemetrySendString(const char *str);
static void TelemetryPrintf(const char *fmt, ...);

static int16_t ApplyDeadzone(int16_t value, int16_t deadzone);
static float ClampFloat(float value, float min_value, float max_value);
static void OmniInverseKinematics(float vx, float vy, float wz, float out[4]);

static void ChassisMotorsInit(void);
static bool BuildChassisCommandFromEt08(ChassisCmd_t *cmd);
static void ChassisStop(void);
static void ChassisApplyCommand(const ChassisCmd_t *cmd);
static void SendTelemetry(void);
static void SendVofaFrame(void);

/* Private user code ---------------------------------------------------------*/
static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

static void InitTelemetryUsart(void)
{
    USART_Init_Config_s config = {
        .module_callback = NULL,
        .recv_buff_size = TELEMETRY_RX_DUMMY,
        .usart_handle = &huart6,
    };
    telemetry_usart = USARTRegister(&config);
    TelemetrySendString("[chs_et08] USART6 telemetry ready\r\n");
}

static void TelemetrySendBuffer(const uint8_t *buffer, uint16_t len)
{
    if (telemetry_usart == NULL || buffer == NULL || len == 0U) {
        return;
    }

    USARTSend(telemetry_usart, (uint8_t *)buffer, len, USART_TRANSFER_BLOCKING);
}

static void TelemetrySendString(const char *str)
{
    if (str == NULL) {
        return;
    }
    TelemetrySendBuffer((const uint8_t *)str, (uint16_t)strlen(str));
}

static void TelemetryPrintf(const char *fmt, ...)
{
    if (telemetry_usart == NULL || fmt == NULL) {
        return;
    }

    char msg[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    if (n <= 0) {
        return;
    }

    TelemetrySendBuffer((const uint8_t *)msg,
                        (uint16_t)((n < (int)sizeof(msg)) ? n : (int)(sizeof(msg) - 1U)));
}

static int16_t ApplyDeadzone(int16_t value, int16_t deadzone)
{
    if (value > -deadzone && value < deadzone) {
        return 0;
    }
    return value;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void OmniInverseKinematics(float vx, float vy, float wz, float out[4])
{
    const float half_base = CHASSIS_WHEEL_BASE * 0.5f;
    /*
     * 当前这台车的实测现象:
     * - 旧公式中的 [+,+,-,-] 轮速模式会产生左平移
     * - 旧公式中的 [-,+,+,-] 轮速模式会打架,不是有效平移轴
     * - 纯自转项符号正确,保持不变
     *
     * 因此对这台车重建平移基向量:
     * - vx(左为正)  -> FR/FL/BR/BL = [+,+,-,-]
     * - vy(前为正)  -> FR/FL/BR/BL = [-,+,-,+]
     * - wz(CCW为正) -> 四轮同号
     */
    const float v_fr = vx - vy - half_base * wz;
    const float v_fl = vx + vy - half_base * wz;
    const float v_br = -vx - vy - half_base * wz;
    const float v_bl = -vx + vy - half_base * wz;

    out[0] = v_fr / CHASSIS_WHEEL_RADIUS;
    out[1] = v_fl / CHASSIS_WHEEL_RADIUS;
    out[2] = v_br / CHASSIS_WHEEL_RADIUS;
    out[3] = v_bl / CHASSIS_WHEEL_RADIUS;
}

static void ChassisMotorsInit(void)
{
    Motor_Init_Config_s config = {
        .motor_type = M3508,
        .can_init_config = {
            .can_handle = &CHASSIS_CAN,
            .tx_id = CHASSIS_MOTOR_ID_FR,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = CHASSIS_SPEED_KP,
                .Ki = CHASSIS_SPEED_KI,
                .Kd = CHASSIS_SPEED_KD,
                .MaxOut = CHASSIS_SPEED_MAX_OUT,
                .IntegralLimit = 1000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
            },
            .current_PID = {
                .Kp = 0.35f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 1000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .MaxOut = 8000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    motor_fr = DJIMotorInit(&config);
    if (motor_fr != NULL) {
        DJIMotorOuterLoop(motor_fr, SPEED_LOOP);
        DJIMotorStop(motor_fr);
    }

    config.can_init_config.tx_id = CHASSIS_MOTOR_ID_FL;
    motor_fl = DJIMotorInit(&config);
    if (motor_fl != NULL) {
        DJIMotorOuterLoop(motor_fl, SPEED_LOOP);
        DJIMotorStop(motor_fl);
    }

    config.can_init_config.tx_id = CHASSIS_MOTOR_ID_BR;
    motor_br = DJIMotorInit(&config);
    if (motor_br != NULL) {
        DJIMotorOuterLoop(motor_br, SPEED_LOOP);
        DJIMotorStop(motor_br);
    }

    config.can_init_config.tx_id = CHASSIS_MOTOR_ID_BL;
    motor_bl = DJIMotorInit(&config);
    if (motor_bl != NULL) {
        DJIMotorOuterLoop(motor_bl, SPEED_LOOP);
        DJIMotorStop(motor_bl);
    }

    TelemetrySendString("[chs_et08] chassis motors initialized\r\n");
}

static bool BuildChassisCommandFromEt08(ChassisCmd_t *cmd)
{
    int16_t left_x;
    int16_t left_y;
    int16_t right_x;

    if (cmd == NULL) {
        return false;
    }

    et08_ctrl = ET08_GetCtrl();
    if (et08_ctrl == NULL || !ET08_IsOnline() || et08_ctrl->failsafe || et08_ctrl->frame_lost) {
        return false;
    }

    left_x = ApplyDeadzone(et08_ctrl->left.x, RC_INPUT_DEADZONE);
    left_y = ApplyDeadzone(et08_ctrl->left.y, RC_INPUT_DEADZONE);
    right_x = ApplyDeadzone(et08_ctrl->right.x, RC_INPUT_DEADZONE);

    cmd->vx_cmd = (-(float)left_x / RC_STICK_SCALE) * CHASSIS_MAX_VX;
    cmd->vy_cmd = ((float)left_y / RC_STICK_SCALE) * CHASSIS_MAX_VY;
    cmd->wz_cmd = ((float)right_x / RC_STICK_SCALE) * CHASSIS_MAX_WZ;

    cmd->vx_cmd = ClampFloat(cmd->vx_cmd, -CHASSIS_MAX_VX, CHASSIS_MAX_VX);
    cmd->vy_cmd = ClampFloat(cmd->vy_cmd, -CHASSIS_MAX_VY, CHASSIS_MAX_VY);
    cmd->wz_cmd = ClampFloat(cmd->wz_cmd, -CHASSIS_MAX_WZ, CHASSIS_MAX_WZ);
    return true;
}

static void ChassisStop(void)
{
    chassis_enabled = 0U;
    filtered_vx = 0.0f;
    filtered_vy = 0.0f;
    filtered_wz = 0.0f;
    last_cmd.vx_cmd = 0.0f;
    last_cmd.vy_cmd = 0.0f;
    last_cmd.wz_cmd = 0.0f;
    memset(wheel_speed_ref, 0, sizeof(wheel_speed_ref));

    if (motor_fr != NULL) {
        DJIMotorStop(motor_fr);
    }
    if (motor_fl != NULL) {
        DJIMotorStop(motor_fl);
    }
    if (motor_br != NULL) {
        DJIMotorStop(motor_br);
    }
    if (motor_bl != NULL) {
        DJIMotorStop(motor_bl);
    }
}

static void ChassisApplyCommand(const ChassisCmd_t *cmd)
{
    float wheel_speed_rad[CHASSIS_MOTOR_COUNT];

    if (cmd == NULL) {
        return;
    }

    last_cmd = *cmd;

    if (fabsf(cmd->vx_cmd) < CHASSIS_DEADZONE_VX &&
        fabsf(cmd->vy_cmd) < CHASSIS_DEADZONE_VY &&
        fabsf(cmd->wz_cmd) < CHASSIS_DEADZONE_WZ) {
        filtered_vx = 0.0f;
        filtered_vy = 0.0f;
        filtered_wz = 0.0f;
    } else {
        filtered_vx = filtered_vx * CHASSIS_SPEED_FILTER_COEF +
                      cmd->vx_cmd * (1.0f - CHASSIS_SPEED_FILTER_COEF);
        filtered_vy = filtered_vy * CHASSIS_SPEED_FILTER_COEF +
                      cmd->vy_cmd * (1.0f - CHASSIS_SPEED_FILTER_COEF);
        filtered_wz = filtered_wz * CHASSIS_SPEED_FILTER_COEF +
                      cmd->wz_cmd * (1.0f - CHASSIS_SPEED_FILTER_COEF);
    }

    OmniInverseKinematics(filtered_vx, filtered_vy, filtered_wz, wheel_speed_rad);

    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; ++i) {
        float speed_dps = wheel_speed_rad[i] * 180.0f / PI;
        speed_dps *= CHASSIS_SPEED_SCALE;

        if (fabsf(speed_dps) < CHASSIS_SPEED_DEADZONE) {
            speed_dps = 0.0f;
        }
        wheel_speed_ref[i] = ClampFloat(speed_dps, -WHEEL_SPEED_LIMIT_DPS, WHEEL_SPEED_LIMIT_DPS);
    }

    if (!chassis_enabled) {
        if (motor_fr != NULL) {
            DJIMotorEnable(motor_fr);
        }
        if (motor_fl != NULL) {
            DJIMotorEnable(motor_fl);
        }
        if (motor_br != NULL) {
            DJIMotorEnable(motor_br);
        }
        if (motor_bl != NULL) {
            DJIMotorEnable(motor_bl);
        }
        chassis_enabled = 1U;
    }

    if (motor_fr != NULL) {
        DJIMotorSetRef(motor_fr, wheel_speed_ref[0]);
    }
    if (motor_fl != NULL) {
        DJIMotorSetRef(motor_fl, wheel_speed_ref[1]);
    }
    if (motor_br != NULL) {
        DJIMotorSetRef(motor_br, wheel_speed_ref[2]);
    }
    if (motor_bl != NULL) {
        DJIMotorSetRef(motor_bl, wheel_speed_ref[3]);
    }
}

static void SendTelemetry(void)
{
    static uint32_t seq = 0U;
    seq++;

    TelemetryPrintf("[chs_et08] #%lu online=%u raw(lx=%d ly=%d rx=%d ry=%d)\r\n",
                    (unsigned long)seq,
                    (unsigned int)ET08_IsOnline(),
                    (int)((et08_ctrl != NULL) ? et08_ctrl->left.x : 0),
                    (int)((et08_ctrl != NULL) ? et08_ctrl->left.y : 0),
                    (int)((et08_ctrl != NULL) ? et08_ctrl->right.x : 0),
                    (int)((et08_ctrl != NULL) ? et08_ctrl->right.y : 0));

    TelemetryPrintf("[chs_et08] cmd(vx=%.2f vy=%.2f wz=%.2f) filt(vx=%.2f vy=%.2f wz=%.2f)\r\n",
                    last_cmd.vx_cmd, last_cmd.vy_cmd, last_cmd.wz_cmd,
                    filtered_vx, filtered_vy, filtered_wz);

    TelemetryPrintf("[chs_et08] wheel_ref(fr=%.1f fl=%.1f br=%.1f bl=%.1f) fdb(fr=%.1f fl=%.1f br=%.1f bl=%.1f)\r\n",
                    wheel_speed_ref[0], wheel_speed_ref[1], wheel_speed_ref[2], wheel_speed_ref[3],
                    (motor_fr != NULL) ? motor_fr->measure.speed_aps : 0.0f,
                    (motor_fl != NULL) ? motor_fl->measure.speed_aps : 0.0f,
                    (motor_br != NULL) ? motor_br->measure.speed_aps : 0.0f,
                    (motor_bl != NULL) ? motor_bl->measure.speed_aps : 0.0f);
}

static void SendVofaFrame(void)
{
    float ch[16];
    uint8_t txbuf[sizeof(ch) + 4U];
    static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

    ch[0] = (float)HAL_GetTick();
    ch[1] = (float)ET08_IsOnline();
    ch[2] = (float)((et08_ctrl != NULL) ? et08_ctrl->left.x : 0);
    ch[3] = (float)((et08_ctrl != NULL) ? et08_ctrl->left.y : 0);
    ch[4] = (float)((et08_ctrl != NULL) ? et08_ctrl->right.x : 0);
    ch[5] = last_cmd.vx_cmd;
    ch[6] = last_cmd.vy_cmd;
    ch[7] = last_cmd.wz_cmd;
    ch[8] = filtered_vx;
    ch[9] = filtered_vy;
    ch[10] = filtered_wz;
    ch[11] = wheel_speed_ref[0];
    ch[12] = wheel_speed_ref[1];
    ch[13] = wheel_speed_ref[2];
    ch[14] = wheel_speed_ref[3];
    ch[15] = (float)chassis_enabled;

    memcpy(txbuf, ch, sizeof(ch));
    memcpy(txbuf + sizeof(ch), tail, sizeof(tail));
    TelemetrySendBuffer(txbuf, (uint16_t)sizeof(txbuf));
}

/* Main ----------------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
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

    Debug_DisableWatchdogs();
    DWT_Init(168);
    BSPLogInit();
    InitTelemetryUsart();

    et08_ctrl = ET08_Init(&RC_UART);
    ChassisMotorsInit();

    HAL_Delay(MOTOR_STABILIZE_TIME_MS);
    TelemetrySendString("[chs_et08] control loop start\r\n");

    {
        uint32_t last_control_tick = DWT_GetTimeline_ms();
        uint32_t last_telemetry_tick = last_control_tick;
        uint32_t last_vofa_tick = last_control_tick;
        uint32_t last_daemon_tick = last_control_tick;
        uint32_t last_motor_tick = last_control_tick;

        while (1) {
            uint32_t now = DWT_GetTimeline_ms();

            if ((now - last_daemon_tick) >= DAEMON_TASK_PERIOD_MS) {
                last_daemon_tick = now;
                DaemonTask();
            }
            if ((now - last_motor_tick) >= MOTOR_CONTROL_PERIOD_MS) {
                last_motor_tick = now;
                DJIMotorControl();
            }

            if ((now - last_control_tick) >= MAIN_LOOP_PERIOD_MS) {
                ChassisCmd_t cmd = {0.0f};
                last_control_tick = now;

                if (!BuildChassisCommandFromEt08(&cmd)) {
                    ChassisStop();
                    if (last_online_state != 0U) {
                        TelemetrySendString("[chs_et08] remote offline/failsafe -> stop\r\n");
                        last_online_state = 0U;
                    }
                } else {
                    ChassisApplyCommand(&cmd);
                    if (last_online_state == 0U) {
                        TelemetrySendString("[chs_et08] remote online -> run\r\n");
                        last_online_state = 1U;
                    }
                }
            }

            if ((now - last_telemetry_tick) >= TELEMETRY_PERIOD_MS) {
                last_telemetry_tick = now;
                SendTelemetry();
            }
            if ((now - last_vofa_tick) >= VOFA_PERIOD_MS) {
                last_vofa_tick = now;
                SendVofaFrame();
            }

            HAL_Delay(1);
        }
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

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14) {
        HAL_IncTick();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
