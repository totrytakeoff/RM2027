/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Pitch compensation scan demo
 ******************************************************************************
 * @attention
 *
 * 用途:
 * - 直接给 Pitch GM6020 下发阶梯电流/力矩命令(open-loop raw current command)；
 * - 通过 USART6 以 VOFA+ JustFloat 输出“命令值 -> 角度反馈”的采样关系；
 * - 同时输出编码器总角度与 IMU Pitch / GyroX，方便比对。
 *
 * 运行方式:
 * - 上电后等待 start_delay_ms；
 * - 从 current_start 开始，每隔 hold_ms 切到下一档 current_start + direction * step * index；
 * - 可选双向扫描：正向扫完后再反向扫回；
 * - 每隔 sample_period_ms 输出一帧 JustFloat，同时在每步后输出一帧稳态统计；
 * - 所有扫描参数都是 volatile，可在调试器里在线修改。
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
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bsp_init.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "minimal_config.h"

/* Private define ------------------------------------------------------------*/
#define LOOP_DELAY_MS 1U

#define SCAN_CURRENT_START_DEFAULT 5000.0f
#define SCAN_CURRENT_STEP_DEFAULT 100.0f
#define SCAN_STEP_COUNT_DEFAULT 100U
#define SCAN_HOLD_MS_DEFAULT 500U
#define SCAN_SAMPLE_PERIOD_MS_DEFAULT 20U
#define SCAN_START_DELAY_MS_DEFAULT 2000U
#define SCAN_SETTLE_MS_DEFAULT 250U
#define SCAN_REPEAT_DEFAULT 0U
#define SCAN_BIDIRECTIONAL_DEFAULT 0U
#define SCAN_DIRECTION_DEFAULT 1
#define TELEMETRY_RX_DUMMY 32U

/* Private variables ---------------------------------------------------------*/
static DJIMotorInstance *pitch_motor = NULL;
static attitude_t *imu_data = NULL;
static USARTInstance *telemetry_usart = NULL;

static uint8_t scan_started = 0U;
static uint32_t scan_stage_start_ms = 0U;
static uint32_t last_sample_ms = 0U;
static uint32_t scan_step_index = 0U;
static float scan_current_cmd = 0.0f;
static int8_t scan_direction_sign = SCAN_DIRECTION_DEFAULT;
static uint8_t scan_pass_index = 0U;
static uint32_t steady_sample_count = 0U;
static float steady_sum_motor_angle = 0.0f;
static float steady_sum_motor_speed = 0.0f;
static float steady_sum_motor_current = 0.0f;
static float steady_sum_imu_pitch = 0.0f;
static float steady_sum_imu_gyro_x = 0.0f;

volatile uint8_t g_scan_enable = 1U;
volatile uint8_t g_scan_repeat = SCAN_REPEAT_DEFAULT;
volatile uint8_t g_scan_bidirectional = SCAN_BIDIRECTIONAL_DEFAULT;
volatile float g_scan_current_start = SCAN_CURRENT_START_DEFAULT;
volatile float g_scan_current_step = SCAN_CURRENT_STEP_DEFAULT;
volatile uint32_t g_scan_step_count = SCAN_STEP_COUNT_DEFAULT;
volatile uint32_t g_scan_hold_ms = SCAN_HOLD_MS_DEFAULT;
volatile uint32_t g_scan_sample_period_ms = SCAN_SAMPLE_PERIOD_MS_DEFAULT;
volatile uint32_t g_scan_start_delay_ms = SCAN_START_DELAY_MS_DEFAULT;
volatile uint32_t g_scan_settle_ms = SCAN_SETTLE_MS_DEFAULT;
volatile int8_t g_scan_direction = SCAN_DIRECTION_DEFAULT;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

static void Debug_DisableWatchdogs(void);
static void InitTelemetryUsart(void);
static void PitchMotorInit(void);
static void TelemetrySendBuffer(const uint8_t *buffer, uint16_t len);
static void SendVofaFrame(uint32_t now_ms, float frame_kind);
static void ScanReset(uint32_t now_ms);
static void ScanUpdate(uint32_t now_ms);
static void ScanApplyCurrent(float current_cmd);
static void ScanResetSteadyAccumulator(void);
static void ScanAccumulateSteadySample(uint32_t now_ms);
static float ScanGetStepCommand(uint32_t step_index);
static uint8_t ScanAdvanceStep(void);

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
}

static void TelemetrySendBuffer(const uint8_t *buffer, uint16_t len)
{
    if (telemetry_usart == NULL || buffer == NULL || len == 0U) {
        return;
    }
    USARTSend(telemetry_usart, (uint8_t *)buffer, len, USART_TRANSFER_BLOCKING);
}

static void SendVofaFrame(uint32_t now_ms, float frame_kind)
{
    float ch[16];
    uint8_t txbuf[sizeof(ch) + 4U];
    static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    const float motor_angle = (pitch_motor != NULL) ? pitch_motor->measure.total_angle : 0.0f;
    const float motor_speed = (pitch_motor != NULL) ? pitch_motor->measure.speed_aps : 0.0f;
    const float motor_current = (pitch_motor != NULL) ? (float)pitch_motor->measure.real_current : 0.0f;
    const float imu_pitch = (imu_data != NULL) ? imu_data->Pitch : 0.0f;
    const float imu_gyro_x = (imu_data != NULL) ? imu_data->Gyro[0] : 0.0f;
    const uint32_t stage_elapsed_ms = now_ms - scan_stage_start_ms;
    const float steady_inv = (steady_sample_count > 0U) ? (1.0f / (float)steady_sample_count) : 0.0f;

    ch[0] = (float)now_ms;
    ch[1] = (float)scan_step_index;
    ch[2] = (float)scan_pass_index;
    ch[3] = (float)scan_direction_sign;
    ch[4] = frame_kind;
    ch[5] = (float)stage_elapsed_ms;
    ch[6] = scan_current_cmd;
    ch[7] = motor_angle;
    ch[8] = motor_speed;
    ch[9] = motor_current;
    ch[10] = imu_pitch;
    ch[11] = imu_gyro_x;
    ch[12] = steady_sum_motor_angle * steady_inv;
    ch[13] = steady_sum_motor_current * steady_inv;
    ch[14] = steady_sum_imu_pitch * steady_inv;
    ch[15] = (float)steady_sample_count;

    memcpy(txbuf, ch, sizeof(ch));
    memcpy(txbuf + sizeof(ch), tail, sizeof(tail));
    TelemetrySendBuffer(txbuf, (uint16_t)sizeof(txbuf));
}

static void PitchMotorInit(void)
{
    Motor_Init_Config_s config = {
        .motor_type = GM6020,
        .can_init_config = {
            .can_handle = &PITCH_CAN,
            .tx_id = PITCH_MOTOR_ID,
        },
        .controller_setting_init_config = {
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
    };

    pitch_motor = DJIMotorInit(&config);
    if (pitch_motor != NULL) {
        DJIMotorOuterLoop(pitch_motor, OPEN_LOOP);
        DJIMotorEnable(pitch_motor);
        DJIMotorSetRef(pitch_motor, 0.0f);
    }
}

static void ScanApplyCurrent(float current_cmd)
{
    scan_current_cmd = current_cmd;
    if (pitch_motor != NULL) {
        DJIMotorEnable(pitch_motor);
        DJIMotorSetRef(pitch_motor, scan_current_cmd);
    }
}

static void ScanReset(uint32_t now_ms)
{
    scan_started = 0U;
    scan_stage_start_ms = now_ms;
    last_sample_ms = now_ms;
    scan_step_index = 0U;
    scan_pass_index = 0U;
    scan_direction_sign = (g_scan_direction >= 0) ? 1 : -1;
    ScanResetSteadyAccumulator();
    ScanApplyCurrent(0.0f);
}

static void ScanResetSteadyAccumulator(void)
{
    steady_sample_count = 0U;
    steady_sum_motor_angle = 0.0f;
    steady_sum_motor_speed = 0.0f;
    steady_sum_motor_current = 0.0f;
    steady_sum_imu_pitch = 0.0f;
    steady_sum_imu_gyro_x = 0.0f;
}

static void ScanAccumulateSteadySample(uint32_t now_ms)
{
    if ((now_ms - scan_stage_start_ms) < g_scan_settle_ms) {
        return;
    }

    steady_sample_count++;
    if (pitch_motor != NULL) {
        steady_sum_motor_angle += pitch_motor->measure.total_angle;
        steady_sum_motor_speed += pitch_motor->measure.speed_aps;
        steady_sum_motor_current += (float)pitch_motor->measure.real_current;
    }
    if (imu_data != NULL) {
        steady_sum_imu_pitch += imu_data->Pitch;
        steady_sum_imu_gyro_x += imu_data->Gyro[0];
    }
}

static float ScanGetStepCommand(uint32_t step_index)
{
    return g_scan_current_start + (float)scan_direction_sign * g_scan_current_step * (float)step_index;
}

static uint8_t ScanAdvanceStep(void)
{
    scan_step_index++;
    if (scan_step_index < g_scan_step_count) {
        return 1U;
    }

    if (g_scan_bidirectional && scan_pass_index == 0U) {
        scan_pass_index = 1U;
        scan_direction_sign = -scan_direction_sign;
        scan_step_index = 0U;
        return 1U;
    }

    if (g_scan_repeat) {
        scan_pass_index = 0U;
        scan_direction_sign = (g_scan_direction >= 0) ? 1 : -1;
        scan_step_index = 0U;
        return 1U;
    }

    return 0U;
}

static void ScanUpdate(uint32_t now_ms)
{
    if (!g_scan_enable) {
        ScanApplyCurrent(0.0f);
        scan_started = 0U;
        return;
    }

    if (!scan_started) {
        if (now_ms - scan_stage_start_ms < g_scan_start_delay_ms) {
            return;
        }
        scan_started = 1U;
        scan_stage_start_ms = now_ms;
        last_sample_ms = now_ms;
        scan_step_index = 0U;
        scan_pass_index = 0U;
        scan_direction_sign = (g_scan_direction >= 0) ? 1 : -1;
        ScanResetSteadyAccumulator();
        ScanApplyCurrent(ScanGetStepCommand(scan_step_index));
        SendVofaFrame(now_ms, 0.0f);
        return;
    }

    if (now_ms - last_sample_ms >= g_scan_sample_period_ms) {
        last_sample_ms = now_ms;
        ScanAccumulateSteadySample(now_ms);
        SendVofaFrame(now_ms, 0.0f);
    }

    if (now_ms - scan_stage_start_ms < g_scan_hold_ms) {
        return;
    }

    SendVofaFrame(now_ms, 1.0f);

    if (!ScanAdvanceStep()) {
        ScanApplyCurrent(0.0f);
        g_scan_enable = 0U;
        return;
    }

    scan_stage_start_ms = now_ms;
    last_sample_ms = now_ms;
    ScanResetSteadyAccumulator();
    ScanApplyCurrent(ScanGetStepCommand(scan_step_index));
    SendVofaFrame(now_ms, 0.0f);
}

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
    InitTelemetryUsart();
    imu_data = INS_Init();
    PitchMotorInit();

    ScanReset(HAL_GetTick());

    while (1) {
        INS_Task();
        DaemonTask();
        DJIMotorControl();
        ScanUpdate(HAL_GetTick());
        HAL_Delay(LOOP_DELAY_MS);
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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
