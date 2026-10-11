/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Infantry ET08 minimal shoot demo
 ******************************************************************************
 * @attention
 *
 * 仅保留发射测试链路：
 * ET08(SBUS) -> SA/SB 开关 -> 摩擦轮 + 拨弹
 *
 * - SBUS 输入: USART3
 * - 调试输出: USART6
 * - 发射电机: CAN2 (friction id1/2, loader id6)
 *
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
#define TELEMETRY_RX_DUMMY 32U
#define TELEMETRY_PERIOD_MS 100U
#define VOFA_PERIOD_MS 20U
#define ET08_POS_UP 0U
#define ET08_POS_MID 1U
#define ET08_POS_DOWN 2U
#define ET08_POS_INVALID 0xFFU
#define SB_SWITCH_DEBOUNCE_MS 30U
#define LOADER_SPEED_MAX 12000.0f
#define LOADER_SPEED_MIN (-LOADER_SPEED_MAX)

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    LOADER_MODE_STOP = 0,
    LOADER_MODE_SINGLE,
    LOADER_MODE_CONTINUOUS,
} LoaderMode_t;

/* Private variables ---------------------------------------------------------*/
static ET08_Ctrl_t *et08_ctrl = NULL;
static USARTInstance *telemetry_usart = NULL;

static DJIMotorInstance *friction_l = NULL;
static DJIMotorInstance *friction_r = NULL;
static DJIMotorInstance *loader_motor = NULL;

static uint8_t shoot_enabled = 0U;
static uint8_t last_online_state = 0U;

static uint8_t friction_enabled = 0U;
static uint8_t loader_enabled = 0U;
static uint8_t loader_continuous = 0U;
static LoaderMode_t loader_mode = LOADER_MODE_STOP;
static uint8_t sb_pos = ET08_POS_MID;
static uint8_t sb_last_pos = ET08_POS_MID;
static uint32_t sb_last_change_ms = 0U;
static uint8_t loader_enabled_last = 0U;
static uint8_t sb_down_armed = 1U;
static uint8_t single_shot_active = 0U;
static uint8_t pending_shots = 0U;
static float single_shot_start_angle = 0.0f;
static uint32_t single_shot_start_ms = 0U;
static float loader_target_angle = 0.0f;
static float loader_speed_cmd = 0.0f;
static uint32_t loader_speed_tick = 0U;
static uint8_t loader_continuous_last = 0U;
static float loader_ref_last = 0.0f;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

static void Debug_DisableWatchdogs(void);
static void InitTelemetryUsart(void);
static void TelemetrySendBuffer(const uint8_t *buffer, uint16_t len);
static void TelemetrySendString(const char *str);
static void TelemetryPrintf(const char *fmt, ...);

static float ClampFloat(float value, float min_value, float max_value);
static uint8_t ET08_MapUpperSwitchPos(uint8_t state);
static uint8_t ET08_MapLowerSwitchPos(uint8_t state);

static void ShootMotorsInit(void);
static void ShootStop(void);
static bool UpdateShootByET08(void);
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
    TelemetrySendString("[sht_et08] USART6 telemetry ready\r\n");
}

static void TelemetrySendBuffer(const uint8_t *buffer, uint16_t len)
{
    if (telemetry_usart == NULL || buffer == NULL || len == 0U)
        return;

    USARTSend(telemetry_usart, (uint8_t *)buffer, len, USART_TRANSFER_BLOCKING);
}

static void TelemetrySendString(const char *str)
{
    if (str == NULL)
        return;
    TelemetrySendBuffer((const uint8_t *)str, (uint16_t)strlen(str));
}

static void TelemetryPrintf(const char *fmt, ...)
{
    if (telemetry_usart == NULL || fmt == NULL)
        return;

    char msg[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    if (n <= 0)
        return;

    uint16_t len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    TelemetrySendBuffer((const uint8_t *)msg, len);
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

static uint8_t ET08_MapUpperSwitchPos(uint8_t state)
{
    if (state == ET08_POS_INVALID || state > 5U) {
        return ET08_POS_INVALID;
    }
    return (state <= 2U) ? ET08_POS_UP : ET08_POS_DOWN;
}

static uint8_t ET08_MapLowerSwitchPos(uint8_t state)
{
    if (state == ET08_POS_INVALID || state > 5U) {
        return ET08_POS_INVALID;
    }
    return (state <= 2U) ? state : (uint8_t)(state - 3U);
}

static void ShootMotorsInit(void)
{
    Motor_Init_Config_s friction_config = {
        .motor_type = M3508,
        .can_init_config = {
            .can_handle = &FRICTION_CAN,
            .tx_id = FRICTION_LEFT_ID,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = FRICTION_SPEED_KP,
                .Ki = FRICTION_SPEED_KI,
                .Kd = FRICTION_SPEED_KD,
                .MaxOut = FRICTION_SPEED_MAX_OUT,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    friction_l = DJIMotorInit(&friction_config);
    if (friction_l) {
        DJIMotorOuterLoop(friction_l, FRICTION_INIT_LOOP);
        DJIMotorStop(friction_l);
    }

    friction_config.can_init_config.tx_id = FRICTION_RIGHT_ID;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);
    if (friction_r) {
        DJIMotorOuterLoop(friction_r, FRICTION_INIT_LOOP);
        DJIMotorStop(friction_r);
    }

    Motor_Init_Config_s loader_config = {
        .motor_type = M2006,
        .can_init_config = {
            .can_handle = &LOADER_CAN,
            .tx_id = LOADER_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = LOADER_ANGLE_KP,
                .Ki = LOADER_ANGLE_KI,
                .Kd = LOADER_ANGLE_KD,
                .MaxOut = LOADER_ANGLE_MAX_OUT,
                .IntegralLimit = 500.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
            },
            .speed_PID = {
                .Kp = LOADER_SPEED_KP,
                .Ki = LOADER_SPEED_KI,
                .Kd = LOADER_SPEED_KD,
                .MaxOut = LOADER_SPEED_MAX_OUT,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = LOADER_INIT_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    loader_motor = DJIMotorInit(&loader_config);
    if (loader_motor) {
        DJIMotorOuterLoop(loader_motor, LOADER_INIT_LOOP);
        DJIMotorStop(loader_motor);
        loader_motor->angle_feedback_sign = 1;
        loader_motor->angle_feedback_locked = 1U;
        loader_target_angle = loader_motor->measure.total_angle;
    }

    TelemetrySendString("[sht_et08] shoot motors initialized\r\n");
}

static void ShootStop(void)
{
    shoot_enabled = 0U;
    friction_enabled = 0U;
    loader_mode = LOADER_MODE_STOP;
    loader_enabled = 0U;
    loader_continuous = 0U;
    sb_pos = ET08_POS_MID;
    sb_last_pos = ET08_POS_MID;
    sb_last_change_ms = 0U;
    loader_enabled_last = 0U;
    sb_down_armed = 1U;
    single_shot_active = 0U;
    pending_shots = 0U;
    single_shot_start_angle = 0.0f;
    single_shot_start_ms = 0U;
    loader_speed_cmd = 0.0f;
    loader_speed_tick = 0U;
    loader_continuous_last = 0U;
    loader_ref_last = 0.0f;

    if (friction_l) {
        DJIMotorStop(friction_l);
    }
    if (friction_r) {
        DJIMotorStop(friction_r);
    }
    if (loader_motor) {
        DJIMotorStop(loader_motor);
    }
}

static bool UpdateShootByET08(void)
{
    uint32_t now;
    uint8_t sa_sb_state;
    uint8_t raw_state;
    uint8_t sa_pos;
    float target_speed;

    et08_ctrl = ET08_GetCtrl();
    if (friction_l == NULL || friction_r == NULL || loader_motor == NULL) {
        return false;
    }

    if (et08_ctrl == NULL || !ET08_IsOnline() || et08_ctrl->failsafe || et08_ctrl->frame_lost) {
        return false;
    }

    if (!shoot_enabled) {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader_motor);
        shoot_enabled = 1U;
    }

    now = HAL_GetTick();
    sa_sb_state = et08_ctrl->switch_sa_sb_state;
    raw_state = ET08_MapSwitchState(et08_ctrl->switch_sa_sb_raw);
    if (raw_state != ET08_POS_INVALID) {
        sa_sb_state = raw_state;
    }

    sa_pos = ET08_MapUpperSwitchPos(sa_sb_state);
    if (sa_pos == ET08_POS_UP) {
        friction_enabled = 1U;
    } else if (sa_pos == ET08_POS_DOWN) {
        friction_enabled = 0U;
    }

    sb_pos = ET08_MapLowerSwitchPos(sa_sb_state);
    if (sb_pos == ET08_POS_INVALID) {
        sb_pos = ET08_POS_MID;
    }

    loader_continuous = (friction_enabled && sb_pos == ET08_POS_UP) ? 1U : 0U;
    loader_enabled = (friction_enabled && sb_pos != ET08_POS_MID) ? 1U : 0U;

    if (!loader_enabled) {
        pending_shots = 0U;
        sb_last_pos = sb_pos;
        loader_enabled_last = 0U;
        sb_down_armed = 1U;
    } else {
        if (!loader_enabled_last && sb_pos == ET08_POS_DOWN) {
            pending_shots = 1U;
            sb_last_pos = sb_pos;
            sb_last_change_ms = now;
            loader_enabled_last = 1U;
            sb_down_armed = 0U;
        } else {
            if (sb_pos != ET08_POS_DOWN) {
                sb_down_armed = 1U;
            }

            if (sb_pos != sb_last_pos) {
                if ((now - sb_last_change_ms) >= SB_SWITCH_DEBOUNCE_MS) {
                    sb_last_change_ms = now;
                    if (sb_pos == ET08_POS_DOWN && sb_down_armed) {
                        pending_shots = 1U;
                        sb_down_armed = 0U;
                    } else if (sb_pos == ET08_POS_UP) {
                        pending_shots = 0U;
                    }
                    sb_last_pos = sb_pos;
                }
            }
            loader_enabled_last = 1U;
        }
    }

    target_speed = ClampFloat(FRICTION_TARGET_SPEED, -fabsf(FRICTION_TARGET_SPEED), fabsf(FRICTION_TARGET_SPEED));
    if (friction_enabled) {
        DJIMotorOuterLoop(friction_l, SPEED_LOOP);
        DJIMotorEnable(friction_l);
        DJIMotorSetRef(friction_l, target_speed);
        DJIMotorOuterLoop(friction_r, SPEED_LOOP);
        DJIMotorEnable(friction_r);
        DJIMotorSetRef(friction_r, target_speed);
    } else {
        DJIMotorOuterLoop(friction_l, SPEED_LOOP);
        DJIMotorStop(friction_l);
        DJIMotorOuterLoop(friction_r, SPEED_LOOP);
        DJIMotorStop(friction_r);
    }

    if (!loader_enabled) {
        DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
        DJIMotorStop(loader_motor);
        loader_mode = LOADER_MODE_STOP;
        single_shot_active = 0U;
        pending_shots = 0U;
        loader_speed_cmd = 0.0f;
        loader_speed_tick = now;
        loader_continuous_last = 0U;
        loader_ref_last = 0.0f;
        return true;
    }

    if (loader_continuous) {
        float speed_target = ClampFloat(LOADER_CONTINUOUS_SPEED, LOADER_SPEED_MIN, LOADER_SPEED_MAX);
        float dt_ms;
        float max_delta;

        pending_shots = 0U;
        single_shot_active = 0U;
        loader_mode = LOADER_MODE_CONTINUOUS;

        if (!loader_continuous_last) {
            loader_speed_cmd = 0.0f;
            loader_speed_tick = now;
        }

        if (loader_speed_tick == 0U) {
            loader_speed_tick = now;
            loader_speed_cmd = speed_target;
        } else {
            dt_ms = (float)(now - loader_speed_tick);
            loader_speed_tick = now;
            max_delta = LOADER_CONTINUOUS_SLEW_PER_MS * dt_ms;
            if (speed_target > loader_speed_cmd + max_delta) {
                loader_speed_cmd += max_delta;
            } else if (speed_target < loader_speed_cmd - max_delta) {
                loader_speed_cmd -= max_delta;
            } else {
                loader_speed_cmd = speed_target;
            }
        }

        DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
        DJIMotorEnable(loader_motor);
        DJIMotorSetRef(loader_motor, loader_speed_cmd);
        loader_ref_last = loader_speed_cmd;
        loader_continuous_last = 1U;
        return true;
    }

    loader_mode = LOADER_MODE_SINGLE;
    loader_speed_cmd = 0.0f;
    loader_speed_tick = now;
    loader_continuous_last = 0U;

    if (pending_shots > 0U && !single_shot_active) {
        loader_target_angle = loader_motor->measure.total_angle + LOADER_ANGLE_STEP;
        single_shot_start_angle = loader_motor->measure.total_angle;
        single_shot_start_ms = now;
        pending_shots -= 1U;
        single_shot_active = 1U;
    }

    if (single_shot_active) {
        float delta = fabsf(loader_motor->measure.total_angle - single_shot_start_angle);
        if (delta >= LOADER_ANGLE_STEP || (now - single_shot_start_ms) >= LOADER_SINGLE_TIMEOUT_MS) {
            single_shot_active = 0U;
            DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
            DJIMotorStop(loader_motor);
            loader_ref_last = 0.0f;
            return true;
        }

        DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
        DJIMotorEnable(loader_motor);
        DJIMotorSetRef(loader_motor, ClampFloat(LOADER_SINGLE_SPEED, LOADER_SPEED_MIN, LOADER_SPEED_MAX));
        loader_ref_last = LOADER_SINGLE_SPEED;
        return true;
    }

    DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
    DJIMotorSetRef(loader_motor, 0.0f);
    loader_ref_last = 0.0f;
    return true;
}

static void SendTelemetry(void)
{
    static uint32_t seq = 0U;
    seq++;

    uint8_t sa_sb_state = (et08_ctrl != NULL) ? et08_ctrl->switch_sa_sb_state : ET08_POS_INVALID;
    uint16_t sa_sb_raw = (et08_ctrl != NULL) ? et08_ctrl->switch_sa_sb_raw : 0U;

    TelemetryPrintf("[sht_et08] #%lu online=%u fs=%u lost=%u sa_sb(raw=%u st=%u sb=%u) friction=%u loader_en=%u cont=%u mode=%u single=%u pending=%u\r\n",
                    (unsigned long)seq,
                    (unsigned int)ET08_IsOnline(),
                    (unsigned int)((et08_ctrl != NULL) ? et08_ctrl->failsafe : 0U),
                    (unsigned int)((et08_ctrl != NULL) ? et08_ctrl->frame_lost : 0U),
                    (unsigned int)sa_sb_raw,
                    (unsigned int)sa_sb_state,
                    (unsigned int)sb_pos,
                    (unsigned int)friction_enabled,
                    (unsigned int)loader_enabled,
                    (unsigned int)loader_continuous,
                    (unsigned int)loader_mode,
                    (unsigned int)single_shot_active,
                    (unsigned int)pending_shots);

    TelemetryPrintf("[sht_et08] friction(l=%.1f r=%.1f) loader(ref=%.1f speed=%.1f angle=%.1f)\r\n",
                    (friction_l != NULL) ? friction_l->measure.speed_aps : 0.0f,
                    (friction_r != NULL) ? friction_r->measure.speed_aps : 0.0f,
                    loader_ref_last,
                    (loader_motor != NULL) ? loader_motor->measure.speed_aps : 0.0f,
                    (loader_motor != NULL) ? loader_motor->measure.total_angle : 0.0f);

    TelemetryPrintf("[sht_et08] loader(single_start=%.1f target=%.1f delta=%.1f err=%.1f speed_cmd=%.1f elapsed=%lu)\r\n",
                    single_shot_start_angle,
                    loader_target_angle,
                    (loader_motor != NULL) ? fabsf(loader_motor->measure.total_angle - single_shot_start_angle) : 0.0f,
                    (loader_motor != NULL) ? (loader_target_angle - loader_motor->measure.total_angle) : 0.0f,
                    loader_speed_cmd,
                    (unsigned long)((single_shot_active != 0U) ? (HAL_GetTick() - single_shot_start_ms) : 0U));
}

static void SendVofaFrame(void)
{
    float ch[32];
    uint8_t txbuf[sizeof(ch) + 4U];
    static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

    ch[0] = (float)HAL_GetTick();
    ch[1] = (float)ET08_IsOnline();
    ch[2] = (float)friction_enabled;
    ch[3] = (float)loader_enabled;
    ch[4] = (float)loader_continuous;
    ch[5] = (float)loader_mode;
    ch[6] = (float)single_shot_active;
    ch[7] = (float)pending_shots;
    ch[8] = (float)(et08_ctrl != NULL ? et08_ctrl->switch_sa_sb_raw : 0U);
    ch[9] = (float)(et08_ctrl != NULL ? et08_ctrl->switch_sa_sb_state : 0U);
    ch[10] = (float)sb_pos;
    ch[11] = loader_ref_last;
    ch[12] = loader_speed_cmd;
    ch[13] = loader_target_angle;
    ch[14] = (loader_motor != NULL) ? loader_motor->measure.total_angle : 0.0f;
    ch[15] = (loader_motor != NULL) ? (loader_target_angle - loader_motor->measure.total_angle) : 0.0f;
    ch[16] = single_shot_start_angle;
    ch[17] = (loader_motor != NULL) ? fabsf(loader_motor->measure.total_angle - single_shot_start_angle) : 0.0f;
    ch[18] = (float)((single_shot_active != 0U) ? (HAL_GetTick() - single_shot_start_ms) : 0U);
    ch[19] = (loader_motor != NULL) ? loader_motor->measure.speed_aps : 0.0f;
    ch[20] = (loader_motor != NULL) ? loader_motor->measure.real_current : 0.0f;
    ch[21] = (friction_l != NULL) ? friction_l->measure.speed_aps : 0.0f;
    ch[22] = (friction_r != NULL) ? friction_r->measure.speed_aps : 0.0f;
    ch[23] = FRICTION_TARGET_SPEED;
    ch[24] = LOADER_CONTINUOUS_SPEED;
    ch[25] = LOADER_SINGLE_SPEED;
    ch[26] = LOADER_ANGLE_STEP;
    ch[27] = (loader_motor != NULL) ? loader_motor->motor_controller.speed_PID.Kp : LOADER_SPEED_KP;
    ch[28] = (loader_motor != NULL) ? loader_motor->motor_controller.speed_PID.Kd : LOADER_SPEED_KD;
    ch[29] = (loader_motor != NULL) ? loader_motor->motor_controller.angle_PID.Kp : LOADER_ANGLE_KP;
    ch[30] = (loader_motor != NULL) ? loader_motor->motor_controller.angle_PID.Kd : LOADER_ANGLE_KD;
    ch[31] = (float)shoot_enabled;

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
    ShootMotorsInit();

    HAL_Delay(MOTOR_STABILIZE_TIME_MS);
    TelemetrySendString("[sht_et08] control loop start\r\n");

    uint32_t last_control_tick = DWT_GetTimeline_ms();
    uint32_t last_telemetry_tick = last_control_tick;
    uint32_t last_vofa_tick = last_control_tick;

    while (1) {
        DaemonTask();
        DJIMotorControl();

        uint32_t now = DWT_GetTimeline_ms();

        if ((now - last_control_tick) >= MAIN_LOOP_PERIOD_MS) {
            last_control_tick = now;

            bool online = UpdateShootByET08();
            if (!online) {
                ShootStop();
                if (last_online_state != 0U) {
                    TelemetrySendString("[sht_et08] remote offline/failsafe -> stop\r\n");
                    last_online_state = 0U;
                }
            } else if (last_online_state == 0U) {
                TelemetrySendString("[sht_et08] remote online -> run\r\n");
                last_online_state = 1U;
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
