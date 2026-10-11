/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Wheel-leg gimbal + shooter integration demo (ET08)
 ******************************************************************************
 * @attention
 *
 * 目标：仿照 tests/firmware/infantry/infantry_shoot_test，将轮腿云台 + 发射机构合并联测，使用 ET08 遥控。
 *
 * 默认控制逻辑（与步兵一致，可按现场改宏）：
 * - CH5(开关组 SA/SB)：摩擦轮开关（state 0~2 开启；3~5 关闭）
 * - CH6(开关组 SD/SC)：预留（不参与发射逻辑）
 * - 右旋钮：拨弹允许（向负方向拧过阈值允许拨弹，回到阈值以上停止；开启即连发）
 * - 右摇杆左右：yaw 轴速度控制
 * - 右摇杆上下：pitch 轴速度控制（默认上推为抬头，幅度减半）
 *
 * 电机配置（按你的需求）：
 * - 摩擦轮：CAN2 ID 1/2 (M3508)
 * - pitch 轴 GM6020：CAN2 ID 1
 * - yaw 轴 GM6020：CAN1 ID 5
 * - 拨弹 M2006：CAN1 ID 5
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
#include <stdlib.h>
#include <math.h>

#include "bsp_init.h"
#include "bsp_log.h"
#include "daemon.h"
#include "dji_motor.h"
#include "et08_remote.h"
#include "user_lib.h"

/* Private define ------------------------------------------------------------*/
#define UPDATE_INTERVAL_MS 20U // 50Hz

// Debug telemetry (prints RC + loader state periodically via LOGINFO).
#define WL_SHOOT_DEBUG_TELEMETRY 1
#define WL_SHOOT_DEBUG_INTERVAL_MS 200U
#define WL_SHOOT_DEBUG_PRINT_CHANNELS 1
#define WL_SHOOT_DEBUG_SWITCH_STABLE 1
#define WL_SWITCH_STABLE_DIFF 8U
#define WL_SWITCH_STABLE_TIME_MS 300U

// ---------------- Shooter motors ----------------
#define FRICTION_MOTOR_COUNT 2U
static const uint8_t FRICTION_CAN_IDS[FRICTION_MOTOR_COUNT] = {1U, 2U}; // CAN2
#define LOADER_CAN_ID 5U                                              // CAN1

#define FRICTION_SPEED_MAX 30000.0f // deg/s
#define FRICTION_SPEED_MIN (-FRICTION_SPEED_MAX)
// Overall direction sign for both friction wheels (keep opposite rotation via motor_reverse_flag).
#define FRICTION_DIRECTION_SIGN (1.0f)
#define FRICTION_SPEED_TARGET (FRICTION_DIRECTION_SIGN * 30000.0f)

#define LOADER_SPEED_MAX 30000.0f
#define LOADER_SPEED_MIN (-LOADER_SPEED_MAX)
// 拨弹使用 M2006：速度环连续拨弹
// Loader direction sign for continuous mode.
#define LOADER_DIRECTION_SIGN (1.0f)
#define LOADER_CONTINUOUS_SPEED (LOADER_DIRECTION_SIGN * 15000.0f)

// Loader PID (tune here)
// - Angle loop output unit is (deg/s) as speed reference for speed loop.
// - Speed loop output is current command.
#define LOADER_ANGLE_PID_KP 8.0f
#define LOADER_ANGLE_PID_KI 0.0f
#define LOADER_ANGLE_PID_KD 0.0f
#define LOADER_ANGLE_PID_MAXOUT 6000.0f
#define LOADER_ANGLE_PID_ILIMIT 500.0f

#define LOADER_SPEED_PID_KP 15.0f
#define LOADER_SPEED_PID_KI 0.0f
#define LOADER_SPEED_PID_KD 0.0f
#define LOADER_SPEED_PID_MAXOUT 10000.0f
#define LOADER_SPEED_PID_ILIMIT 3000.0f

#define LOADER_CURRENT_PID_KP 0.6f
#define LOADER_CURRENT_PID_KI 0.0f
#define LOADER_CURRENT_PID_KD 0.0f
#define LOADER_CURRENT_PID_MAXOUT 10000.0f
#define LOADER_CURRENT_PID_ILIMIT 3000.0f

// ---------------- ET08 mapping (tunable) ----------------
// CH5 switch group (SA/SB): friction enable if state in [0,2]
#define ET08_FRICTION_ON_STATE_MIN 0U
#define ET08_FRICTION_ON_STATE_MAX 2U

// CH6 (SD/SC) reserved.

// KnobRight acts like "dial": negative direction = trigger
#define ET08_DIAL_ON_THRESHOLD (-200)
#define ET08_DIAL_OFF_THRESHOLD (-100)

// ---------------- Gimbal motors ----------------
#define GM6020_SPEED_MAX 3600.0f
#define GM6020_SPEED_MIN (-GM6020_SPEED_MAX)
#define GM6020_ANGLE_MAX 1440.0f
#define GM6020_ANGLE_MIN (-GM6020_ANGLE_MAX)
#define GM6020_SPEED_DEADZONE 30.0f

#define GIMBAL_RC_DEADZONE 50
#define YAW_SPEED_SCALE 0.10f
#define PITCH_SPEED_SCALE 0.35f
#define ET08_STICK_SCALE_DEN 660.0f // ET08 centered stick typically ~[-660,660], adjust if needed.

#define YAW_MOTOR_ID 5U   // CAN1
#define PITCH_MOTOR_ID 1U // CAN2
#define YAW_DIRECTION_SIGN (-1.0f)
#define PITCH_DIRECTION_SIGN (-1.0f)
#define YAW_REF_LPF_ALPHA 0.85f
#define YAW_SPEED_DEADZONE 60.0f

// Pitch gravity FF + release hold (copied from infantry_shoot_test defaults; tune on real gimbal)
#define PITCH_GRAVITY_FF_K -10000.0f
#define PITCH_GRAVITY_FF_MAX 20000.0f
#define PITCH_GRAVITY_FF_OFFSET_DEG 0.0f

#define PITCH_HOLD_KP 12.0f
#define PITCH_HOLD_MAX_SPEED 1200.0f
#define PITCH_HOLD_DEADBAND_DEG 0.2f
#define PITCH_HOLD_KD 0.35f
#define PITCH_HOLD_KI 0.8f
#define PITCH_HOLD_I_LIMIT 800.0f
#define PITCH_FF_LPF 0.9f
#define PITCH_ANGLE_PID_KP 10.0f
#define PITCH_ANGLE_PID_MAXOUT 1200.0f

/* Private variables ---------------------------------------------------------*/
static ET08_Ctrl_t *et08_ctrl = NULL;

static DJIMotorInstance *friction_motors[FRICTION_MOTOR_COUNT] = {NULL};
static DJIMotorInstance *loader_motor = NULL;
static DJIMotorInstance *yaw_motor = NULL;
static DJIMotorInstance *pitch_motor = NULL;

static uint8_t friction_enabled = 0;
static uint8_t dial_active = 0;
static uint8_t dial_last = 0;

static float yaw_speed_ref = 0.0f;
static float pitch_speed_ref = 0.0f;
static float pitch_hold_angle = 0.0f;
static uint8_t pitch_manual_active = 0;
static uint8_t pitch_manual_active_last = 0;
static float pitch_hold_i = 0.0f;
static float pitch_current_ff = 0.0f;
static uint32_t gimbal_last_tick = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Debug_DisableWatchdogs(void);
static void EnsureFrictionMotorsReady(void);
static void EnsureLoaderMotorReady(void);
static void ProcessRemoteControl(void);
static void UpdateFrictionControl(void);
static void UpdateLoaderControl(void);
static void GimbalMotorsInit(void);
static void ProcessGimbalControl(void);
static void UpdateGimbalControl(void);
static float ClampFloat(float value, float min, float max);
static void DebugTelemetryTick(uint32_t now);
static uint8_t ET08_IsFrictionEnabled(const ET08_Ctrl_t *rc);
static void DebugSwitchStableTick(const ET08_Ctrl_t *rc, uint32_t now);

/* Private user code ---------------------------------------------------------*/
static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

static void EnsureFrictionMotorsReady(void)
{
    for (uint8_t i = 0; i < FRICTION_MOTOR_COUNT; ++i) {
        if (friction_motors[i] != NULL) {
            continue;
        }

        Motor_Init_Config_s config = {
            .can_init_config = {
                .can_handle = &hcan2,
                .tx_id = FRICTION_CAN_IDS[i],
            },
            .controller_param_init_config = {
                .angle_PID = {
                    .Kp = 5.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .MaxOut = FRICTION_SPEED_MAX,
                    .IntegralLimit = 500.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                },
                .speed_PID = {
                    .Kp = 10.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .IntegralLimit = 3000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                               PID_Derivative_On_Measurement,
                    .MaxOut = 12000.0f,
                },
                .current_PID = {
                    .Kp = 0.5f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .IntegralLimit = 3000.0f,
                    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                               PID_Derivative_On_Measurement,
                    .MaxOut = 15000.0f,
                },
            },
            .controller_setting_init_config = {
                .angle_feedback_source = MOTOR_FEED,
                .speed_feedback_source = MOTOR_FEED,
                .outer_loop_type = SPEED_LOOP,
                .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
                .motor_reverse_flag = (i == 1U) ? MOTOR_DIRECTION_REVERSE
                                                : MOTOR_DIRECTION_NORMAL,
            },
            .motor_type = M3508,
        };

        friction_motors[i] = DJIMotorInit(&config);
    }
}

static void EnsureLoaderMotorReady(void)
{
    if (loader_motor != NULL) {
        return;
    }

    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = LOADER_CAN_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = LOADER_ANGLE_PID_KP,
                .Ki = LOADER_ANGLE_PID_KI,
                .Kd = LOADER_ANGLE_PID_KD,
                .MaxOut = LOADER_ANGLE_PID_MAXOUT,
                .IntegralLimit = LOADER_ANGLE_PID_ILIMIT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
            },
            .speed_PID = {
                .Kp = LOADER_SPEED_PID_KP,
                .Ki = LOADER_SPEED_PID_KI,
                .Kd = LOADER_SPEED_PID_KD,
                .IntegralLimit = LOADER_SPEED_PID_ILIMIT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = LOADER_SPEED_PID_MAXOUT,
            },
            .current_PID = {
                .Kp = LOADER_CURRENT_PID_KP,
                .Ki = LOADER_CURRENT_PID_KI,
                .Kd = LOADER_CURRENT_PID_KD,
                .IntegralLimit = LOADER_CURRENT_PID_ILIMIT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = LOADER_CURRENT_PID_MAXOUT,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };

    loader_motor = DJIMotorInit(&config);
}

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
    LOGINFO("[wl_shoot] Yaw GM6020 registered on CAN1 id %u", (unsigned)YAW_MOTOR_ID);

    config.controller_param_init_config.current_feedforward_ptr = &pitch_current_ff;
    config.controller_setting_init_config.feedforward_flag = CURRENT_FEEDFORWARD;
    config.controller_param_init_config.angle_PID.Kp = PITCH_ANGLE_PID_KP;
    config.controller_param_init_config.angle_PID.MaxOut = PITCH_ANGLE_PID_MAXOUT;
    config.controller_param_init_config.speed_PID.MaxOut = 30000.0f;
    config.controller_param_init_config.speed_PID.Ki = 20.0f;
    config.can_init_config.can_handle = &hcan2;
    config.can_init_config.tx_id = PITCH_MOTOR_ID;
    pitch_motor = DJIMotorInit(&config);
    LOGINFO("[wl_shoot] Pitch GM6020 registered on CAN2 id %u", (unsigned)PITCH_MOTOR_ID);

    if (yaw_motor != NULL) {
        DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
        DJIMotorStop(yaw_motor);
    }

    if (pitch_motor != NULL) {
        DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
        DJIMotorStop(pitch_motor);
    }
}

static uint8_t ET08_IsFrictionEnabled(const ET08_Ctrl_t *rc)
{
    if (rc == NULL)
        return 0;

    // Prefer discrete state if ET08 level mapping works.
    if (rc->switch_sa_sb_state != 0xFFu) {
        return (rc->switch_sa_sb_state >= ET08_FRICTION_ON_STATE_MIN &&
                rc->switch_sa_sb_state <= ET08_FRICTION_ON_STATE_MAX);
    }

    // Fallback: CH5 "on" positions are typically above center (centered > 0).
    return (rc->switch_sa_sb_centered > 0);
}

static void ProcessRemoteControl(void)
{
    if (et08_ctrl == NULL || !ET08_IsOnline()) {
        friction_enabled = 0;
        dial_active = 0;
        return;
    }

    const ET08_Ctrl_t *rc = et08_ctrl;

    friction_enabled = ET08_IsFrictionEnabled(rc);

    if (!friction_enabled) {
        dial_active = 0;
        return;
    }

    int16_t dial = rc->knob_right;
    if (!dial_active && dial < ET08_DIAL_ON_THRESHOLD) {
        dial_active = 1;
    } else if (dial_active && dial > ET08_DIAL_OFF_THRESHOLD) {
        dial_active = 0;
    }

    dial_last = dial_active;

    if (!dial_active) {
        return;
    }

}

static void DebugSwitchStableTick(const ET08_Ctrl_t *rc, uint32_t now)
{
#if !WL_SHOOT_DEBUG_SWITCH_STABLE
    (void)rc;
    (void)now;
    return;
#else
    if (rc == NULL) {
        return;
    }

    typedef struct {
        uint16_t last_raw;
        uint16_t last_reported_raw;
        uint32_t last_change_tick;
        uint8_t stable_reported;
        uint8_t initialized;
    } SwitchStableTracker_t;

    static SwitchStableTracker_t sa_sb = {0};
    static SwitchStableTracker_t sd_sc = {0};

    const uint16_t sa_raw = rc->switch_sa_sb_raw;
    const uint16_t sc_raw = rc->switch_sd_sc_raw;

    SwitchStableTracker_t *trackers[2] = {&sa_sb, &sd_sc};
    const uint16_t raws[2] = {sa_raw, sc_raw};
    const int16_t cens[2] = {rc->switch_sa_sb_centered, rc->switch_sd_sc_centered};
    const uint8_t sts[2] = {rc->switch_sa_sb_state, rc->switch_sd_sc_state};
    const char *names[2] = {"sa_sb", "sd_sc"};

    for (uint8_t i = 0; i < 2; ++i) {
        SwitchStableTracker_t *t = trackers[i];
        uint16_t raw = raws[i];

        if (!t->initialized) {
            t->initialized = 1;
            t->last_raw = raw;
            t->last_reported_raw = raw;
            t->last_change_tick = now;
            t->stable_reported = 0;
            continue;
        }

        uint16_t diff = (raw > t->last_raw) ? (raw - t->last_raw) : (t->last_raw - raw);
        if (diff > WL_SWITCH_STABLE_DIFF) {
            t->last_raw = raw;
            t->last_change_tick = now;
            t->stable_reported = 0;
            continue;
        }

        if (t->stable_reported) {
            continue;
        }

        if (now - t->last_change_tick < WL_SWITCH_STABLE_TIME_MS) {
            continue;
        }

        uint16_t report_diff = (raw > t->last_reported_raw) ? (raw - t->last_reported_raw)
                                                            : (t->last_reported_raw - raw);
        if (report_diff <= WL_SWITCH_STABLE_DIFF) {
            t->stable_reported = 1;
            continue;
        }

        t->last_reported_raw = raw;
        t->stable_reported = 1;
        LOGINFO("[wl_shoot][dbg] switch_stable %s raw=%u cen=%d st=%u",
                names[i],
                (unsigned)raw,
                (int)cens[i],
                (unsigned)sts[i]);
    }
#endif
}

static void ProcessGimbalControl(void)
{
    if (et08_ctrl == NULL || !ET08_IsOnline()) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        pitch_manual_active = 0;
        return;
    }

    const ET08_Ctrl_t *rc = et08_ctrl;

    int16_t yaw_in = rc->right.x;
    int16_t pitch_in = rc->right.y;

    uint8_t all_in_deadzone = (abs(yaw_in) < GIMBAL_RC_DEADZONE) &&
                              (abs(pitch_in) < GIMBAL_RC_DEADZONE);

    if (all_in_deadzone) {
        yaw_speed_ref = 0.0f;
        pitch_speed_ref = 0.0f;
        pitch_manual_active = 0;
        return;
    }

    float yaw_ratio = float_constrain((float)yaw_in / ET08_STICK_SCALE_DEN, -1.0f, 1.0f);
    float pitch_ratio = float_constrain((float)pitch_in / ET08_STICK_SCALE_DEN, -1.0f, 1.0f);

    float yaw_cmd = YAW_DIRECTION_SIGN * yaw_ratio * GM6020_SPEED_MAX * YAW_SPEED_SCALE;
    float pitch_cmd = PITCH_DIRECTION_SIGN * (-pitch_ratio) * GM6020_SPEED_MAX * PITCH_SPEED_SCALE;

    yaw_cmd = float_deadband(yaw_cmd, -YAW_SPEED_DEADZONE, YAW_SPEED_DEADZONE);
    pitch_cmd = float_deadband(pitch_cmd, -GM6020_SPEED_DEADZONE, GM6020_SPEED_DEADZONE);

    yaw_speed_ref = yaw_speed_ref * YAW_REF_LPF_ALPHA + yaw_cmd * (1.0f - YAW_REF_LPF_ALPHA);
    pitch_speed_ref = pitch_cmd;

    pitch_manual_active = (abs(pitch_in) >= GIMBAL_RC_DEADZONE);
}

static void UpdateFrictionControl(void)
{
    EnsureFrictionMotorsReady();

    if (!friction_enabled) {
        for (uint8_t i = 0; i < FRICTION_MOTOR_COUNT; ++i) {
            if (friction_motors[i] != NULL) {
                DJIMotorStop(friction_motors[i]);
            }
        }
        return;
    }

    float target_speed = ClampFloat(FRICTION_SPEED_TARGET, FRICTION_SPEED_MIN, FRICTION_SPEED_MAX);
    for (uint8_t i = 0; i < FRICTION_MOTOR_COUNT; ++i) {
        DJIMotorInstance *motor = friction_motors[i];
        if (motor == NULL) {
            continue;
        }
        DJIMotorOuterLoop(motor, SPEED_LOOP);
        DJIMotorEnable(motor);
        DJIMotorSetRef(motor, target_speed);
    }
}

static void UpdateLoaderControl(void)
{
    EnsureLoaderMotorReady();

    if (!friction_enabled || loader_motor == NULL) {
        if (loader_motor != NULL) {
            DJIMotorStop(loader_motor);
        }
        return;
    }

    if (!dial_active) {
        DJIMotorStop(loader_motor);
        return;
    }

    float speed_ref = ClampFloat(LOADER_CONTINUOUS_SPEED, LOADER_SPEED_MIN, LOADER_SPEED_MAX);
    DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
    DJIMotorEnable(loader_motor);
    DJIMotorSetRef(loader_motor, speed_ref);
}

static void UpdateGimbalControl(void)
{
    if (yaw_motor == NULL || pitch_motor == NULL) {
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt = 0.02f;
    if (gimbal_last_tick != 0) {
        dt = (now - gimbal_last_tick) / 1000.0f;
        if (dt <= 0.0f) {
            dt = 0.02f;
        }
    }
    dt = float_constrain(dt, 0.001f, 0.05f);
    gimbal_last_tick = now;

    yaw_speed_ref = float_constrain(yaw_speed_ref, GM6020_SPEED_MIN, GM6020_SPEED_MAX);

    float pitch_ff_raw =
        PITCH_GRAVITY_FF_K *
        sinf((pitch_motor->measure.total_angle - PITCH_GRAVITY_FF_OFFSET_DEG) * PI / 180.0f);
    pitch_ff_raw = float_constrain(pitch_ff_raw, -PITCH_GRAVITY_FF_MAX, PITCH_GRAVITY_FF_MAX);
    pitch_current_ff = pitch_current_ff * PITCH_FF_LPF + pitch_ff_raw * (1.0f - PITCH_FF_LPF);

    DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
    DJIMotorEnable(yaw_motor);
    DJIMotorSetRef(yaw_motor, yaw_speed_ref);

    if (pitch_manual_active_last && !pitch_manual_active) {
        pitch_hold_angle = pitch_motor->measure.total_angle;
        pitch_hold_i = 0.0f;
    }
    pitch_manual_active_last = pitch_manual_active;

    if (pitch_manual_active) {
        float pitch_speed_cmd = float_constrain(pitch_speed_ref, GM6020_SPEED_MIN, GM6020_SPEED_MAX);
        DJIMotorOuterLoop(pitch_motor, SPEED_LOOP);
        DJIMotorEnable(pitch_motor);
        DJIMotorSetRef(pitch_motor, pitch_speed_cmd);
    } else {
        DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
        DJIMotorEnable(pitch_motor);
        DJIMotorSetRef(pitch_motor, pitch_hold_angle);
    }
}

static void DebugTelemetryTick(uint32_t now)
{
#if !WL_SHOOT_DEBUG_TELEMETRY
    (void)now;
    return;
#else
    static uint32_t last = 0;
    if ((now - last) < WL_SHOOT_DEBUG_INTERVAL_MS) {
        return;
    }
    last = now;

    if (et08_ctrl == NULL) {
        LOGINFO("[wl_shoot][dbg] et08_ctrl=NULL");
        return;
    }

    LOGINFO("[wl_shoot][dbg] online=%u sa_sb(raw=%u st=%u) sd_sc(raw=%u st=%u cen=%d) knobR=%d friction=%u dial=%u",
            (unsigned)ET08_IsOnline(),
            (unsigned)et08_ctrl->switch_sa_sb_raw,
            (unsigned)et08_ctrl->switch_sa_sb_state,
            (unsigned)et08_ctrl->switch_sd_sc_raw,
            (unsigned)et08_ctrl->switch_sd_sc_state,
            (int)et08_ctrl->switch_sd_sc_centered,
            (int)et08_ctrl->knob_right,
            (unsigned)friction_enabled,
            (unsigned)dial_active);

#if WL_SHOOT_DEBUG_PRINT_CHANNELS
    LOGINFO("[wl_shoot][dbg] ch_raw 1..8: %u %u %u %u %u %u %u %u",
            (unsigned)et08_ctrl->raw[0],
            (unsigned)et08_ctrl->raw[1],
            (unsigned)et08_ctrl->raw[2],
            (unsigned)et08_ctrl->raw[3],
            (unsigned)et08_ctrl->raw[4],
            (unsigned)et08_ctrl->raw[5],
            (unsigned)et08_ctrl->raw[6],
            (unsigned)et08_ctrl->raw[7]);
    LOGINFO("[wl_shoot][dbg] ch_cen 1..8: %d %d %d %d %d %d %d %d",
            (int)et08_ctrl->centered[0],
            (int)et08_ctrl->centered[1],
            (int)et08_ctrl->centered[2],
            (int)et08_ctrl->centered[3],
            (int)et08_ctrl->centered[4],
            (int)et08_ctrl->centered[5],
            (int)et08_ctrl->centered[6],
            (int)et08_ctrl->centered[7]);
    LOGINFO("[wl_shoot][dbg] ch_raw 9..16: %u %u %u %u %u %u %u %u",
            (unsigned)et08_ctrl->raw_full[8],
            (unsigned)et08_ctrl->raw_full[9],
            (unsigned)et08_ctrl->raw_full[10],
            (unsigned)et08_ctrl->raw_full[11],
            (unsigned)et08_ctrl->raw_full[12],
            (unsigned)et08_ctrl->raw_full[13],
            (unsigned)et08_ctrl->raw_full[14],
            (unsigned)et08_ctrl->raw_full[15]);
    LOGINFO("[wl_shoot][dbg] ch_cen 9..16: %d %d %d %d %d %d %d %d",
            (int)et08_ctrl->centered_full[8],
            (int)et08_ctrl->centered_full[9],
            (int)et08_ctrl->centered_full[10],
            (int)et08_ctrl->centered_full[11],
            (int)et08_ctrl->centered_full[12],
            (int)et08_ctrl->centered_full[13],
            (int)et08_ctrl->centered_full[14],
            (int)et08_ctrl->centered_full[15]);
    {
    }
#endif

    if (loader_motor != NULL) {
        char ref_str[24];
        char speed_str[24];
        char angle_str[24];
        Float2Str(ref_str, loader_motor->motor_controller.pid_ref);
        Float2Str(speed_str, loader_motor->measure.speed_aps);
        Float2Str(angle_str, loader_motor->measure.total_angle);

        LOGINFO("[wl_shoot][dbg] loader stop=%u outer=%u ref=%s speed=%s current=%d angle=%s",
                (unsigned)loader_motor->stop_flag,
                (unsigned)loader_motor->motor_settings.outer_loop_type,
                ref_str,
                speed_str,
                (int)loader_motor->measure.real_current,
                angle_str);
    } else {
        LOGINFO("[wl_shoot][dbg] loader_motor=NULL");
    }
#endif
}

static float ClampFloat(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
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

    GimbalMotorsInit();

    et08_ctrl = ET08_Init(&huart3);

    LOGINFO("[wl_shoot] demo initialized");
    LOGINFO("[wl_shoot] ET08 map: SA/SB=CH%u SD/SC=CH%u knobR=CH%u right(x/y)=CH%u/%u",
            (unsigned)(ET08_MAP_SA_SB_CH + 1),
            (unsigned)(ET08_MAP_SD_SC_CH + 1),
            (unsigned)(ET08_MAP_KNOB_RIGHT_CH + 1),
            (unsigned)(ET08_MAP_RIGHT_X_CH + 1),
            (unsigned)(ET08_MAP_RIGHT_Y_CH + 1));
    LOGINFO("[wl_shoot] CH5 (SA/SB) -> friction enable (0~2 on, 3~5 off)");
    LOGINFO("[wl_shoot] CH6 (SD/SC) -> unused (reserved)");
    LOGINFO("[wl_shoot] knob_right -> loader enable (turn negative for continuous fire)");
    LOGINFO("[wl_shoot] right stick -> yaw/pitch");
    LOGINFO("[wl_shoot] CAN: friction(CAN2,id1/2,M3508), pitch(CAN2,id1,6020), yaw(CAN1,id5,6020), loader(CAN1,id5,M2006)");

    uint32_t last_update_tick = 0;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

    while (1) {
        DaemonTask();
        DJIMotorControl();

        uint32_t now = HAL_GetTick();
        if (now - last_update_tick >= UPDATE_INTERVAL_MS) {
            last_update_tick = now;

            ProcessRemoteControl();
            DebugSwitchStableTick(et08_ctrl, now);
            ProcessGimbalControl();
            UpdateFrictionControl();
            UpdateLoaderControl();
            UpdateGimbalControl();
            DebugTelemetryTick(now);
        }

        HAL_Delay(5);
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
