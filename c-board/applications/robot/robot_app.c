#include "robot_app.h"

#include <stdatomic.h>
#include <string.h>

#include "dji_motor.h"
#include "chassis.h"
#include "robot_config.h"
#include "debug.h"
#include "gimbal.h"
#include "input.h"
#include "referee.h"
#include "shoot.h"
#include "tuning_telemetry.h"
#include "ins_task.h"
#include "main.h"
#include "usart.h"
#include "vision_link.h"

Robot_Context_t g_robot;

static RmSafetyManager safety_manager;
static atomic_bool task_health_ok;
static uint8_t last_motor_health_mask;
static bool motor_health_observed;

void RobotApp_ForceSafeStop(void)
{
    DJIMotorSetGlobalOutputEnabled(false);
    Chassis_Stop();
    Gimbal_Stop();
    Shoot_Stop();
}

bool RobotApp_Init(void)
{
    static const RmSafetyPolicy safety_policy = {
        .require_explicit_rearm =
            ROBOT_SAFETY_REQUIRE_EXPLICIT_REARM != 0U,
    };
    bool input_ready;
    bool chassis_ready;
    bool gimbal_ready;
    bool shoot_ready;

    memset(&g_robot, 0, sizeof(g_robot));
    RmSafety_InitWithPolicy(&safety_manager, &safety_policy);
    atomic_init(&task_health_ok, true);
    last_motor_health_mask = 0U;
    motor_health_observed = false;
    DJIMotorSetGlobalOutputEnabled(false);

    MinimalDebug_Init();
    (void)TuningTelemetry_Init();

    if (!DJIMotorSetCommandTimeout(ROBOT_MOTOR_COMMAND_TIMEOUT_MS)) {
        MDBG_SYS("invalid motor command timeout");
        return false;
    }

    /* Preserve the proven bare-metal startup delay during migration. */
    HAL_Delay(MOTOR_STABILIZE_TIME_MS);

    input_ready = Input_Init();
    MinimalReferee_Init();
    chassis_ready = Chassis_Init();
    gimbal_ready = Gimbal_Init();
    shoot_ready = Shoot_Init();

    /*
     * 视觉链路：与裁判系统共用 USART1，由 VISION_ENABLE / REFEREE_ENABLE
     * 编译期互斥保证不会同时注册。注册失败只记日志，不阻断启动——
     * 视觉是可降级的可选链路，不应把整车拉下水。
     */
#if VISION_ENABLE
    {
        static const RmVisionLinkConfig vision_config = {
            .uart = &VISION_UART_HANDLE,
            .timeout_ms = VISION_ONLINE_TIMEOUT_MS,
            .initial_grace_ms = VISION_STARTUP_GRACE_MS,
        };

        if (!RmVisionLink_Init(&vision_config)) {
            MDBG_SYS("ERROR vision link init failed; auto-aim unavailable");
        } else {
            MDBG_SYS("vision link ready uart=%u baud=%u timeout=%ums",
                     (unsigned)VISION_UART_PORT,
                     (unsigned)VISION_BAUDRATE,
                     (unsigned)VISION_ONLINE_TIMEOUT_MS);
        }
    }
#endif

    g_robot.initialized = (input_ready && chassis_ready && gimbal_ready &&
                           shoot_ready)
                              ? 1U
                              : 0U;
    if (g_robot.initialized == 0U) {
        RobotApp_ForceSafeStop();
        MDBG_SYS("rm_robot application init failed input=%u chassis=%u gimbal=%u shoot=%u",
                 (unsigned)input_ready, (unsigned)chassis_ready,
                 (unsigned)gimbal_ready, (unsigned)shoot_ready);
        return false;
    }

    MDBG_SYS("rm_robot application initialized; safety rearm=%u motor_gate=%u",
             (unsigned)ROBOT_SAFETY_REQUIRE_EXPLICIT_REARM,
             (unsigned)ROBOT_SAFETY_GATE_ON_MOTOR_HEALTH);
    return true;
}

void RobotApp_MotorStep(void)
{
    Gimbal_MotorStep();
    DJIMotorControl();
}

void RobotApp_ControlStep(uint32_t now_ms)
{
    Input_Data_t input;
    RmSafetyInputs safety_inputs;
    bool chassis_motors_healthy;
    bool gimbal_motors_healthy;
    bool shoot_motors_healthy;
    bool all_motors_healthy;
    uint8_t motor_health_mask;

    Input_GetData(&input);
    MinimalReferee_Update();
    g_robot.referee = *MinimalReferee_GetData();

#if VISION_ENABLE
    /*
     * 自瞄模式由遥控 SB 中位选择。链路按整机模式决定是否接受目标帧，
     * 并把姿态回传给上位机。视觉不参与安全判定，因此这一段不改变
     * 下面的安全输入构造与执行顺序。
     */
    {
        attitude_t vision_attitude = {0};
        bool vision_attitude_ok = INS_Read(&vision_attitude);

        RmVisionLink_SetAutoAim(input.control_mode == ROBOT_CONTROL_AUTO_AIM);
        RmVisionLink_SetEnemyColor(ROBOT_ENEMY_COLOR);
        RmVisionLink_SetBulletSpeed((uint8_t)ROBOT_BULLET_SPEED_MPS);
        RmVisionLink_Update(
            now_ms,
            vision_attitude_ok ? vision_attitude.YawTotalAngle : 0.0f,
            vision_attitude_ok ? vision_attitude.Pitch : 0.0f,
            vision_attitude_ok ? vision_attitude.Roll : 0.0f);
    }
#endif

    safety_inputs.initialization_complete = (g_robot.initialized != 0U);
    safety_inputs.input_online = (input.online != 0U);
    safety_inputs.emergency_stop = (input.emergency_stop != 0U);
    safety_inputs.operator_enable_request =
        (input.operator_enable_request != 0U);
    safety_inputs.operator_safe_position =
        (input.operator_safe_position != 0U);
    safety_inputs.operator_arm_event = (input.operator_arm_event != 0U);
    safety_inputs.task_health_ok =
        atomic_load_explicit(&task_health_ok, memory_order_relaxed);
    chassis_motors_healthy = Chassis_IsHealthy();
    gimbal_motors_healthy = Gimbal_AreMotorsHealthy();
    shoot_motors_healthy = Shoot_IsHealthy();
    all_motors_healthy = chassis_motors_healthy &&
                         gimbal_motors_healthy &&
                         shoot_motors_healthy;
    motor_health_mask = (chassis_motors_healthy ? 1U : 0U) |
                        (gimbal_motors_healthy ? 2U : 0U) |
                        (shoot_motors_healthy ? 4U : 0U);

    if (!motor_health_observed ||
        motor_health_mask != last_motor_health_mask) {
        if (all_motors_healthy) {
            MDBG_SYS("motor health restored chassis=1 gimbal=1 shoot=1");
        } else {
            MDBG_SYS("ERROR motor offline chassis=%u gimbal=%u shoot=%u policy=report-only",
                     (unsigned)chassis_motors_healthy,
                     (unsigned)gimbal_motors_healthy,
                     (unsigned)shoot_motors_healthy);
        }
        last_motor_health_mask = motor_health_mask;
        motor_health_observed = true;
    }

    /* INS remains safety-critical; motor health is configurable per vehicle. */
    safety_inputs.device_health_ok =
        INS_IsReady() &&
        ((ROBOT_SAFETY_GATE_ON_MOTOR_HEALTH == 0U) ||
         all_motors_healthy);

    if (RmSafety_Update(&safety_manager, &safety_inputs)) {
        MDBG_SYS("safety state=%u reasons=0x%08lx",
                 (unsigned)safety_manager.state,
                 (unsigned long)safety_manager.reasons);
    }

    g_robot.input = input;
    g_robot.emergency_stop = RmSafety_OutputPermitted(&safety_manager) ? 0U : 1U;
    if (!RmSafety_OutputPermitted(&safety_manager)) {
        RobotApp_ForceSafeStop();
        return;
    }

    /* Keep the minimal baseline's execution order unchanged. */
    Gimbal_Update(&input);
    Chassis_Update(&input);
    Shoot_Update(&input);
    DJIMotorSetGlobalOutputEnabled(true);
}

void RobotApp_DiagnosticsStep(uint32_t now_ms)
{
    MinimalDebug_UpdatePeriodic(now_ms);
}

void RobotApp_TuningTelemetryStep(uint32_t now_ms)
{
    TuningTelemetry_Publish(now_ms);
}

void RobotApp_SetTaskHealth(bool healthy)
{
    atomic_store_explicit(&task_health_ok, healthy, memory_order_relaxed);
}

RmSafetyState RobotApp_GetSafetyState(void)
{
    return safety_manager.state;
}

uint32_t RobotApp_GetSafetyReasons(void)
{
    return safety_manager.reasons;
}

const Robot_Context_t *RobotApp_GetContext(void)
{
    return &g_robot;
}
