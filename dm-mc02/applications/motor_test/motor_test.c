#include "bringup_app.h"
#include "bringup.h"
#include "chassis_control.h"
#include "chassis_config.h"
#include "motor_test_config.h"
#include "motor_test_control.h"
#include "motor_test_device.h"
#include "output_gate.h"
#include "safety_manager.h"
#include "logging.h"
#include <stdio.h>


static RmRuntimeConfig runtime;
static RmSafetyManager safety;
static uint64_t start_us;
static void (*base_diagnostics)(uint64_t);
static float commanded_reference;
static bool command_permitted;
static MotorTestMode active_mode;
static MotorTestMode reported_mode;
static uint32_t stop_reasons;
static bool position_fault;

static void control(uint64_t now_us)
{
    RemoteControlState remote;
    RmChassisIntent intent = {0};
    RmInsSnapshot imu;
    RmDjiFeedback feedback;
    RmCanStats bus;
    bool remote_ok = RmBringup_TakeRemote(&remote) && RmChassis_MapEt08(&remote, &intent);
    MotorTestMode mode = remote_ok ? RmMotorTest_Mode(&remote) : MOTOR_TEST_OFF;
    bool imu_ok = RmBringup_ReadIns(&imu);
    bool motor_ok = RmMotorTest_DeviceRead(&feedback);
    if (!intent.enable || mode != MOTOR_TEST_POSITION) position_fault = false;
    if (mode == MOTOR_TEST_POSITION && active_mode == MOTOR_TEST_POSITION && motor_ok && !feedback.position_valid) position_fault = true;
    bool bus_ok = RmCan_GetStats(MOTOR_TEST_CAN, &bus) && bus.online;
    uint32_t now_ms = RmTime_NowMs();
    const RmSafetyInputs input = {
        .initialization_complete = imu_ok && now_us - start_us >= CHASSIS_DEMO_STARTUP_MS * 1000ULL,
        .input_online = remote_ok, .emergency_stop = !remote_ok || RmBoard_KeyPressed(),
        .operator_enable_request = intent.enable && mode != MOTOR_TEST_OFF, .operator_safe_position = intent.safe_position,
        .operator_arm_event = intent.arm_event, .task_health_ok = RmRuntime_Healthy(now_ms),
        .device_health_ok = bus_ok && motor_ok && !position_fault,
    };
    (void)RmSafety_Update(&safety, &input);
    bool permit = RmSafety_OutputPermitted(&safety);
    float reference = 0.0f;
    if (permit) {
        /* Keep the final live check and lease/command publication atomic. */
        RmCriticalState key = RmCritical_Enter();
        RemoteControlState live_remote;
        RmChassisIntent live_intent;
        now_ms = RmTime_NowMs();
        permit = RmBringup_ReadRemote(&live_remote) && RmChassis_MapEt08(&live_remote, &live_intent) &&
            live_intent.enable && RmMotorTest_Mode(&live_remote) == mode && RmBringup_ReadIns(&imu) && RmMotorTest_DeviceRead(&feedback) &&
            RmRuntime_Healthy(now_ms) && !RmBoard_KeyPressed() &&
            RmCan_GetStats(MOTOR_TEST_CAN, &bus) && bus.online;
        bool mode_changed = active_mode != mode;
        if (permit && mode_changed) {
            RmOutputGate_Stop(1U << 14);
            if (mode == MOTOR_TEST_POSITION)
                permit = RmMotorTest_DeviceZero() && RmMotorTest_DeviceRead(&feedback);
        }
        if (permit && mode == MOTOR_TEST_POSITION && !feedback.position_valid) {
            position_fault = true;
            permit = false;
        }
        if (permit) {
            reference = RmMotorTest_Reference(&live_remote, mode);
            RmOutputGateSnapshot gate;
            RmOutputGate_Read(&gate);
            uint32_t epoch = gate.permitted && RmOutputGate_Renew(gate.epoch, now_ms, RM_COMMAND_LEASE_MS) ?
                gate.epoch : RmOutputGate_Arm(now_ms, RM_COMMAND_LEASE_MS);
            const RmDjiCommand command = {
                .reference = reference,
                .epoch = epoch, .deadline_ms = now_ms + RM_COMMAND_LEASE_MS,
                .outer_loop = mode == MOTOR_TEST_POSITION ? RM_DJI_LOOP_ANGLE : RM_DJI_LOOP_SPEED,
                .reset_mask = mode_changed ? 7U : 0U, .enabled = true,
            };
            permit = RmMotorTest_DeviceSubmit(&command);
        }
        if (!permit) RmOutputGate_Stop(1U << 13);
        RmCritical_Exit(key);
    } else {
        RmOutputGate_Stop(safety.reasons ? safety.reasons : 1U << 13);
    }
    RmCriticalState key = RmCritical_Enter();
    active_mode = permit ? mode : MOTOR_TEST_OFF;
    reported_mode = mode;
    commanded_reference = permit ? reference : 0.0f;
    command_permitted = permit;
    stop_reasons = permit ? 0U : (safety.reasons ? safety.reasons : 1U << 13);
    RmCritical_Exit(key);
}

static void diagnostics(uint64_t now_us)
{
    static uint32_t next_report;
    base_diagnostics(now_us);
    uint32_t now_ms = (uint32_t)(now_us / 1000U);
    if ((int32_t)(now_ms - next_report) < 0) return;
    next_report = now_ms + 10U;
    RmDjiFeedback feedback = {0};
    bool online = RmMotorTest_DeviceRead(&feedback);
    RmCriticalState key = RmCritical_Enter();
    float reference = commanded_reference;
    MotorTestMode mode = reported_mode;
    bool enabled = command_permitted;
    uint32_t reasons = stop_reasons;
    RmCritical_Exit(key);
    char line[64];
    int length;
#if MOTOR_TEST_MODEL == MOTOR_TEST_MODEL_DM4310
    static uint32_t next_config;
    if ((int32_t)(now_ms-next_config)>=0) {
        length=snprintf(line,sizeof(line),"DC t=%lu cfg=%lu err=%lu w=%lu o=%u\r\n",
            (unsigned long)now_ms,(unsigned long)RmMotorTest_DmConfigMask(),
            (unsigned long)RmMotorTest_DmFaults(),(unsigned long)reasons,(unsigned)online);
        if (length>0 && (size_t)length<sizeof(line) &&
            RmSerial_TrySendCopy(RM_SERIAL_VISION,line,(size_t)length)==RM_OK)
            next_config=now_ms+1000;
        return;
    }
#endif
    if (mode == MOTOR_TEST_POSITION) {
        /* Position target/feedback are output shaft milliradians from zero. */
        length = snprintf(line, sizeof(line), "MP t=%lu e=%u w=%lu p=%ld a=%ld v=%d\r\n",
            (unsigned long)now_ms, (unsigned)enabled, (unsigned long)reasons,
            (long)(reference * 17.4532925199f / MOTOR_TEST_REDUCTION),
            (long)(feedback.angle_rad * 1000.0f / MOTOR_TEST_REDUCTION), (int)feedback.rpm);
    } else {
#if MOTOR_TEST_MODEL == MOTOR_TEST_MODEL_DM4310
        length = snprintf(line,sizeof(line),"DT t=%lu e=%u r=%ld v=%ld q=%d u=%ld\r\n",
            (unsigned long)now_ms,(unsigned)enabled,
            (long)(reference*1000.0f),(long)(feedback.speed_rad_s*1000.0f),(int)feedback.current_raw,
            (long)(RmMotorTest_DmTorque()*1000.0f));
#else
        length = snprintf(line, sizeof(line), "MT t=%lu e=%u w=%lu o=%u r=%ld v=%d i=%d T=%u\r\n",
            (unsigned long)now_ms, (unsigned)enabled, (unsigned long)reasons, (unsigned)online,
            (long)(reference * 9.5492965855f), (int)feedback.rpm, (int)feedback.current_raw,
            (unsigned)feedback.temperature);
#endif
    }
    if (length > 0 && (size_t)length < sizeof(line))
        (void)RmSerial_TrySendCopy(RM_SERIAL_VISION, line, (size_t)length);
}

const RmRuntimeConfig *RmMotorTest_Init(void)
{
    if (!RmLog_SetOutput(RM_SERIAL_VISION)) RmFault_Panic(0x5001U);
    runtime = *RmBringup_Init();
    base_diagnostics = runtime.tick[RM_TASK_DIAGNOSTICS];
    RmMotorTest_DeviceInit(&runtime);
    runtime.tick[RM_TASK_CONTROL] = control;
    runtime.tick[RM_TASK_DIAGNOSTICS] = diagnostics;
    const RmSafetyPolicy policy = { .require_explicit_rearm = CHASSIS_DEMO_REQUIRE_REARM };
    RmSafety_InitWithPolicy(&safety, &policy);
    start_us = RmTime_NowUs();
    return &runtime;
}
