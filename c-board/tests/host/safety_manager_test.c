#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "safety_manager.h"

static unsigned failures;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,           \
                    #condition);                                               \
            failures++;                                                        \
        }                                                                      \
    } while (false)

static RmSafetyInputs SafeInputs(void)
{
    const RmSafetyInputs inputs = {
        .initialization_complete = true,
        .input_online = true,
        .emergency_stop = false,
        .operator_enable_request = false,
        .operator_safe_position = true,
        .operator_arm_event = false,
        .task_health_ok = true,
        .device_health_ok = true,
    };
    return inputs;
}

static void RequestArm(RmSafetyInputs *inputs)
{
    inputs->operator_enable_request = true;
    inputs->operator_arm_event = true;
}

static void ArmManager(RmSafetyManager *manager, RmSafetyInputs *inputs)
{
    CHECK(RmSafety_Update(manager, inputs));
    CHECK(manager->state == RM_SAFETY_STATE_DISARMED);
    CHECK(manager->safe_position_seen);

    RequestArm(inputs);
    CHECK(RmSafety_Update(manager, inputs));
    CHECK(manager->state == RM_SAFETY_STATE_ACTIVE);
    CHECK(manager->reasons == RM_SAFETY_REASON_NONE);
    CHECK(RmSafety_OutputPermitted(manager));
    inputs->operator_arm_event = false;
}

static void TestBootAndExplicitActivation(void)
{
    RmSafetyManager manager;
    RmSafetyInputs inputs = SafeInputs();

    RmSafety_Init(&manager);
    CHECK(manager.state == RM_SAFETY_STATE_BOOT);
    CHECK(manager.reasons == RM_SAFETY_REASON_NOT_READY);
    CHECK(manager.transition_count == 0U);
    CHECK(!RmSafety_OutputPermitted(&manager));

    ArmManager(&manager, &inputs);
    CHECK(manager.transition_count == 2U);

    CHECK(!RmSafety_Update(&manager, &inputs));
    CHECK(manager.transition_count == 2U);
}

static void TestFaultReasonsAndExplicitRecovery(void)
{
    RmSafetyManager manager;
    RmSafetyInputs inputs = SafeInputs();

    RmSafety_Init(&manager);
    ArmManager(&manager, &inputs);

    inputs.input_online = false;
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_STOPPED);
    CHECK(manager.reasons == RM_SAFETY_REASON_INPUT_OFFLINE);
    CHECK(!RmSafety_OutputPermitted(&manager));

    inputs.emergency_stop = true;
    inputs.task_health_ok = false;
    inputs.device_health_ok = false;
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.reasons == (RM_SAFETY_REASON_INPUT_OFFLINE |
                              RM_SAFETY_REASON_EMERGENCY_STOP |
                              RM_SAFETY_REASON_TASK_UNHEALTHY |
                              RM_SAFETY_REASON_DEVICE_UNHEALTHY));

    /* 故障恢复且安全门仍在上位时，禁止自动恢复输出。 */
    inputs = SafeInputs();
    RequestArm(&inputs);
    inputs.operator_arm_event = false;
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_DISARMED);
    CHECK(manager.reasons == (RM_SAFETY_REASON_OPERATOR_DISARMED |
                              RM_SAFETY_REASON_REARM_REQUIRED));
    CHECK(!RmSafety_OutputPermitted(&manager));

    /* 必须先观察安全姿态，再收到新的上拨沿。 */
    inputs = SafeInputs();
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.safe_position_seen);
    RequestArm(&inputs);
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_ACTIVE);
    CHECK(manager.reasons == RM_SAFETY_REASON_NONE);
}

static void TestNotReadyIsDisarmed(void)
{
    RmSafetyManager manager;
    RmSafetyInputs inputs = SafeInputs();

    RmSafety_Init(&manager);
    inputs.initialization_complete = false;
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_DISARMED);
    CHECK(manager.reasons == RM_SAFETY_REASON_NOT_READY);
    CHECK(!manager.safe_position_seen);
    CHECK(!RmSafety_OutputPermitted(&manager));
}

static void TestDeviceFaultRequiresFreshRearm(void)
{
    RmSafetyManager manager;
    RmSafetyInputs inputs = SafeInputs();

    RmSafety_Init(&manager);
    ArmManager(&manager, &inputs);

    inputs.device_health_ok = false;
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_STOPPED);
    CHECK(manager.reasons == RM_SAFETY_REASON_DEVICE_UNHEALTHY);
    CHECK(!RmSafety_OutputPermitted(&manager));

    inputs.device_health_ok = true;
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_DISARMED);
    CHECK((manager.reasons & RM_SAFETY_REASON_REARM_REQUIRED) != 0U);
    CHECK(!RmSafety_OutputPermitted(&manager));
}

static void TestOperatorDisarmIsImmediate(void)
{
    RmSafetyManager manager;
    RmSafetyInputs inputs = SafeInputs();

    RmSafety_Init(&manager);
    ArmManager(&manager, &inputs);

    inputs = SafeInputs();
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_DISARMED);
    CHECK(manager.reasons == RM_SAFETY_REASON_OPERATOR_DISARMED);
    CHECK(!RmSafety_OutputPermitted(&manager));
    CHECK(manager.safe_position_seen);
}

static void TestUnsafePostureInvalidatesQualification(void)
{
    RmSafetyManager manager;
    RmSafetyInputs inputs = SafeInputs();

    RmSafety_Init(&manager);
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.safe_position_seen);

    /* 安全门仍关闭时移动模式、扳机或摇杆，旧资格必须立即失效。 */
    inputs.operator_safe_position = false;
    (void)RmSafety_Update(&manager, &inputs);
    CHECK(!manager.safe_position_seen);

    RequestArm(&inputs);
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_DISARMED);
    CHECK((manager.reasons & RM_SAFETY_REASON_REARM_REQUIRED) != 0U);
    CHECK(!RmSafety_OutputPermitted(&manager));
}

static void TestArmRequiresCurrentSafePosture(void)
{
    RmSafetyManager manager;
    RmSafetyInputs inputs = SafeInputs();

    RmSafety_Init(&manager);
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.safe_position_seen);

    /* 同一快照中上拨安全门并带入不安全控制，禁止使用旧安全快照。 */
    inputs.operator_safe_position = false;
    RequestArm(&inputs);
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_DISARMED);
    CHECK(!manager.safe_position_seen);
    CHECK(!RmSafety_OutputPermitted(&manager));

    /* 未在安全门关闭时观察过安全姿态，也不能直接从启动状态解锁。 */
    RmSafety_Init(&manager);
    inputs = SafeInputs();
    RequestArm(&inputs);
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_DISARMED);
    CHECK((manager.reasons & RM_SAFETY_REASON_REARM_REQUIRED) != 0U);
    CHECK(!RmSafety_OutputPermitted(&manager));
}

static void TestNullArgumentsRemainSafe(void)
{
    RmSafetyManager manager;
    RmSafetyInputs inputs = SafeInputs();

    RmSafety_Init(NULL);
    RmSafety_Init(&manager);
    CHECK(!RmSafety_Update(NULL, &inputs));
    ArmManager(&manager, &inputs);
    CHECK(RmSafety_OutputPermitted(&manager));
    CHECK(RmSafety_Update(&manager, NULL));
    CHECK(manager.state == RM_SAFETY_STATE_STOPPED);
    CHECK(manager.reasons == RM_SAFETY_REASON_INVALID_INPUT);
    CHECK(!manager.safe_position_seen);
    CHECK(!RmSafety_OutputPermitted(&manager));
    CHECK(!RmSafety_Update(&manager, NULL));
    CHECK(!RmSafety_OutputPermitted(NULL));
}

static void TestLevelGatePolicyAndReportOnlyDeviceHealth(void)
{
    const RmSafetyPolicy policy = {
        .require_explicit_rearm = false,
    };
    RmSafetyManager manager;
    RmSafetyInputs inputs = SafeInputs();

    RmSafety_InitWithPolicy(&manager, &policy);
    inputs.operator_enable_request = true;
    inputs.operator_safe_position = false;
    inputs.operator_arm_event = false;

    /* The application chooses which devices are safety-critical. */
    inputs.device_health_ok = true;
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_ACTIVE);
    CHECK(manager.reasons == RM_SAFETY_REASON_NONE);
    CHECK(RmSafety_OutputPermitted(&manager));

    inputs.operator_enable_request = false;
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_DISARMED);
    CHECK(manager.reasons == RM_SAFETY_REASON_OPERATOR_DISARMED);

    /* Re-enabling is level-driven; no edge or neutral posture is required. */
    inputs.operator_enable_request = true;
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_ACTIVE);

    inputs.input_online = false;
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_STOPPED);
    CHECK(manager.reasons == RM_SAFETY_REASON_INPUT_OFFLINE);

    inputs.input_online = true;
    CHECK(RmSafety_Update(&manager, &inputs));
    CHECK(manager.state == RM_SAFETY_STATE_ACTIVE);
}

int main(void)
{
    TestBootAndExplicitActivation();
    TestFaultReasonsAndExplicitRecovery();
    TestNotReadyIsDisarmed();
    TestDeviceFaultRequiresFreshRearm();
    TestOperatorDisarmIsImmediate();
    TestUnsafePostureInvalidatesQualification();
    TestArmRequiresCurrentSafePosture();
    TestNullArgumentsRemainSafe();
    TestLevelGatePolicyAndReportOnlyDeviceHealth();

    if (failures != 0U) {
        fprintf(stderr, "%u safety-manager checks failed\n", failures);
        return 1;
    }

    puts("safety-manager checks passed");
    return 0;
}
