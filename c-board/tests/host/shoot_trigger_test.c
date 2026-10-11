#include <stdbool.h>
#include <stdio.h>

#include "shoot_trigger.h"

static unsigned failures;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,          \
                    #condition);                                               \
            failures++;                                                        \
        }                                                                      \
    } while (false)

static void TestOneActivationPerAssertedLevel(void)
{
    SingleShotTrigger trigger;

    SingleShotTrigger_Init(&trigger);
    CHECK(!SingleShotTrigger_Update(&trigger, true, false));
    CHECK(SingleShotTrigger_Update(&trigger, true, true));
    CHECK(trigger.consumed == 1U);
    CHECK(trigger.activation_count == 1U);
    CHECK(!SingleShotTrigger_Update(&trigger, true, true));
    CHECK(trigger.activation_count == 1U);

    CHECK(!SingleShotTrigger_Update(&trigger, true, false));
    CHECK(trigger.consumed == 0U);
    CHECK(SingleShotTrigger_Update(&trigger, true, true));
    CHECK(trigger.activation_count == 2U);
}

static void TestEnteringSingleModeWithTriggerHeld(void)
{
    SingleShotTrigger trigger;

    SingleShotTrigger_Init(&trigger);
    CHECK(!SingleShotTrigger_Update(&trigger, false, true));
    CHECK(trigger.consumed == 0U);
    CHECK(SingleShotTrigger_Update(&trigger, true, true));
    CHECK(!SingleShotTrigger_Update(&trigger, true, true));

    CHECK(!SingleShotTrigger_Update(&trigger, false, true));
    CHECK(SingleShotTrigger_Update(&trigger, true, true));
    CHECK(trigger.activation_count == 2U);
}

static void TestNullStateFailsClosed(void)
{
    SingleShotTrigger_Init(NULL);
    CHECK(!SingleShotTrigger_Update(NULL, true, true));
}

int main(void)
{
    TestOneActivationPerAssertedLevel();
    TestEnteringSingleModeWithTriggerHeld();
    TestNullStateFailsClosed();

    if (failures != 0U) {
        fprintf(stderr, "%u single-trigger checks failed\n", failures);
        return 1;
    }
    puts("shoot_trigger_test: PASS");
    return 0;
}
