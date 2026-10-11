#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "controller.h"

static unsigned failures;
static uint32_t fake_cycle_count;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,           \
                    #condition);                                               \
            failures++;                                                        \
        }                                                                      \
    } while (false)

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    fake_cycle_count++;
    if (cnt_last != NULL) {
        *cnt_last = fake_cycle_count;
    }
    return 0.001f;
}

static void TestResetPreservesConfiguration(void)
{
    PID_Init_Config_s config = {
        .Kp = 2.0f,
        .Ki = 3.0f,
        .Kd = 4.0f,
        .MaxOut = 100.0f,
        .DeadBand = 0.5f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 25.0f,
        .CoefA = 1.0f,
        .CoefB = 2.0f,
        .Output_LPF_RC = 0.3f,
        .Derivative_LPF_RC = 0.4f,
    };
    PIDInstance pid;

    PIDInit(&pid, &config);
    pid.Measure = 10.0f;
    pid.Last_Measure = 9.0f;
    pid.Err = 8.0f;
    pid.Last_Err = 7.0f;
    pid.Last_ITerm = 6.0f;
    pid.Pout = 5.0f;
    pid.Iout = 4.0f;
    pid.Dout = 3.0f;
    pid.ITerm = 2.0f;
    pid.Output = 1.0f;
    pid.Last_Output = -1.0f;
    pid.Last_Dout = -2.0f;
    pid.Ref = 20.0f;
    pid.dt = 0.02f;
    pid.ERRORHandler.ERRORCount = 9U;
    pid.ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;

    PIDReset(&pid);

    CHECK(pid.Kp == config.Kp);
    CHECK(pid.Ki == config.Ki);
    CHECK(pid.Kd == config.Kd);
    CHECK(pid.MaxOut == config.MaxOut);
    CHECK(pid.DeadBand == config.DeadBand);
    CHECK(pid.Improve == config.Improve);
    CHECK(pid.IntegralLimit == config.IntegralLimit);
    CHECK(pid.CoefA == config.CoefA);
    CHECK(pid.CoefB == config.CoefB);
    CHECK(pid.Output_LPF_RC == config.Output_LPF_RC);
    CHECK(pid.Derivative_LPF_RC == config.Derivative_LPF_RC);
    CHECK(pid.Measure == 0.0f);
    CHECK(pid.Last_Measure == 0.0f);
    CHECK(pid.Err == 0.0f);
    CHECK(pid.Last_Err == 0.0f);
    CHECK(pid.Last_ITerm == 0.0f);
    CHECK(pid.Pout == 0.0f);
    CHECK(pid.Iout == 0.0f);
    CHECK(pid.Dout == 0.0f);
    CHECK(pid.ITerm == 0.0f);
    CHECK(pid.Output == 0.0f);
    CHECK(pid.Last_Output == 0.0f);
    CHECK(pid.Last_Dout == 0.0f);
    CHECK(pid.Ref == 0.0f);
    CHECK(pid.dt == 0.0f);
    CHECK(pid.DWT_CNT == fake_cycle_count);
    CHECK(pid.ERRORHandler.ERRORCount == 0U);
    CHECK(pid.ERRORHandler.ERRORType == PID_ERROR_NONE);
}

static void TestNullResetIsSafe(void)
{
    PIDReset(NULL);
}

int main(void)
{
    TestResetPreservesConfiguration();
    TestNullResetIsSafe();

    if (failures != 0U) {
        fprintf(stderr, "%u pid-reset checks failed\n", failures);
        return 1;
    }

    puts("pid-reset checks passed");
    return 0;
}
