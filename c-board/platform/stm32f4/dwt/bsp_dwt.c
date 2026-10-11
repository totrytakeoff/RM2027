/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @author modified by Neo with annotation
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 */

#include "bsp_dwt.h"
#include "cmsis_os.h"

static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RoundCount;
static uint32_t CYCCNT_LAST;

/**
 * @brief Extend the 32-bit CYCCNT register into a serialized 64-bit timeline.
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 */
static uint64_t DWT_GetCycleCount64(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t cnt_now;
    uint64_t cycle_count;

    __disable_irq();
    cnt_now = DWT->CYCCNT;
    if (cnt_now < CYCCNT_LAST) {
        CYCCNT_RoundCount++;
    }
    CYCCNT_LAST = cnt_now;
    cycle_count = ((uint64_t)CYCCNT_RoundCount << 32U) | cnt_now;
    __set_PRIMASK(primask);

    return cycle_count;
}

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RoundCount = 0U;
    CYCCNT_LAST = DWT->CYCCNT;
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    (void)DWT_GetCycleCount64();

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    (void)DWT_GetCycleCount64();

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    (void)DWT_GetCycleCount64();
}

float DWT_GetTimeline_s(void)
{
    return (float)DWT_GetCycleCount64() / (float)CPU_FREQ_Hz;
}

float DWT_GetTimeline_ms(void)
{
    return (float)DWT_GetCycleCount64() / (float)CPU_FREQ_Hz_ms;
}

uint64_t DWT_GetTimeline_us(void)
{
    return DWT_GetCycleCount64() / CPU_FREQ_Hz_us;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}
