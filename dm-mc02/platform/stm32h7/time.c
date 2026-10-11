#include "h7_private.h"
static TIM_HandleTypeDef hal_tick_timer;
static uint64_t microsecond_base;
static uint32_t cycles_per_us;
static uint32_t timer_clock(void)
{
    RCC_ClkInitTypeDef c;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&c, &latency);
    uint32_t hz = HAL_RCC_GetPCLK1Freq();
    return c.APB1CLKDivider == RCC_APB1_DIV1 ? hz : hz * 2U;
}
HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
    if (priority >= (1UL << __NVIC_PRIO_BITS)) return HAL_ERROR;
    __HAL_RCC_TIM6_CLK_ENABLE();
    TIM6->CR1 = 0U; TIM6->DIER = 0U;
    hal_tick_timer.Instance = TIM6;
    hal_tick_timer.Init.Prescaler = timer_clock() / 1000000U - 1U;
    hal_tick_timer.Init.Period = 999U;
    hal_tick_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    hal_tick_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&hal_tick_timer) != HAL_OK) return HAL_ERROR;
    __HAL_TIM_CLEAR_FLAG(&hal_tick_timer, TIM_FLAG_UPDATE);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, priority, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    uwTickPrio = priority;
    return HAL_TIM_Base_Start_IT(&hal_tick_timer);
}
void HAL_SuspendTick(void) { TIM6->DIER &= ~TIM_DIER_UIE; }
void HAL_ResumeTick(void) { TIM6->DIER |= TIM_DIER_UIE; }
void TIM6_DAC_IRQHandler(void)
{
    if ((TIM6->SR & TIM_SR_UIF) != 0U) {
        TIM6->SR = ~TIM_SR_UIF;
        HAL_IncTick();
    }
}
void RmTime_Init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->CR1 = 0U; TIM5->DIER = 0U;
    TIM5->PSC = timer_clock() / 1000000U - 1U;
    TIM5->ARR = UINT32_MAX;
    TIM5->CNT = 0U; TIM5->EGR = TIM_EGR_UG; TIM5->SR = 0U;
    microsecond_base = 0U;
    TIM5->DIER = TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->CR1 = TIM_CR1_CEN;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cycles_per_us = SystemCoreClock / 1000000U;
}
void RmTime_TimerIRQ(void)
{
    if ((TIM5->SR & TIM_SR_UIF) != 0U) {
        TIM5->SR = ~TIM_SR_UIF;
        microsecond_base += 1ULL << 32;
    }
}
uint64_t RmTime_NowUs(void)
{
    RmCriticalState key = RmCritical_Enter();
    uint64_t base = microsecond_base;
    uint32_t low = TIM5->CNT;
    if ((TIM5->SR & TIM_SR_UIF) != 0U) {
        base += 1ULL << 32;
        low = TIM5->CNT;
    }
    RmCritical_Exit(key);
    return base + low;
}
uint32_t RmTime_NowMs(void) { return (uint32_t)(RmTime_NowUs() / 1000U); }
uint32_t RmTime_Cycles(void) { return DWT->CYCCNT; }
uint32_t RmTime_CyclesToUs(uint32_t cycles) { return cycles_per_us ? cycles / cycles_per_us : 0U; }
void RmTime_DelayUs(uint32_t us)
{
    uint64_t start = RmTime_NowUs();
    while (RmTime_NowUs() - start < us) __NOP();
}
