#include "rm_time.h"

#include "bsp_dwt.h"
#include "stm32f4xx_hal.h"

uint32_t RmTime_NowMs(void)
{
    return HAL_GetTick();
}

uint64_t RmTime_NowUs(void)
{
    return DWT_GetTimeline_us();
}
