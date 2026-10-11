#include "h7_private.h"
static IWDG_HandleTypeDef watchdog;
static bool running;
void RmWatchdog_EarlyStart(void)
{
    /* Register-only startup supervision: no HAL tick or initialized globals.
       Nominal 12 s at 32 kHz; runtime subsequently switches to 500 ms. */
#if defined(RM_DEBUG)
    __HAL_DBGMCU_FREEZE_IWDG1();
#else
    __HAL_DBGMCU_UnFreeze_IWDG1();
#endif
    IWDG1->KR=IWDG_KEY_ENABLE;
    IWDG1->KR=IWDG_KEY_WRITE_ACCESS_ENABLE;
    IWDG1->PR=IWDG_PRESCALER_256;
    IWDG1->RLR=1499U;
    IWDG1->WINR=IWDG_WINDOW_DISABLE;
    uint32_t budget=10000000U;
    while ((IWDG1->SR & (IWDG_SR_PVU|IWDG_SR_RVU|IWDG_SR_WVU))!=0U) {
        if (--budget==0U) RmFault_Panic(0x2008U);
    }
    IWDG1->KR=IWDG_KEY_RELOAD;
}
bool RmWatchdog_Start(void)
{
    watchdog.Instance = IWDG1;
    watchdog.Init.Prescaler = IWDG_PRESCALER_32;
    watchdog.Init.Reload = 499U; /* nominal 500 ms at 32 kHz LSI; tolerance is device dependent */
    watchdog.Init.Window = IWDG_WINDOW_DISABLE;
    running = HAL_IWDG_Init(&watchdog) == HAL_OK;
    return running;
}
void RmWatchdog_Refresh(void) { if (running) (void)HAL_IWDG_Refresh(&watchdog); }
