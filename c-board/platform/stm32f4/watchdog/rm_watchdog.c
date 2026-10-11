#include "rm_watchdog.h"

#include "stm32f4xx.h"

enum {
    RM_IWDG_KEY_ENABLE = 0xCCCCU,
    RM_IWDG_KEY_RELOAD = 0xAAAAU,
    RM_IWDG_KEY_WRITE_ACCESS = 0x5555U,
    RM_IWDG_PRESCALER_CODE_256 = 6U,
    RM_IWDG_PRESCALER = 256U,
    RM_IWDG_RELOAD_MAX = 0x0FFFU,
    RM_IWDG_LSI_MAX_HZ = 48000U,
    RM_IWDG_UPDATE_WAIT_ITERATIONS = 1000000U,
};

static bool watchdog_active;

static bool RmWatchdog_ComputeReload(uint32_t timeout_ms,
                                     uint16_t *reload)
{
    const uint64_t denominator =
        (uint64_t)RM_IWDG_PRESCALER * UINT64_C(1000);
    uint64_t ticks;

    if ((timeout_ms == 0U) || (reload == NULL)) {
        return false;
    }

    ticks = ((uint64_t)timeout_ms * RM_IWDG_LSI_MAX_HZ +
             denominator - 1U) /
            denominator;
    if ((ticks == 0U) || (ticks > (RM_IWDG_RELOAD_MAX + UINT64_C(1)))) {
        return false;
    }

    *reload = (uint16_t)(ticks - 1U);
    return true;
}

bool RmWatchdog_Start(uint32_t timeout_ms)
{
    uint16_t reload;
    uint32_t wait_iterations = RM_IWDG_UPDATE_WAIT_ITERATIONS;

    if (watchdog_active ||
        !RmWatchdog_ComputeReload(timeout_ms, &reload)) {
        return false;
    }

#if defined(DEBUG)
    /* A halted debugger must not create a misleading watchdog reset. */
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#endif

    IWDG->KR = RM_IWDG_KEY_ENABLE;
    watchdog_active = true;
    IWDG->KR = RM_IWDG_KEY_WRITE_ACCESS;
    IWDG->PR = RM_IWDG_PRESCALER_CODE_256;
    IWDG->RLR = reload;

    while (((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U) &&
           (wait_iterations > 0U)) {
        wait_iterations--;
    }
    if ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U) {
        return false;
    }

    IWDG->KR = RM_IWDG_KEY_RELOAD;
    return true;
}

bool RmWatchdog_Refresh(void)
{
    if (!watchdog_active) {
        return false;
    }

    IWDG->KR = RM_IWDG_KEY_RELOAD;
    return true;
}

bool RmWatchdog_IsActive(void)
{
    return watchdog_active;
}

bool RmWatchdog_WasReset(void)
{
    return (RCC->CSR & RCC_CSR_IWDGRSTF) != 0U;
}
