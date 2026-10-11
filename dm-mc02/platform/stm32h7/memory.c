#include "h7_private.h"
extern uint32_t __dtcm_start, __dtcm_end, __dma_start, __dma_end, __bdma_start, __bdma_end;
/* Called by reset assembly before constructors and before caches are enabled. */
void RmMemory_ClearExtraSections(void)
{
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    for (uint32_t *p = &__dtcm_start; p < &__dtcm_end; ++p) *p = 0U;
    for (uint32_t *p = &__dma_start; p < &__dma_end; ++p) *p = 0U;
    for (uint32_t *p = &__bdma_start; p < &__bdma_end; ++p) *p = 0U;
    __DSB();
}
static void region(uint32_t number, uint32_t base, uint32_t size, bool cache)
{
    MPU_Region_InitTypeDef m = {0};
    m.Enable = MPU_REGION_ENABLE;
    m.Number = number; m.BaseAddress = base; m.Size = size;
    m.AccessPermission = MPU_REGION_FULL_ACCESS;
    m.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    m.TypeExtField = MPU_TEX_LEVEL1; /* Normal memory */
    m.IsCacheable = cache ? MPU_ACCESS_CACHEABLE : MPU_ACCESS_NOT_CACHEABLE;
    m.IsBufferable = cache ? MPU_ACCESS_BUFFERABLE : MPU_ACCESS_NOT_BUFFERABLE;
    m.IsShareable = cache ? MPU_ACCESS_NOT_SHAREABLE : MPU_ACCESS_SHAREABLE;
    HAL_MPU_ConfigRegion(&m);
}
void RmMemory_EnableCaches(void)
{
    HAL_MPU_Disable();
    region(MPU_REGION_NUMBER0, 0x24000000U, MPU_REGION_SIZE_256KB, true);
    region(MPU_REGION_NUMBER1, 0x24040000U, MPU_REGION_SIZE_64KB, true);
    region(MPU_REGION_NUMBER2, 0x30000000U, MPU_REGION_SIZE_32KB, false);
    region(MPU_REGION_NUMBER3, 0x38000000U, MPU_REGION_SIZE_16KB, false);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
    SCB_EnableICache();
#if RM_ENABLE_DCACHE
    SCB_EnableDCache();
#endif
}
bool RmMemory_IsDmaBuffer(const void *data, size_t length)
{
    uintptr_t p = (uintptr_t)data;
    return length != 0U && p >= (uintptr_t)&__dma_start &&
           p < (uintptr_t)&__dma_end && length <= (uintptr_t)&__dma_end - p;
}
