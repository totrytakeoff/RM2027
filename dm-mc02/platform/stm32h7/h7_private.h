#ifndef H7_PRIVATE_H
#define H7_PRIVATE_H
#include "rm_platform.h"
#include "stm32h7xx_hal.h"
#define RM_DMA_STORAGE __attribute__((section(".dma_buffer"), aligned(32)))
#define RM_DTCM_STORAGE __attribute__((section(".dtcm_bss"), aligned(8)))
void RmMemory_EnableCaches(void);
bool RmMemory_IsDmaBuffer(const void *data, size_t length);
void RmTime_Init(void);
void RmTime_TimerIRQ(void);
void RmSerial_EnableDmaIRQs(void);
void RmSensorBus_Init(void);
/* Reset assembly calls this before .data/.bss initialization. */
void RmWatchdog_EarlyStart(void);
#endif
