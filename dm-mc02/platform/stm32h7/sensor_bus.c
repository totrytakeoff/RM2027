#include "h7_private.h"
#include "spi.h"
#include <string.h>
static bool owned;
static uint8_t tx_buffer[64] RM_DMA_STORAGE;
static uint8_t rx_buffer[64] RM_DMA_STORAGE;
static RmStatus transfer(void *context, RmImuChip chip, const uint8_t *tx, uint8_t *rx, size_t length)
{
    (void)context;
    if (!tx || !rx || length == 0U || length > sizeof(tx_buffer) || chip > RM_IMU_GYRO) return RM_INVALID;
    RmCriticalState key = RmCritical_Enter();
    if (owned) { RmCritical_Exit(key); return RM_BUSY; }
    owned = true;
    RmCritical_Exit(key);
    memcpy(tx_buffer, tx, length);
    uint16_t pin = chip == RM_IMU_ACCEL ? GPIO_PIN_0 : GPIO_PIN_3;
    HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_RESET);
    /* Bounded polling is the first bringup path. DRDY/DMA follows after sensor validation. */
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi2, tx_buffer, rx_buffer, (uint16_t)length, 2U);
    HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_SET);
    if (result == HAL_OK) memcpy(rx, rx_buffer, length);
    key = RmCritical_Enter(); owned = false; RmCritical_Exit(key);
    return result == HAL_OK ? RM_OK : result == HAL_BUSY ? RM_BUSY : RM_ERROR;
}
static uint64_t now(void *context) { (void)context; return RmTime_NowUs(); }
static void delay(void *context, uint32_t us) { (void)context; RmTime_DelayUs(us); }
static const RmSensorBus bus = { .transfer=transfer, .now_us=now, .delay_us=delay };
void RmSensorBus_Init(void) { owned = false; }
const RmSensorBus *RmBoard_ImuBus(void) { return &bus; }
