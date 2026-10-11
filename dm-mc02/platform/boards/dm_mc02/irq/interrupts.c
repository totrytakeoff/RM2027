#include "h7_private.h"
#include "usart.h"
#include "spi.h"
#include "fdcan.h"
/* SysTick, SVC and PendSV are exclusively provided by the selected RTOS port. */
void TIM5_IRQHandler(void) { RmTime_TimerIRQ(); }
void UART5_IRQHandler(void) { HAL_UART_IRQHandler(&huart5); }
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void UART7_IRQHandler(void) { HAL_UART_IRQHandler(&huart7); }
void USART10_IRQHandler(void) { HAL_UART_IRQHandler(&huart10); }
void DMA1_Stream4_IRQHandler(void) { HAL_DMA_IRQHandler(huart5.hdmarx); }
void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(huart1.hdmarx); }
void DMA1_Stream6_IRQHandler(void) { HAL_DMA_IRQHandler(huart1.hdmatx); }
void DMA1_Stream7_IRQHandler(void) { HAL_DMA_IRQHandler(huart7.hdmarx); }
void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(huart7.hdmatx); }
void DMA2_Stream1_IRQHandler(void) { HAL_DMA_IRQHandler(huart10.hdmarx); }
void DMA2_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(huart10.hdmatx); }
void DMA2_Stream3_IRQHandler(void) { HAL_DMA_IRQHandler(hspi2.hdmarx); }
void DMA2_Stream4_IRQHandler(void) { HAL_DMA_IRQHandler(hspi2.hdmatx); }
void SPI2_IRQHandler(void) { HAL_SPI_IRQHandler(&hspi2); }
void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&hfdcan1); }
void FDCAN2_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&hfdcan2); }
void FDCAN3_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&hfdcan3); }
