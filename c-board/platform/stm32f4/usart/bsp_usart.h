#ifndef BSP_USART_H
#define BSP_USART_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define USART_MAX_INSTANCES 3U
#define USART_RX_BUFFER_CAPACITY 256U
#define USART_DEFERRED_EVENT_SLOTS 4U
#define USART_BLOCKING_TX_TIMEOUT_MS 100U

typedef void (*USARTReceiveCallback)(void);

typedef enum {
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

typedef enum {
    USART_DISPATCH_INTERRUPT = 0,
    USART_DISPATCH_DEFERRED,
} USARTDispatchMode;

typedef struct {
    uint32_t received_events;
    uint32_t dispatched_events;
    uint32_t overwritten_events;
    uint32_t rejected_events;
    uint32_t uart_errors;
    uint32_t recovery_failures;
} USARTDispatchStats;

typedef struct {
    /** Stable callback-facing buffer; never used as DMA storage in deferred mode. */
    uint8_t recv_buff[USART_RX_BUFFER_CAPACITY];
    uint16_t recv_buff_size;
    uint16_t recv_len;
    UART_HandleTypeDef *usart_handle;
    USARTReceiveCallback module_callback;
    USARTDispatchMode dispatch_mode;
    /** Debugger-visible receive and recovery counters. */
    volatile uint32_t rx_event_count;
    volatile uint32_t dispatched_event_count;
    volatile uint32_t overwritten_event_count;
    volatile uint32_t rejected_event_count;
    volatile uint32_t error_count;
    volatile uint32_t recovery_failure_count;
} USARTInstance;

typedef struct {
    uint16_t recv_buff_size;
    UART_HandleTypeDef *usart_handle;
    USARTReceiveCallback module_callback;
} USART_Init_Config_s;

/** Select callback context before the first UART endpoint is registered. */
bool USARTConfigureDispatch(USARTDispatchMode mode);

/** Register an exclusive UART endpoint in fixed static storage during startup. */
USARTInstance *USARTRegister(const USART_Init_Config_s *config);

/**
 * Roll back the most recently registered UART endpoint during startup.
 *
 * Registration starts receive DMA immediately, so callers that fail a later
 * initialization step must use this API instead of leaking the endpoint.
 * Only the newest endpoint can be removed; this keeps all returned instance
 * addresses stable for already initialized devices. Returns false if the
 * endpoint is not the newest one or receive DMA cannot be stopped safely.
 */
bool USARTUnregister(USARTInstance *instance);

/** Restart receive-to-idle DMA for an endpoint. */
uint8_t USARTServiceInit(USARTInstance *instance);

/** Dispatch retained DMA events from task context. Zero means all retained events. */
uint16_t USARTDispatchPending(uint16_t max_callbacks);

/** Copy aggregate receive/dispatch counters. */
bool USARTGetDispatchStats(USARTDispatchStats *stats);

/** Send one frame using the selected transfer mode. */
uint8_t USARTSend(USARTInstance *instance,
                  uint8_t *send_buf,
                  uint16_t send_size,
                  USART_TRANSFER_MODE mode);

/** Return 1 when the UART transmit state is ready. */
uint8_t USARTIsReady(const USARTInstance *instance);

#endif /* BSP_USART_H */
