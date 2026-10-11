#include "bsp_usart.h"

#include <stddef.h>
#include <string.h>

#include "bsp_log.h"
#include "rm_critical.h"
#include "rm_rx_queue.h"

typedef struct {
    uint8_t dma_buffer[USART_RX_BUFFER_CAPACITY];
    uint8_t event_storage[USART_DEFERRED_EVENT_SLOTS]
                         [USART_RX_BUFFER_CAPACITY];
    uint16_t event_lengths[USART_DEFERRED_EVENT_SLOTS];
    RmRxQueue queue;
} USARTRxInbox;

static USARTInstance usart_instances[USART_MAX_INSTANCES];
static USARTRxInbox usart_rx_inboxes[USART_MAX_INSTANCES];
static size_t usart_instance_count;
static size_t usart_dispatch_cursor;
static USARTDispatchMode usart_dispatch_mode = USART_DISPATCH_INTERRUPT;

static bool USARTFindInstance(const USARTInstance *instance, size_t *index)
{
    if ((instance == NULL) || (index == NULL)) {
        return false;
    }

    for (size_t i = 0U; i < usart_instance_count; ++i) {
        if (&usart_instances[i] == instance) {
            *index = i;
            return true;
        }
    }
    return false;
}

bool USARTConfigureDispatch(USARTDispatchMode mode)
{
    if ((mode != USART_DISPATCH_INTERRUPT) &&
        (mode != USART_DISPATCH_DEFERRED)) {
        return false;
    }
    if (usart_instance_count != 0U) {
        return false;
    }

    usart_dispatch_mode = mode;
    usart_dispatch_cursor = 0U;
    return true;
}

uint8_t USARTServiceInit(USARTInstance *instance)
{
    HAL_StatusTypeDef status;
    uint8_t *receive_buffer;
    size_t index = 0U;

    if ((instance == NULL) || (instance->usart_handle == NULL) ||
        (instance->usart_handle->hdmarx == NULL) ||
        (instance->recv_buff_size == 0U)) {
        return 0U;
    }

    if (instance->dispatch_mode == USART_DISPATCH_DEFERRED) {
        if (!USARTFindInstance(instance, &index)) {
            return 0U;
        }
        receive_buffer = usart_rx_inboxes[index].dma_buffer;
    } else {
        receive_buffer = instance->recv_buff;
    }

    status = HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle,
                                         receive_buffer,
                                         instance->recv_buff_size);
    if (status == HAL_BUSY) {
        if (HAL_UART_AbortReceive(instance->usart_handle) != HAL_OK) {
            return 0U;
        }
        status = HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle,
                                             receive_buffer,
                                             instance->recv_buff_size);
    }
    if (status != HAL_OK) {
        return 0U;
    }

    __HAL_DMA_DISABLE_IT(instance->usart_handle->hdmarx, DMA_IT_HT);
    return 1U;
}

USARTInstance *USARTRegister(const USART_Init_Config_s *config)
{
    USARTInstance *instance;
    USARTRxInbox *inbox;

    if ((config == NULL) || (config->usart_handle == NULL) ||
        (config->recv_buff_size == 0U) ||
        (config->recv_buff_size > USART_RX_BUFFER_CAPACITY)) {
        return NULL;
    }
    if (usart_instance_count >= USART_MAX_INSTANCES) {
        LOGERROR("[bsp_usart] USART instance capacity exhausted");
        return NULL;
    }

    for (size_t i = 0U; i < usart_instance_count; ++i) {
        if (usart_instances[i].usart_handle == config->usart_handle) {
            LOGERROR("[bsp_usart] UART already registered");
            return NULL;
        }
    }

    instance = &usart_instances[usart_instance_count];
    inbox = &usart_rx_inboxes[usart_instance_count];
    memset(instance, 0, sizeof(*instance));
    memset(inbox, 0, sizeof(*inbox));
    if (!RmRxQueue_Init(&inbox->queue,
                        inbox->event_storage,
                        inbox->event_lengths,
                        USART_RX_BUFFER_CAPACITY,
                        USART_DEFERRED_EVENT_SLOTS)) {
        return NULL;
    }

    instance->usart_handle = config->usart_handle;
    instance->recv_buff_size = config->recv_buff_size;
    instance->module_callback = config->module_callback;
    instance->dispatch_mode = usart_dispatch_mode;
    usart_instance_count++;

    if (!USARTServiceInit(instance)) {
        usart_instance_count--;
        memset(instance, 0, sizeof(*instance));
        memset(inbox, 0, sizeof(*inbox));
        LOGERROR("[bsp_usart] failed to start receive DMA");
        return NULL;
    }

    return instance;
}

bool USARTUnregister(USARTInstance *instance)
{
    USARTInstance *last_instance;
    USARTRxInbox *last_inbox;
    RmCriticalState state;

    if ((instance == NULL) || (usart_instance_count == 0U)) {
        return false;
    }

    last_instance = &usart_instances[usart_instance_count - 1U];
    if (instance != last_instance) {
        /* Removing an older slot would invalidate a live endpoint pointer. */
        return false;
    }

    if (last_instance->usart_handle != NULL) {
        if (HAL_UART_AbortReceive(last_instance->usart_handle) != HAL_OK) {
            /* Keep the slot owned if DMA could not be stopped safely. */
            return false;
        }
    }

    last_inbox = &usart_rx_inboxes[usart_instance_count - 1U];
    state = RmCritical_Enter();
    memset(last_instance, 0, sizeof(*last_instance));
    memset(last_inbox, 0, sizeof(*last_inbox));
    usart_instance_count--;
    usart_dispatch_cursor = (usart_instance_count == 0U)
                                ? 0U
                                : (usart_dispatch_cursor % usart_instance_count);
    RmCritical_Exit(state);

    return true;
}

uint8_t USARTSend(USARTInstance *instance,
                  uint8_t *send_buf,
                  uint16_t send_size,
                  USART_TRANSFER_MODE mode)
{
    HAL_StatusTypeDef status;

    if ((instance == NULL) || (instance->usart_handle == NULL) ||
        (send_buf == NULL) || (send_size == 0U)) {
        return 0U;
    }

    switch (mode) {
    case USART_TRANSFER_BLOCKING:
        status = HAL_UART_Transmit(instance->usart_handle,
                                   send_buf,
                                   send_size,
                                   USART_BLOCKING_TX_TIMEOUT_MS);
        break;
    case USART_TRANSFER_IT:
        status = HAL_UART_Transmit_IT(instance->usart_handle,
                                     send_buf,
                                     send_size);
        break;
    case USART_TRANSFER_DMA:
        status = HAL_UART_Transmit_DMA(instance->usart_handle,
                                      send_buf,
                                      send_size);
        break;
    case USART_TRANSFER_NONE:
    default:
        return 0U;
    }

    return (status == HAL_OK) ? 1U : 0U;
}

uint8_t USARTIsReady(const USARTInstance *instance)
{
    if ((instance == NULL) || (instance->usart_handle == NULL)) {
        return 0U;
    }

    return (instance->usart_handle->gState == HAL_UART_STATE_READY) ? 1U : 0U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handle, uint16_t size)
{
    for (size_t i = 0U; i < usart_instance_count; ++i) {
        USARTInstance *instance = &usart_instances[i];
        uint16_t received_size;

        if (instance->usart_handle != handle) {
            continue;
        }

        received_size = (size <= instance->recv_buff_size)
                            ? size
                            : instance->recv_buff_size;
        instance->rx_event_count++;

        if (instance->dispatch_mode == USART_DISPATCH_DEFERRED) {
            RmRxQueuePushResult result = RmRxQueue_PushLatest(
                &usart_rx_inboxes[i].queue,
                usart_rx_inboxes[i].dma_buffer,
                received_size);
            if (result == RM_RX_QUEUE_PUSH_OVERWROTE_OLDEST) {
                instance->overwritten_event_count++;
            } else if (result == RM_RX_QUEUE_PUSH_REJECTED) {
                instance->rejected_event_count++;
            }
        } else {
            instance->recv_len = received_size;
            if (instance->module_callback != NULL) {
                instance->module_callback();
            }
            instance->dispatched_event_count++;
            memset(instance->recv_buff, 0, received_size);
        }

        if (!USARTServiceInit(instance)) {
            instance->recovery_failure_count++;
        }
        return;
    }
}

uint16_t USARTDispatchPending(uint16_t max_callbacks)
{
    uint16_t processed = 0U;
    uint16_t budget = (max_callbacks == 0U)
                          ? (USART_MAX_INSTANCES * USART_DEFERRED_EVENT_SLOTS)
                          : max_callbacks;

    if ((usart_dispatch_mode != USART_DISPATCH_DEFERRED) ||
        (usart_instance_count == 0U)) {
        return 0U;
    }

    while (processed < budget) {
        size_t checked;
        bool found = false;

        for (checked = 0U; checked < usart_instance_count; ++checked) {
            size_t index =
                (usart_dispatch_cursor + checked) % usart_instance_count;
            USARTInstance *instance = &usart_instances[index];
            USARTRxInbox *inbox = &usart_rx_inboxes[index];
            uint16_t length = 0U;
            RmCriticalState state = RmCritical_Enter();
            bool available = RmRxQueue_Pop(&inbox->queue,
                                           instance->recv_buff,
                                           sizeof(instance->recv_buff),
                                           &length);
            RmCritical_Exit(state);

            if (!available) {
                continue;
            }

            instance->recv_len = length;
            usart_dispatch_cursor = (index + 1U) % usart_instance_count;
            if (instance->module_callback != NULL) {
                instance->module_callback();
            }
            instance->dispatched_event_count++;
            memset(instance->recv_buff, 0, length);
            processed++;
            found = true;
            break;
        }

        if (!found) {
            break;
        }
    }

    return processed;
}

bool USARTGetDispatchStats(USARTDispatchStats *stats)
{
    RmCriticalState state;

    if (stats == NULL) {
        return false;
    }

    memset(stats, 0, sizeof(*stats));
    state = RmCritical_Enter();
    for (size_t i = 0U; i < usart_instance_count; ++i) {
        const USARTInstance *instance = &usart_instances[i];
        stats->received_events += instance->rx_event_count;
        stats->dispatched_events += instance->dispatched_event_count;
        stats->overwritten_events += instance->overwritten_event_count;
        stats->rejected_events += instance->rejected_event_count;
        stats->uart_errors += instance->error_count;
        stats->recovery_failures += instance->recovery_failure_count;
    }
    RmCritical_Exit(state);
    return true;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
    for (size_t i = 0U; i < usart_instance_count; ++i) {
        USARTInstance *instance = &usart_instances[i];

        if (instance->usart_handle != handle) {
            continue;
        }

        instance->error_count++;
        if (!USARTServiceInit(instance)) {
            instance->recovery_failure_count++;
        }
        return;
    }
}
