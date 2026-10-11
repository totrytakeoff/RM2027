#include "bsp_can.h"

#include <stddef.h>
#include <string.h>

#include "bsp_log.h"
#include "rm_critical.h"
#include "rm_rx_queue.h"
#include "rm_time.h"

enum {
    CAN_PENDING_SLOT_COUNT = 1U,
};

typedef struct {
    uint8_t storage[CAN_PENDING_SLOT_COUNT][CAN_CLASSIC_MAX_DATA_LENGTH];
    uint16_t lengths[CAN_PENDING_SLOT_COUNT];
    RmRxQueue queue;
} CANRxInbox;

static CANInstance can_instances[CAN_MAX_INSTANCES];
static CANRxInbox can_rx_inboxes[CAN_MAX_INSTANCES];
static size_t can_instance_count;
static uint8_t can1_filter_count;
static uint8_t can2_filter_count;
static uint8_t can_service_started;
static CANDispatchMode can_dispatch_mode = CAN_DISPATCH_INTERRUPT;
static size_t can_dispatch_cursor;
static volatile uint32_t can_received_frames;
static volatile uint32_t can_dispatched_frames;
static volatile uint32_t can_rejected_frames;

static uint8_t CANStartService(void)
{
    const uint32_t notifications =
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING;
    uint8_t can1_started = 0U;
    uint8_t can2_started = 0U;

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        goto failure;
    }
    can1_started = 1U;
    if (HAL_CAN_ActivateNotification(&hcan1, notifications) != HAL_OK) {
        goto failure;
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        goto failure;
    }
    can2_started = 1U;
    if (HAL_CAN_ActivateNotification(&hcan2, notifications) != HAL_OK) {
        goto failure;
    }

    can_service_started = 1U;
    LOGINFO("[bsp_can] CAN service initialized");
    return 1U;

failure:
    if (can2_started != 0U) {
        (void)HAL_CAN_DeactivateNotification(&hcan2, notifications);
        (void)HAL_CAN_Stop(&hcan2);
    }
    if (can1_started != 0U) {
        (void)HAL_CAN_DeactivateNotification(&hcan1, notifications);
        (void)HAL_CAN_Stop(&hcan1);
    }
    return 0U;
}

static uint8_t CANAddFilter(const CANInstance *instance)
{
    CAN_FilterTypeDef filter = {0};
    uint8_t *filter_count;
    uint32_t filter_bank_base;
    const uint32_t filter_id = instance->rx_id << 5U;

    if (instance->can_handle == &hcan1) {
        filter_count = &can1_filter_count;
        filter_bank_base = 0U;
    } else if (instance->can_handle == &hcan2) {
        filter_count = &can2_filter_count;
        filter_bank_base = CAN_FILTER_BANKS_PER_BUS;
    } else {
        return 0U;
    }

    if (*filter_count >= CAN_FILTER_BANKS_PER_BUS) {
        return 0U;
    }

    filter.FilterBank = filter_bank_base + *filter_count;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterFIFOAssignment =
        ((instance->rx_id & 1U) != 0U) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    filter.FilterIdHigh = filter_id;
    filter.FilterIdLow = filter_id;
    filter.FilterMaskIdHigh = filter_id;
    filter.FilterMaskIdLow = filter_id;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = CAN_FILTER_BANKS_PER_BUS;

    if (HAL_CAN_ConfigFilter(instance->can_handle, &filter) != HAL_OK) {
        return 0U;
    }

    (*filter_count)++;
    return 1U;
}

static void CANReleaseFilterSlot(const CANInstance *instance)
{
    uint8_t *filter_count = NULL;

    if (instance->can_handle == &hcan1) {
        filter_count = &can1_filter_count;
    } else if (instance->can_handle == &hcan2) {
        filter_count = &can2_filter_count;
    }

    if ((filter_count != NULL) && (*filter_count != 0U)) {
        (*filter_count)--;
    }
}

CANInstance *CANRegister(const CAN_Init_Config_s *config)
{
    CANInstance *instance;
    CANRxInbox *inbox;

    if ((config == NULL) || (config->can_handle == NULL) ||
        (config->tx_id > 0x7FFU) || (config->rx_id > 0x7FFU)) {
        return NULL;
    }
    if (can_instance_count >= CAN_MAX_INSTANCES) {
        LOGERROR("[bsp_can] CAN instance capacity exhausted");
        return NULL;
    }

    for (size_t i = 0U; i < can_instance_count; ++i) {
        if ((can_instances[i].can_handle == config->can_handle) &&
            (can_instances[i].rx_id == config->rx_id)) {
            LOGERROR("[bsp_can] duplicate CAN rx id 0x%03lx",
                     (unsigned long)config->rx_id);
            return NULL;
        }
    }

    instance = &can_instances[can_instance_count];
    inbox = &can_rx_inboxes[can_instance_count];
    memset(instance, 0, sizeof(*instance));
    memset(inbox, 0, sizeof(*inbox));
    if (!RmRxQueue_Init(&inbox->queue,
                        inbox->storage,
                        inbox->lengths,
                        CAN_CLASSIC_MAX_DATA_LENGTH,
                        CAN_PENDING_SLOT_COUNT)) {
        return NULL;
    }
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;
    instance->dispatch_mode = can_dispatch_mode;
    instance->txconf.StdId = config->tx_id;
    instance->txconf.IDE = CAN_ID_STD;
    instance->txconf.RTR = CAN_RTR_DATA;
    instance->txconf.DLC = CAN_CLASSIC_MAX_DATA_LENGTH;

    if (!CANAddFilter(instance)) {
        LOGERROR("[bsp_can] failed to configure CAN rx id 0x%03lx",
                 (unsigned long)config->rx_id);
        memset(instance, 0, sizeof(*instance));
        memset(inbox, 0, sizeof(*inbox));
        return NULL;
    }

    can_instance_count++;
    if ((!can_service_started) && (!CANStartService())) {
        can_instance_count--;
        CANReleaseFilterSlot(instance);
        memset(instance, 0, sizeof(*instance));
        memset(inbox, 0, sizeof(*inbox));
        LOGERROR("[bsp_can] failed to start CAN service");
        return NULL;
    }

    return instance;
}

bool CANConfigureDispatch(CANDispatchMode mode)
{
    if ((mode != CAN_DISPATCH_INTERRUPT) &&
        (mode != CAN_DISPATCH_DEFERRED)) {
        return false;
    }
    if ((can_instance_count != 0U) || (can_service_started != 0U)) {
        return false;
    }

    can_dispatch_mode = mode;
    can_dispatch_cursor = 0U;
    can_received_frames = 0U;
    can_dispatched_frames = 0U;
    can_rejected_frames = 0U;
    return true;
}

void CANSetDLC(CANInstance *instance, uint8_t length)
{
    if ((instance == NULL) || (length == 0U) ||
        (length > CAN_CLASSIC_MAX_DATA_LENGTH)) {
        LOGERROR("[bsp_can] invalid CAN payload length %u",
                 (unsigned)length);
        return;
    }

    instance->txconf.DLC = length;
}

uint8_t CANTransmit(CANInstance *instance, uint32_t timeout_us)
{
    static uint32_t busy_count;
    static uint32_t queue_fail_count;
    uint64_t start_us;

    if ((instance == NULL) || (instance->can_handle == NULL)) {
        return 0U;
    }

    start_us = RmTime_NowUs();
    while (HAL_CAN_GetTxMailboxesFreeLevel(instance->can_handle) == 0U) {
        if ((RmTime_NowUs() - start_us) >= timeout_us) {
            busy_count++;
            if ((busy_count <= 4U) ||
                ((busy_count & (busy_count - 1U)) == 0U)) {
                LOGWARNING("[bsp_can] mailbox busy, dropped frames=%lu",
                           (unsigned long)busy_count);
            }
            return 0U;
        }
    }

    if (HAL_CAN_AddTxMessage(instance->can_handle,
                            &instance->txconf,
                            instance->tx_buff,
                            &instance->tx_mailbox) != HAL_OK) {
        queue_fail_count++;
        if ((queue_fail_count <= 4U) ||
            ((queue_fail_count & (queue_fail_count - 1U)) == 0U)) {
            LOGWARNING("[bsp_can] failed to queue frames=%lu",
                       (unsigned long)queue_fail_count);
        }
        return 0U;
    }

    return 1U;
}

static void CANFIFOCallback(CAN_HandleTypeDef *handle, uint32_t fifo)
{
    CAN_RxHeaderTypeDef header;
    uint8_t rx_buffer[CAN_CLASSIC_MAX_DATA_LENGTH];

    while (HAL_CAN_GetRxFifoFillLevel(handle, fifo) != 0U) {
        if (HAL_CAN_GetRxMessage(handle, fifo, &header, rx_buffer) != HAL_OK) {
            return;
        }
        if ((header.IDE != CAN_ID_STD) || (header.RTR != CAN_RTR_DATA)) {
            continue;
        }

        for (size_t i = 0U; i < can_instance_count; ++i) {
            CANInstance *instance = &can_instances[i];
            uint8_t frame_length;

            if ((instance->can_handle != handle) ||
                (instance->rx_id != header.StdId)) {
                continue;
            }

            frame_length =
                (header.DLC <= CAN_CLASSIC_MAX_DATA_LENGTH)
                    ? (uint8_t)header.DLC
                    : CAN_CLASSIC_MAX_DATA_LENGTH;
            can_received_frames++;
            if (instance->dispatch_mode == CAN_DISPATCH_DEFERRED) {
                CANRxInbox *inbox = &can_rx_inboxes[i];
                if (RmRxQueue_PushLatest(&inbox->queue,
                                         rx_buffer,
                                         frame_length) ==
                    RM_RX_QUEUE_PUSH_REJECTED) {
                    can_rejected_frames++;
                }
            } else {
                instance->rx_len = frame_length;
                memcpy(instance->rx_buff, rx_buffer, frame_length);
                if (instance->can_module_callback != NULL) {
                    instance->can_module_callback(instance);
                }
                can_dispatched_frames++;
            }
            break;
        }
    }
}

uint16_t CANDispatchPending(uint16_t max_callbacks)
{
    uint16_t processed = 0U;
    uint16_t budget = (max_callbacks == 0U)
                          ? CAN_MAX_INSTANCES
                          : max_callbacks;

    if ((can_dispatch_mode != CAN_DISPATCH_DEFERRED) ||
        (can_instance_count == 0U)) {
        return 0U;
    }

    while (processed < budget) {
        size_t checked;
        bool found = false;

        for (checked = 0U; checked < can_instance_count; ++checked) {
            size_t index = (can_dispatch_cursor + checked) % can_instance_count;
            CANInstance *instance = &can_instances[index];
            CANRxInbox *inbox = &can_rx_inboxes[index];
            uint16_t length = 0U;
            RmCriticalState state = RmCritical_Enter();
            bool available = RmRxQueue_Pop(&inbox->queue,
                                           instance->rx_buff,
                                           sizeof(instance->rx_buff),
                                           &length);
            RmCritical_Exit(state);

            if (!available) {
                continue;
            }

            instance->rx_len = (uint8_t)length;
            can_dispatch_cursor = (index + 1U) % can_instance_count;
            if (instance->can_module_callback != NULL) {
                instance->can_module_callback(instance);
            }
            can_dispatched_frames++;
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

bool CANGetDispatchStats(CANDispatchStats *stats)
{
    uint32_t coalesced = 0U;
    RmCriticalState state;

    if (stats == NULL) {
        return false;
    }

    state = RmCritical_Enter();
    for (size_t i = 0U; i < can_instance_count; ++i) {
        coalesced += can_rx_inboxes[i].queue.overwrite_count;
    }
    stats->received_frames = can_received_frames;
    stats->dispatched_frames = can_dispatched_frames;
    stats->coalesced_frames = coalesced;
    stats->rejected_frames = can_rejected_frames;
    RmCritical_Exit(state);
    return true;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handle)
{
    CANFIFOCallback(handle, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *handle)
{
    CANFIFOCallback(handle, CAN_RX_FIFO1);
}
