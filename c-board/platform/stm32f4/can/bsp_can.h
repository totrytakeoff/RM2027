#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "can.h"

#define CAN_MAX_INSTANCES 16U
#define CAN_FILTER_BANKS_PER_BUS 14U
#define CAN_CLASSIC_MAX_DATA_LENGTH 8U

typedef struct CANInstance CANInstance;
typedef void (*CANReceiveCallback)(CANInstance *instance);

typedef enum {
    CAN_DISPATCH_INTERRUPT = 0,
    CAN_DISPATCH_DEFERRED,
} CANDispatchMode;

typedef struct {
    uint32_t received_frames;
    uint32_t dispatched_frames;
    uint32_t coalesced_frames;
    uint32_t rejected_frames;
} CANDispatchStats;

struct CANInstance {
    CAN_HandleTypeDef *can_handle;
    CAN_TxHeaderTypeDef txconf;
    uint32_t tx_id;
    uint32_t tx_mailbox;
    uint8_t tx_buff[CAN_CLASSIC_MAX_DATA_LENGTH];
    uint8_t rx_buff[CAN_CLASSIC_MAX_DATA_LENGTH];
    uint32_t rx_id;
    uint8_t rx_len;
    CANReceiveCallback can_module_callback;
    void *id;
    CANDispatchMode dispatch_mode;
};

typedef struct {
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    CANReceiveCallback can_module_callback;
    void *id;
} CAN_Init_Config_s;

/** Register one standard-ID endpoint in fixed static storage during startup. */
CANInstance *CANRegister(const CAN_Init_Config_s *config);

/** Select callback context before the first endpoint is registered. */
bool CANConfigureDispatch(CANDispatchMode mode);

/** Dispatch retained endpoint snapshots from task context. Zero means all. */
uint16_t CANDispatchPending(uint16_t max_callbacks);

/** Copy aggregate receive/dispatch counters. */
bool CANGetDispatchStats(CANDispatchStats *stats);

/** Set the classic CAN payload length (1..8 bytes). */
void CANSetDLC(CANInstance *instance, uint8_t length);

/**
 * Queue the current transmit buffer, waiting at most timeout_us for a mailbox.
 * Returns 1 on success and 0 on invalid input, timeout, or HAL failure.
 */
uint8_t CANTransmit(CANInstance *instance, uint32_t timeout_us);

#endif /* BSP_CAN_H */
