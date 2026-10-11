#ifndef RM_RX_QUEUE_H
#define RM_RX_QUEUE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    RM_RX_QUEUE_PUSH_REJECTED = 0,
    RM_RX_QUEUE_PUSH_STORED,
    RM_RX_QUEUE_PUSH_OVERWROTE_OLDEST,
} RmRxQueuePushResult;

/**
 * Fixed-storage receive queue with newest-data overflow semantics.
 *
 * The queue does not allocate memory and does not provide its own locking.
 * A single-core ISR producer and task consumer must serialize Pop() against
 * the producer, for example by briefly masking interrupts while popping.
 */
typedef struct {
    uint8_t *storage;
    uint16_t *lengths;
    uint16_t slot_size;
    uint16_t slot_count;
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
    volatile uint32_t push_count;
    volatile uint32_t pop_count;
    volatile uint32_t overwrite_count;
} RmRxQueue;

bool RmRxQueue_Init(RmRxQueue *queue,
                    void *storage,
                    uint16_t *lengths,
                    uint16_t slot_size,
                    uint16_t slot_count);

/** Store a payload, discarding the oldest queued payload when full. */
RmRxQueuePushResult RmRxQueue_PushLatest(RmRxQueue *queue,
                                         const void *payload,
                                         uint16_t length);

/** Pop the oldest retained payload. The queue is unchanged on failure. */
bool RmRxQueue_Pop(RmRxQueue *queue,
                   void *destination,
                   uint16_t destination_size,
                   uint16_t *out_length);

uint16_t RmRxQueue_Pending(const RmRxQueue *queue);

#endif /* RM_RX_QUEUE_H */
