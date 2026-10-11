#include "rm_rx_queue.h"

#include <string.h>

bool RmRxQueue_Init(RmRxQueue *queue,
                    void *storage,
                    uint16_t *lengths,
                    uint16_t slot_size,
                    uint16_t slot_count)
{
    if ((queue == NULL) || (storage == NULL) || (lengths == NULL) ||
        (slot_size == 0U) || (slot_count == 0U)) {
        return false;
    }

    queue->storage = storage;
    queue->lengths = lengths;
    queue->slot_size = slot_size;
    queue->slot_count = slot_count;
    queue->head = 0U;
    queue->tail = 0U;
    queue->count = 0U;
    queue->push_count = 0U;
    queue->pop_count = 0U;
    queue->overwrite_count = 0U;
    memset(storage, 0, (size_t)slot_size * slot_count);
    memset(lengths, 0, sizeof(*lengths) * slot_count);
    return true;
}

RmRxQueuePushResult RmRxQueue_PushLatest(RmRxQueue *queue,
                                         const void *payload,
                                         uint16_t length)
{
    RmRxQueuePushResult result = RM_RX_QUEUE_PUSH_STORED;
    uint16_t head;

    if ((queue == NULL) || (payload == NULL) || (length == 0U) ||
        (length > queue->slot_size) || (queue->slot_count == 0U)) {
        return RM_RX_QUEUE_PUSH_REJECTED;
    }

    if (queue->count == queue->slot_count) {
        queue->tail = (uint16_t)((queue->tail + 1U) % queue->slot_count);
        queue->count--;
        queue->overwrite_count++;
        result = RM_RX_QUEUE_PUSH_OVERWROTE_OLDEST;
    }

    head = queue->head;
    memcpy(queue->storage + ((size_t)head * queue->slot_size),
           payload,
           length);
    queue->lengths[head] = length;
    queue->head = (uint16_t)((head + 1U) % queue->slot_count);
    queue->count++;
    queue->push_count++;
    return result;
}

bool RmRxQueue_Pop(RmRxQueue *queue,
                   void *destination,
                   uint16_t destination_size,
                   uint16_t *out_length)
{
    uint16_t tail;
    uint16_t length;

    if ((queue == NULL) || (destination == NULL) ||
        (out_length == NULL) || (queue->count == 0U) ||
        (queue->slot_count == 0U)) {
        return false;
    }

    tail = queue->tail;
    length = queue->lengths[tail];
    if ((length == 0U) || (length > destination_size)) {
        return false;
    }

    memcpy(destination,
           queue->storage + ((size_t)tail * queue->slot_size),
           length);
    *out_length = length;
    queue->lengths[tail] = 0U;
    queue->tail = (uint16_t)((tail + 1U) % queue->slot_count);
    queue->count--;
    queue->pop_count++;
    return true;
}

uint16_t RmRxQueue_Pending(const RmRxQueue *queue)
{
    return (queue != NULL) ? queue->count : 0U;
}
