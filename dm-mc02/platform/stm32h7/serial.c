#include "h7_private.h"
#include "resources.h"
#include "usart.h"
#include <string.h>
typedef struct {
    UART_HandleTypeDef *uart;
    RmSerialChunk chunks[MC02_SERIAL_SOFTWARE_DEPTH];
    RmSerialStats stats;
    uint32_t head, tail, tx_head, tx_tail;
    uint16_t rx_capacity, rx_position, tx_lengths[MC02_SERIAL_TX_SLOTS];
    uint16_t tx_capacity, depth;
    bool tx_running, fault, aborting, aborted, started;
    bool rx_wrap_before_tc;
    uint32_t retry_ms, abort_deadline_ms;
    unsigned recovery_attempts;
} Endpoint;
static Endpoint endpoints[RM_SERIAL_COUNT];
static uint8_t rx_buffers[RM_SERIAL_COUNT][MC02_SERIAL_DMA_BYTES] RM_DMA_STORAGE;
static uint8_t tx_buffers[RM_SERIAL_COUNT][MC02_SERIAL_TX_SLOTS][MC02_SERIAL_TX_BYTES] RM_DMA_STORAGE;
static bool registered;
static const IRQn_Type uart_irqs[]={UART5_IRQn,USART1_IRQn,UART7_IRQn,USART10_IRQn};
static const IRQn_Type rx_irqs[]={DMA1_Stream4_IRQn,DMA1_Stream5_IRQn,DMA1_Stream7_IRQn,DMA2_Stream1_IRQn};
/* UART5 has no TX DMA; its placeholder is never enabled or disabled here. */
static const IRQn_Type tx_irqs[]={NonMaskableInt_IRQn,DMA1_Stream6_IRQn,DMA2_Stream0_IRQn,DMA2_Stream2_IRQn};
/* Called with interrupts masked. Storage remains pinned until MCU reset: a
   missing callback is never treated as permission to reuse a DMA TX buffer. */
static void quarantine(Endpoint *e,unsigned id)
{
    e->stats.quarantined=true; e->fault=true; e->started=false;
    e->aborting=false; e->aborted=false; ++e->stats.generation;
    e->tail=e->head;
    e->uart->Instance->CR3 &= ~(USART_CR3_DMAR|USART_CR3_DMAT);
    e->uart->Instance->CR1 &= ~USART_CR1_UE;
    DMA_HandleTypeDef *streams[]={e->uart->hdmarx,e->uart->hdmatx};
    for (unsigned i=0;i<2U;++i) if (streams[i]) {
        DMA_Stream_TypeDef *stream=(DMA_Stream_TypeDef *)streams[i]->Instance;
        stream->CR &= ~(DMA_SxCR_EN|DMA_SxCR_TCIE|DMA_SxCR_HTIE|DMA_SxCR_TEIE|DMA_SxCR_DMEIE);
        stream->FCR &= ~DMA_SxFCR_FEIE;
    }
    HAL_NVIC_DisableIRQ(uart_irqs[id]); HAL_NVIC_DisableIRQ(rx_irqs[id]);
    if (e->uart->hdmatx && tx_irqs[id]>=0) HAL_NVIC_DisableIRQ(tx_irqs[id]);
    __DSB();
}
static Endpoint *find(UART_HandleTypeDef *uart, unsigned *id)
{
    if (!registered) return NULL;
    for (unsigned i=0; i<RM_SERIAL_COUNT; ++i) if (endpoints[i].uart == uart) {
        if (id) *id = i;
        return &endpoints[i];
    }
    return NULL;
}
static bool pump_tx(Endpoint *e, unsigned id)
{
    if (e->tx_running || e->tx_head == e->tx_tail) return true;
    if (e->fault || e->aborting || !e->started) return false;
    unsigned slot = e->tx_tail % MC02_SERIAL_TX_SLOTS;
    e->tx_running = true;
    HAL_StatusTypeDef r = HAL_UART_Transmit_DMA(e->uart, tx_buffers[id][slot], e->tx_lengths[slot]);
    if (r != HAL_OK) {
        e->tx_running = false; e->fault = true; ++e->stats.tx_errors; ++e->stats.generation;
        return false;
    }
    return true;
}
void RmSerial_EnableDmaIRQs(void)
{
    const IRQn_Type irqs[] = { DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn,
        DMA1_Stream7_IRQn, DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn,
        DMA2_Stream3_IRQn, DMA2_Stream4_IRQn };
    for (unsigned i=0; i<sizeof(irqs)/sizeof(irqs[0]); ++i) {
        HAL_NVIC_SetPriority(irqs[i], 6, 0); HAL_NVIC_EnableIRQ(irqs[i]);
    }
}
bool RmSerial_Start(void)
{
    if (registered) return false;
    UART_HandleTypeDef *handles[] = { &huart5, &huart1, &huart7, &huart10 };
    for (unsigned i=0; i<RM_SERIAL_COUNT; ++i) {
        Endpoint *e = &endpoints[i];
        e->uart = handles[i]; e->stats.generation = 1U;
        e->rx_capacity = i == RM_SERIAL_REMOTE ? 64U : i == RM_SERIAL_CONSOLE ? 256U : 512U;
        e->tx_capacity = i == RM_SERIAL_VISION ? 64U : 256U;
        e->depth = i == RM_SERIAL_REMOTE ? 4U : i == RM_SERIAL_CONSOLE ? 8U : 32U;
        if (!RmMemory_IsDmaBuffer(rx_buffers[i], e->rx_capacity) ||
            !RmMemory_IsDmaBuffer(tx_buffers[i], sizeof(tx_buffers[i]))) return false;
    }
    registered = true; /* before any receive DMA can publish a callback */
    RmSerial_EnableDmaIRQs();
    for (unsigned i=0; i<RM_SERIAL_COUNT; ++i) {
        Endpoint *e = &endpoints[i];
        e->started = HAL_UARTEx_ReceiveToIdle_DMA(e->uart, rx_buffers[i], e->rx_capacity) == HAL_OK;
        if (!e->started) { e->fault = true; ++e->stats.restart_failures; }
    }
    /* Individual RX failures use the bounded service recovery. They must not
       prevent the scheduler, diagnostics and other transports from starting. */
    return true;
}
static void publish(Endpoint *e, const uint8_t *src, unsigned length, uint64_t timestamp)
{
    while (length != 0U) {
        unsigned n = length > RM_SERIAL_CHUNK_BYTES ? RM_SERIAL_CHUNK_BYTES : length;
        if (e->head - e->tail >= e->depth) {
            e->stats.rx_dropped += length; ++e->stats.generation;
            return; /* preserve ordering, never overwrite a record being consumed */
        }
        RmSerialChunk *chunk = &e->chunks[e->head % e->depth];
        chunk->timestamp_us = timestamp; chunk->generation = e->stats.generation;
        chunk->length = (uint16_t)n; memcpy(chunk->data, src, n);
        ++e->head; src += n; length -= n;
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    unsigned id;
    Endpoint *e = find(uart, &id);
    if (!e || e->fault || e->aborting) return;
    RmCriticalState key = RmCritical_Enter();
    if (size == 0U || size > e->rx_capacity) {
        e->fault=true; ++e->stats.rx_errors; ++e->stats.generation;
        RmCritical_Exit(key); return;
    }
    uint64_t timestamp = RmTime_NowUs();
    /* HT/TC report fixed boundaries, which may already be behind an IDLE
       callback. Read the live DMA cursor instead; account a wrap seen by IDLE
       once, then acknowledge its delayed TC. Equal IRQ priority and PRIMASK
       serialize callbacks. Hardware acceptance must bound IRQ latency below
       one ring revolution (half-ring is the intended budget). */
    unsigned remaining=__HAL_DMA_GET_COUNTER(uart->hdmarx);
    if (remaining>e->rx_capacity) {
        e->fault=true; ++e->stats.rx_errors; ++e->stats.generation;
        RmCritical_Exit(key); return;
    }
    unsigned position=e->rx_capacity-remaining;
    if (position==e->rx_capacity) position=0U;
    unsigned old=e->rx_position, n;
    bool tc=HAL_UARTEx_GetRxEventType(uart)==HAL_UART_RXEVENT_TC;
    bool wrapped=position<old;
    if (tc && e->rx_wrap_before_tc) e->rx_wrap_before_tc=false;
    else if (tc) wrapped=true;
    if (wrapped) {
        n = e->rx_capacity - old + position;
        publish(e, &rx_buffers[id][old], e->rx_capacity-old, timestamp);
        publish(e, rx_buffers[id], position, timestamp);
        e->rx_wrap_before_tc=!tc;
    } else {
        n=position-old; publish(e,&rx_buffers[id][old],n,timestamp);
    }
    e->rx_position = (uint16_t)position;
    e->stats.rx_bytes += n; ++e->stats.rx_events;
    RmCritical_Exit(key);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    unsigned id;
    Endpoint *e = find(uart, &id);
    if (!e || e->stats.quarantined) return;
    RmCriticalState key = RmCritical_Enter();
    if (e->tx_running) { e->tx_running=false; ++e->tx_tail; ++e->stats.tx_completed; }
    (void)pump_tx(e,id);
    RmCritical_Exit(key);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    Endpoint *e = find(uart,NULL);
    if (!e || e->stats.quarantined) return;
    RmCriticalState key = RmCritical_Enter();
    e->fault=true; ++e->stats.rx_errors; ++e->stats.generation;
    RmCritical_Exit(key);
}
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *uart)
{
    Endpoint *e = find(uart,NULL);
    if (!e || e->stats.quarantined || !e->aborting) return;
    RmCriticalState key = RmCritical_Enter(); e->aborted=true; RmCritical_Exit(key);
}
bool RmSerial_Read(RmSerialId id, RmSerialChunk *chunk)
{
    if ((unsigned)id >= RM_SERIAL_COUNT || !chunk || !registered) return false;
    Endpoint *e = &endpoints[id];
    RmCriticalState key = RmCritical_Enter();
    bool available = e->head != e->tail;
    if (available) { *chunk = e->chunks[e->tail % e->depth]; ++e->tail; }
    RmCritical_Exit(key);
    return available;
}
RmStatus RmSerial_TrySendCopy(RmSerialId id, const void *data, size_t length)
{
    if ((unsigned)id >= RM_SERIAL_COUNT || !data || !length || !registered) return RM_INVALID;
    Endpoint *e = &endpoints[id];
    if (!e->uart->hdmatx || length > e->tx_capacity) return RM_INVALID;
    RmCriticalState key = RmCritical_Enter();
    if (e->fault || e->aborting || !e->started) { RmCritical_Exit(key); return RM_OFFLINE; }
    if (e->tx_head - e->tx_tail >= MC02_SERIAL_TX_SLOTS) {
        ++e->stats.tx_busy; RmCritical_Exit(key); return RM_BUSY;
    }
    unsigned slot = e->tx_head % MC02_SERIAL_TX_SLOTS;
    memcpy(tx_buffers[id][slot],data,length); e->tx_lengths[slot]=(uint16_t)length;
    ++e->tx_head;
    bool accepted = pump_tx(e,id);
    if (accepted) ++e->stats.tx_accepted;
    else --e->tx_head;
    RmCritical_Exit(key);
    return accepted ? RM_OK : RM_ERROR;
}
bool RmSerial_GetStats(RmSerialId id, RmSerialStats *stats)
{
    if ((unsigned)id >= RM_SERIAL_COUNT || !stats || !registered) return false;
    RmCriticalState key = RmCritical_Enter();
    *stats = endpoints[id].stats;
    stats->online=endpoints[id].started && !endpoints[id].fault && !endpoints[id].aborting && !stats->quarantined;
    stats->pending_chunks = endpoints[id].head-endpoints[id].tail;
    RmCritical_Exit(key);
    return true;
}
void RmSerial_Service(uint32_t now_ms)
{
    (void)now_ms; /* Release time is not the recovery deadline clock. */
    if (!registered) return;
    for (unsigned i=0;i<RM_SERIAL_COUNT;++i) {
        Endpoint *e=&endpoints[i];
        RmCriticalState key=RmCritical_Enter();
        uint32_t now=RmTime_NowMs();
        if (e->stats.quarantined) { RmCritical_Exit(key); continue; }
        if (e->fault && !e->aborting && (int32_t)(now-e->retry_ms)>=0) {
            if (e->recovery_attempts>=MC02_SERIAL_RECOVERY_ATTEMPTS) {
                ++e->stats.recovery_exhausted; quarantine(e,i);
                RmCritical_Exit(key); continue;
            }
            ++e->recovery_attempts;
            e->aborting=true; e->aborted=false; e->started=false;
            e->abort_deadline_ms=now+MC02_SERIAL_ABORT_TIMEOUT_MS;
            if (HAL_UART_Abort_IT(e->uart)!=HAL_OK) {
                e->aborting=false; ++e->stats.restart_failures; e->retry_ms=now+100U;
            }
        }
        if (e->aborting && !e->aborted && (int32_t)(RmTime_NowMs()-e->abort_deadline_ms)>=0) {
            ++e->stats.abort_timeouts; quarantine(e,i);
            RmCritical_Exit(key); continue;
        }
        if (e->aborting && e->aborted) {
            e->tx_running=false; e->tx_tail=e->tx_head;
            e->tail=e->head; e->rx_position=0U; e->rx_wrap_before_tc=false;
            e->aborting=false; e->aborted=false;
            ++e->stats.generation;
            e->started=HAL_UARTEx_ReceiveToIdle_DMA(e->uart,rx_buffers[i],e->rx_capacity)==HAL_OK;
            e->fault=!e->started;
            if (e->started) { ++e->stats.restarts; e->recovery_attempts=0U; }
            else { ++e->stats.restart_failures; e->retry_ms=RmTime_NowMs()+100U; }
        }
        RmCritical_Exit(key);
    }
}
