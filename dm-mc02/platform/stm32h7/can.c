#include "h7_private.h"
#include "resources.h"
#include "fdcan.h"
#include <string.h>
_Static_assert(sizeof(RmCanFrame)==32U,"classic CAN ring record budget changed");
typedef struct {
    RmCanFrame frame;
    uint32_t epoch, deadline_ms, flight_epoch, flight_deadline_ms;
    bool pending, flight, abort_requested, zero_frame, flight_zero;
} TxSlot;
typedef struct {
    FDCAN_HandleTypeDef *handle;
    RmCanFrame ring[MC02_CAN_SOFTWARE_DEPTH];
    TxSlot tx[3];
    RmCanStats stats;
    uint32_t head, tail, recover_at;
    unsigned recover_attempts;
    bool fault, started;
} CanBus;
static CanBus buses[RM_CAN_COUNT];
static bool registered;
static bool (*gate_permit)(uint32_t,uint32_t);
static void (*gate_stop)(uint32_t);
static const uint32_t buffers[]={FDCAN_TX_BUFFER0,FDCAN_TX_BUFFER1,FDCAN_TX_BUFFER2};
static CanBus *find(FDCAN_HandleTypeDef *handle, RmCanId *id)
{
    if (!registered) return NULL;
    for (unsigned i=0;i<RM_CAN_COUNT;++i) if (buses[i].handle==handle) {
        if (id) *id=(RmCanId)i;
        return &buses[i];
    }
    return NULL;
}
void RmCan_SetSafetyHooks(bool (*permit)(uint32_t,uint32_t),void (*stop)(uint32_t))
{
    if (registered) return;
    gate_permit=permit; gate_stop=stop;
}
bool RmCan_Start(void)
{
    if (registered) return false;
    FDCAN_HandleTypeDef *handles[]={&hfdcan1,&hfdcan2,&hfdcan3};
    for (unsigned i=0;i<RM_CAN_COUNT;++i) {
        buses[i].handle=handles[i]; buses[i].stats.generation=1U;
    }
    registered=true;
    for (unsigned i=0;i<RM_CAN_COUNT;++i) {
        CanBus *b=&buses[i];
        /* Bringup is a receive monitor: full classic ID ranges. Production profiles
           will replace these with registered device filters before permitting motion. */
        FDCAN_FilterTypeDef f={0};
        f.IdType=FDCAN_STANDARD_ID; f.FilterIndex=0; f.FilterType=FDCAN_FILTER_MASK;
        f.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
        if (HAL_FDCAN_ConfigFilter(b->handle,&f)!=HAL_OK) return false;
        f.IdType=FDCAN_EXTENDED_ID; f.FilterConfig=FDCAN_FILTER_TO_RXFIFO1;
        if (HAL_FDCAN_ConfigFilter(b->handle,&f)!=HAL_OK ||
            HAL_FDCAN_ConfigGlobalFilter(b->handle,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE)!=HAL_OK)
            return false;
        uint32_t notifications=FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO1_NEW_MESSAGE|
            FDCAN_IT_RX_FIFO0_MESSAGE_LOST|FDCAN_IT_RX_FIFO1_MESSAGE_LOST|
            FDCAN_IT_BUS_OFF|FDCAN_IT_ERROR_PASSIVE|FDCAN_IT_TX_COMPLETE|FDCAN_IT_TX_ABORT_COMPLETE;
        if (HAL_FDCAN_ActivateNotification(b->handle,notifications,7U)!=HAL_OK ||
            HAL_FDCAN_Start(b->handle)!=HAL_OK) return false;
        b->started=true; b->stats.online=true;
    }
    return true;
}
static void receive(FDCAN_HandleTypeDef *handle,uint32_t fifo,uint32_t interrupts)
{
    RmCanId id;
    CanBus *b=find(handle,&id);
    if (!b) return;
    RmCriticalState key=RmCritical_Enter();
    if ((interrupts&(FDCAN_IT_RX_FIFO0_MESSAGE_LOST|FDCAN_IT_RX_FIFO1_MESSAGE_LOST))!=0U) {
        ++b->stats.rx_dropped; ++b->stats.generation;
        if (gate_stop) gate_stop(1U<<10);
    }
    unsigned budget=fifo==FDCAN_RX_FIFO0?MC02_CAN_RX0_ELEMENTS:MC02_CAN_RX1_ELEMENTS;
    while (budget-- && HAL_FDCAN_GetRxFifoFillLevel(handle,fifo)!=0U) {
        FDCAN_RxHeaderTypeDef h;
        uint8_t raw[64]; /* HAL copies DLC bytes: never receive directly into an 8-byte record */
        if (HAL_FDCAN_GetRxMessage(handle,fifo,&h,raw)!=HAL_OK) { ++b->stats.rx_errors; break; }
        if (h.FDFormat!=FDCAN_CLASSIC_CAN || h.RxFrameType!=FDCAN_DATA_FRAME || h.DataLength>FDCAN_DLC_BYTES_8) {
            ++b->stats.rx_errors; continue;
        }
        ++b->stats.rx_frames;
        if (b->head-b->tail>=MC02_CAN_SOFTWARE_DEPTH) {
            ++b->stats.rx_dropped; ++b->stats.generation;
            if (gate_stop) gate_stop(1U<<10);
            continue;
        }
        RmCanFrame *frame=&b->ring[b->head%MC02_CAN_SOFTWARE_DEPTH];
        *frame=(RmCanFrame){.bus=id,.identifier=h.Identifier,.extended=h.IdType==FDCAN_EXTENDED_ID,
            .length=(uint8_t)h.DataLength,.timestamp_us=RmTime_NowUs(),.generation=b->stats.generation};
        memcpy(frame->data,raw,frame->length); ++b->head;
    }
    RmCritical_Exit(key);
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h,uint32_t i) { receive(h,FDCAN_RX_FIFO0,i); }
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *h,uint32_t i) { receive(h,FDCAN_RX_FIFO1,i); }
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *h,uint32_t indexes)
{
    CanBus *b=find(h,NULL); if (!b) return;
    RmCriticalState key=RmCritical_Enter();
    for (unsigned i=0;i<3U;++i) if ((indexes&buffers[i])!=0U) {
        b->tx[i].flight=false; b->tx[i].abort_requested=false; ++b->stats.tx_completed;
    }
    RmCritical_Exit(key);
}
void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *h,uint32_t indexes)
{
    CanBus *b=find(h,NULL); if (!b) return;
    RmCriticalState key=RmCritical_Enter();
    for (unsigned i=0;i<3U;++i) if ((indexes&buffers[i])!=0U) {
        b->tx[i].flight=false; b->tx[i].abort_requested=false; ++b->stats.tx_aborted;
    }
    RmCritical_Exit(key);
}
static void bus_fault(CanBus *b)
{
    if (!b) return;
    RmCriticalState key=RmCritical_Enter();
    if (!b->fault) { ++b->stats.bus_off_count; ++b->stats.generation; }
    b->fault=true; b->stats.online=false;
    if (gate_stop) gate_stop(1U<<9);
    RmCritical_Exit(key);
}
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *h,uint32_t flags)
{
    CanBus *b=find(h,NULL);
    if (!b) return;
    if ((flags&FDCAN_IT_BUS_OFF)!=0U) bus_fault(b);
    else ++b->stats.rx_errors;
}
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *h) { bus_fault(find(h,NULL)); }
bool RmCan_Read(RmCanId id,RmCanFrame *frame)
{
    if ((unsigned)id>=RM_CAN_COUNT || !frame || !registered) return false;
    CanBus *b=&buses[id];
    RmCriticalState key=RmCritical_Enter();
    bool available=b->head!=b->tail;
    if (available) { *frame=b->ring[b->tail%MC02_CAN_SOFTWARE_DEPTH]; ++b->tail; }
    RmCritical_Exit(key);
    return available;
}
static RmStatus submit(const RmCanFrame *frame,uint32_t epoch,uint32_t deadline_ms,bool zero)
{
    if (!registered || !frame || (unsigned)frame->bus>=RM_CAN_COUNT || frame->length>8U ||
        frame->identifier>(frame->extended?0x1FFFFFFFU:0x7FFU)) return RM_INVALID;
    CanBus *b=&buses[frame->bus];
    RmCriticalState key=RmCritical_Enter();
    uint32_t now=RmTime_NowMs();
    if ((!zero && (!gate_permit || !gate_permit(epoch,now))) || (int32_t)(deadline_ms-now)<=0 ||
        (zero && deadline_ms-now>100U)) {
        RmCritical_Exit(key); return RM_DENIED;
    }
    if (b->fault || !b->started) { RmCritical_Exit(key); return RM_OFFLINE; }
    TxSlot *chosen=NULL;
    for (unsigned i=0;i<3U;++i) {
        TxSlot *s=&b->tx[i];
        if ((s->pending||s->flight) && s->frame.identifier==frame->identifier && s->frame.extended==frame->extended) {
            chosen=s; break;
        }
        if (!chosen && !s->pending && !s->flight) chosen=s;
    }
    if (!chosen) { ++b->stats.tx_busy; RmCritical_Exit(key); return RM_BUSY; }
    chosen->frame=*frame; chosen->epoch=epoch; chosen->deadline_ms=deadline_ms; chosen->pending=true;
    chosen->zero_frame=zero;
    RmCritical_Exit(key);
    return RM_OK;
}
RmStatus RmCan_Submit(const RmCanFrame *frame,uint32_t epoch,uint32_t deadline_ms)
{
    return submit(frame,epoch,deadline_ms,false);
}
RmStatus RmCan_SubmitZeroFrame(const RmCanFrame *frame,uint32_t deadline_ms)
{
    if (!frame || frame->extended || frame->length!=8U) return RM_INVALID;
    for (unsigned i=0;i<8U;++i) if (frame->data[i]!=0U) return RM_DENIED;
    return submit(frame,0U,deadline_ms,true);
}
bool RmCan_GetStats(RmCanId id,RmCanStats *stats)
{
    if ((unsigned)id>=RM_CAN_COUNT || !stats || !registered) return false;
    RmCriticalState key=RmCritical_Enter();
    *stats=buses[id].stats; stats->pending_frames=buses[id].head-buses[id].tail;
    RmCritical_Exit(key);
    return true;
}
void RmCan_CancelMotion(void)
{
    if (!registered) return;
    for (unsigned b=0;b<RM_CAN_COUNT;++b) {
        RmCriticalState key=RmCritical_Enter();
        for (unsigned i=0;i<3U;++i) {
            TxSlot *s=&buses[b].tx[i];
            if (!s->zero_frame) s->pending=false;
            if (s->flight && !s->flight_zero && buses[b].started &&
                HAL_FDCAN_AbortTxRequest(buses[b].handle,buffers[i])==HAL_OK) s->abort_requested=true;
        }
        RmCritical_Exit(key);
    }
}
void RmCan_Service(uint32_t now_ms)
{
    if (!registered) return;
    for (unsigned id=0;id<RM_CAN_COUNT;++id) {
        CanBus *b=&buses[id];
        if (b->fault && b->recover_attempts<3U && (int32_t)(now_ms-b->recover_at)>=0) {
            RmCriticalState key=RmCritical_Enter();
            /* Never restart a controller while an old motion request remains
               pending in hardware, even when its software slot was invalidated. */
            HAL_StatusTypeDef aborted=b->handle->State==HAL_FDCAN_STATE_BUSY ?
                HAL_FDCAN_AbortTxRequest(b->handle,7U) : HAL_OK;
            for (unsigned i=0;i<3U;++i) b->tx[i]=(TxSlot){0};
            b->tail=b->head;
            RmCritical_Exit(key);
            /* Stop/Start have bounded HAL waits; do not hold global interrupt masking. */
            ++b->recover_attempts; b->recover_at=now_ms+100U*b->recover_attempts;
            if (aborted==HAL_OK && HAL_FDCAN_IsTxBufferMessagePending(b->handle,7U)==0U &&
                (b->handle->State==HAL_FDCAN_STATE_READY || HAL_FDCAN_Stop(b->handle)==HAL_OK) &&
                HAL_FDCAN_Start(b->handle)==HAL_OK) {
                key=RmCritical_Enter(); b->fault=false; b->stats.online=true;
                b->started=true;
                ++b->stats.recoveries; RmCritical_Exit(key);
            } else {
                b->started=b->handle->State==HAL_FDCAN_STATE_BUSY;
                ++b->stats.recovery_failures;
            }
        }
        if (b->fault || !b->started) continue;
        RmCriticalState key=RmCritical_Enter();
        for (unsigned i=0;i<3U;++i) {
            TxSlot *s=&b->tx[i];
            /* The service release time is only a recovery/backoff reference.
               Preemption must never extend a hardware request's validity. */
            uint32_t tx_now=RmTime_NowMs();
            bool flight_valid=(s->flight_zero || (gate_permit && gate_permit(s->flight_epoch,tx_now))) &&
                              (int32_t)(s->flight_deadline_ms-tx_now)>0;
            if (s->flight && !flight_valid && !s->abort_requested) {
                if (HAL_FDCAN_AbortTxRequest(b->handle,buffers[i])==HAL_OK) s->abort_requested=true;
            }
            if (!s->pending) continue;
            if ((!s->zero_frame && (!gate_permit || !gate_permit(s->epoch,tx_now))) || (int32_t)(s->deadline_ms-tx_now)<=0) {
                s->pending=false; ++b->stats.tx_expired; continue;
            }
            if (s->flight) continue;
            FDCAN_TxHeaderTypeDef h={0};
            h.Identifier=s->frame.identifier; h.IdType=s->frame.extended?FDCAN_EXTENDED_ID:FDCAN_STANDARD_ID;
            h.TxFrameType=FDCAN_DATA_FRAME; h.DataLength=s->frame.length;
            h.ErrorStateIndicator=FDCAN_ESI_ACTIVE; h.BitRateSwitch=FDCAN_BRS_OFF;
            h.FDFormat=FDCAN_CLASSIC_CAN; h.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
            uint8_t raw[8]={0}; memcpy(raw,s->frame.data,s->frame.length);
            if (HAL_FDCAN_AddMessageToTxBuffer(b->handle,&h,raw,buffers[i])!=HAL_OK) {
                ++b->stats.tx_busy; continue;
            }
            tx_now=RmTime_NowMs();
            if ((!s->zero_frame && (!gate_permit || !gate_permit(s->epoch,tx_now))) ||
                (int32_t)(s->deadline_ms-tx_now)<=0) {
                s->pending=false; ++b->stats.tx_expired; continue;
            }
            if (HAL_FDCAN_EnableTxBufferRequest(b->handle,buffers[i])==HAL_OK) {
                s->flight=true; s->pending=false; s->abort_requested=false;
                s->flight_epoch=s->epoch; s->flight_deadline_ms=s->deadline_ms;
                s->flight_zero=s->zero_frame;
                ++b->stats.tx_accepted;
            } else ++b->stats.tx_busy;
        }
        RmCritical_Exit(key);
    }
}
