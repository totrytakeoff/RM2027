#ifndef RM_PLATFORM_H
#define RM_PLATFORM_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint32_t RmCriticalState;
RmCriticalState RmCritical_Enter(void);
void RmCritical_Exit(RmCriticalState state);
uint64_t RmTime_NowUs(void);
uint32_t RmTime_NowMs(void);
uint32_t RmTime_Cycles(void);
uint32_t RmTime_CyclesToUs(uint32_t cycles);
void RmTime_DelayUs(uint32_t us);

typedef enum { RM_OK, RM_BUSY, RM_INVALID, RM_OFFLINE, RM_DENIED, RM_ERROR } RmStatus;
typedef enum { RM_SERIAL_REMOTE, RM_SERIAL_VISION, RM_SERIAL_REFEREE, RM_SERIAL_CONSOLE, RM_SERIAL_COUNT } RmSerialId;
#define RM_SERIAL_CHUNK_BYTES 64U
typedef struct {
    uint64_t timestamp_us;
    uint32_t generation; /* changes on loss/error: consumers must reset stream parsers */
    uint16_t length;
    uint8_t data[RM_SERIAL_CHUNK_BYTES];
} RmSerialChunk;
typedef struct {
    uint32_t rx_bytes, rx_events, rx_dropped, rx_errors, restarts, restart_failures;
    uint32_t tx_accepted, tx_completed, tx_errors, tx_busy;
    uint32_t generation, pending_chunks;
    uint32_t abort_timeouts, recovery_exhausted;
    /* Quarantine is terminal until MCU reset; DMA storage is never recycled. */
    bool online, quarantined;
} RmSerialStats;
bool RmSerial_Read(RmSerialId id, RmSerialChunk *chunk);
RmStatus RmSerial_TrySendCopy(RmSerialId id, const void *data, size_t length);
bool RmSerial_GetStats(RmSerialId id, RmSerialStats *stats);
void RmSerial_Service(uint32_t now_ms);

typedef enum { RM_CAN_1, RM_CAN_2, RM_CAN_3, RM_CAN_COUNT } RmCanId;
typedef struct {
    uint64_t timestamp_us;
    uint32_t identifier, generation;
    uint8_t data[8];
    uint8_t length;
    bool extended;
    RmCanId bus;
} RmCanFrame;
typedef struct {
    uint32_t rx_frames, rx_dropped, rx_errors, tx_accepted, tx_completed, tx_aborted;
    uint32_t tx_expired, tx_busy, bus_off_count, recoveries, recovery_failures;
    uint32_t generation, pending_frames;
    bool online;
} RmCanStats;
bool RmCan_Read(RmCanId bus, RmCanFrame *frame);
/* Bound once during assembly; null hooks deny all motion output. */
void RmCan_SetSafetyHooks(bool (*permit)(uint32_t epoch, uint32_t now_ms),
                          void (*stop)(uint32_t reason));
/* Copy a classic motion command. Requires a live output-gate epoch and deadline. */
RmStatus RmCan_Submit(const RmCanFrame *frame, uint32_t epoch, uint32_t deadline_ms);
/* Gate-independent all-zero standard 8-byte payload, bounded to 100 ms.
   Application must bind IDs for protocols where this payload means zero output. */
RmStatus RmCan_SubmitZeroFrame(const RmCanFrame *frame,uint32_t deadline_ms);
void RmCan_CancelMotion(void);
bool RmCan_GetStats(RmCanId bus, RmCanStats *stats);
void RmCan_Service(uint32_t now_ms);

typedef enum { RM_IMU_ACCEL, RM_IMU_GYRO } RmImuChip;
typedef struct {
    void *context;
    RmStatus (*transfer)(void *context, RmImuChip chip, const uint8_t *tx,
                         uint8_t *rx, size_t length);
    uint64_t (*now_us)(void *context);
    void (*delay_us)(void *context, uint32_t us);
} RmSensorBus;
const RmSensorBus *RmBoard_ImuBus(void);
void RmBoard_HeaterOff(void);
bool RmBoard_KeyPressed(void);
typedef struct {
    uint32_t cpu_hz, hclk_hz, fdcan_hz, reset_flags, cpuid;
    uint32_t hal_version;
    bool dcache_enabled;
} RmBoardInfo;
void RmBoard_Init(void);
const RmBoardInfo *RmBoard_GetInfo(void);
/* Success means endpoints/storage are registered. A failed initial RX remains
   offline and follows bounded Service recovery; inspect GetStats for readiness. */
bool RmSerial_Start(void);
bool RmCan_Start(void);
bool RmWatchdog_Start(void);
void RmWatchdog_Refresh(void);
typedef struct {
    uint32_t reason, ipsr, cfsr, hfsr, mmfar, bfar;
    uint32_t frame[8]; /* r0-r3, r12, lr, pc, xpsr; zero when stack is unreadable */
} RmFaultInfo;
/* Startup consumer: validates retained SRAM record, consumes it once. */
bool RmFault_ReadPrevious(RmFaultInfo *out);
_Noreturn void RmFault_Panic(uint32_t reason);
#endif
