#ifndef RM_TIME_H
#define RM_TIME_H

#include <stdbool.h>
#include <stdint.h>

/** Return the monotonic platform clock in milliseconds. */
uint32_t RmTime_NowMs(void);

/** Return the high-resolution monotonic platform clock in microseconds. */
uint64_t RmTime_NowUs(void);

/**
 * Return the elapsed milliseconds between two samples.
 *
 * Unsigned subtraction keeps this operation valid across a uint32_t wrap.
 */
static inline uint32_t RmTime_ElapsedMs(uint32_t now_ms, uint32_t start_ms)
{
    return now_ms - start_ms;
}

/**
 * Test a millisecond deadline using wrap-safe serial-number arithmetic.
 * Deadlines must be less than 2^31 ms into the future.
 */
static inline bool RmTime_DeadlineReached(uint32_t now_ms,
                                         uint32_t deadline_ms)
{
    return (uint32_t)(now_ms - deadline_ms) < UINT32_C(0x80000000);
}

#endif /* RM_TIME_H */
