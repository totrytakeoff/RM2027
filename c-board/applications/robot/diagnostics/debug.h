/**
 * @file debug.h
 * @brief 最小框架统一调试输出层
 */

#ifndef ROBOT_DEBUG_H
#define ROBOT_DEBUG_H

#include <stdint.h>
#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MINIMAL_DEBUG_TEXT_STREAM_ACTIVE \
    (MINIMAL_DEBUG_ENABLE && ((MINIMAL_DEBUG_MODE & MINIMAL_DEBUG_MODE_TEXT) != 0) && \
     ((((MINIMAL_DEBUG_MODE & MINIMAL_DEBUG_MODE_VOFA) == 0) || MINIMAL_DEBUG_ALLOW_MIXED_STREAM)))

#define MINIMAL_DEBUG_VOFA_STREAM_ACTIVE \
    (MINIMAL_DEBUG_ENABLE && ((MINIMAL_DEBUG_MODE & MINIMAL_DEBUG_MODE_VOFA) != 0))

void MinimalDebug_Init(void);
void MinimalDebug_UpdatePeriodic(uint32_t now_ms);
void MinimalDebug_LogEventSystem(const char *fmt, ...);
void MinimalDebug_LogEventInput(const char *fmt, ...);
void MinimalDebug_LogEventChassis(const char *fmt, ...);
void MinimalDebug_LogEventGimbal(const char *fmt, ...);
void MinimalDebug_LogEventShoot(const char *fmt, ...);
void MinimalDebug_PublishVofaFrame(void);

#ifdef __cplusplus
}
#endif

#if MINIMAL_DEBUG_TEXT_STREAM_ACTIVE && MINIMAL_DEBUG_MOD_SYSTEM
#define MDBG_SYS(...) MinimalDebug_LogEventSystem(__VA_ARGS__)
#else
#define MDBG_SYS(...)
#endif

#if MINIMAL_DEBUG_TEXT_STREAM_ACTIVE && MINIMAL_DEBUG_MOD_INPUT
#define MDBG_IN(...) MinimalDebug_LogEventInput(__VA_ARGS__)
#else
#define MDBG_IN(...)
#endif

#if MINIMAL_DEBUG_TEXT_STREAM_ACTIVE && MINIMAL_DEBUG_MOD_CHASSIS
#define MDBG_CHS(...) MinimalDebug_LogEventChassis(__VA_ARGS__)
#else
#define MDBG_CHS(...)
#endif

#if MINIMAL_DEBUG_TEXT_STREAM_ACTIVE && MINIMAL_DEBUG_MOD_GIMBAL
#define MDBG_GMB(...) MinimalDebug_LogEventGimbal(__VA_ARGS__)
#else
#define MDBG_GMB(...)
#endif

#if MINIMAL_DEBUG_TEXT_STREAM_ACTIVE && MINIMAL_DEBUG_MOD_SHOOT
#define MDBG_SHT(...) MinimalDebug_LogEventShoot(__VA_ARGS__)
#else
#define MDBG_SHT(...)
#endif

#endif /* ROBOT_DEBUG_H */
