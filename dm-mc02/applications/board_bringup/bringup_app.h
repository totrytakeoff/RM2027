#ifndef RM_BRINGUP_APP_H
#define RM_BRINGUP_APP_H
#include "runtime.h"
#include "ins.h"
#include "remote.h"
#include "dji.h"
const RmRuntimeConfig *RmBringup_Init(void);
/* Optional bounded I/O observer, configured before runtime starts. */
void RmBringup_SetCanObserver(void (*observer)(const RmCanFrame *));
bool RmBringup_ReadIns(RmInsSnapshot *);
bool RmBringup_ReadRemote(RemoteControlState *);
/* One control consumer; atomically consumes accumulated published edges. */
bool RmBringup_TakeRemote(RemoteControlState *);
/* Passive 0x201..0x20b candidates are not proof of motor model/identity. */
bool RmBringup_ReadDjiCandidate(RmCanId bus,uint32_t id,RmDjiFeedback *);
/* Copied I/O mailbox with current transport generation and feedback TTL. */
bool RmBringup_ReadMotor(unsigned motor,RmDjiFeedback *);
/* CONTROL may capture a new zero only while the output gate is stopped. */
bool RmBringup_ZeroMotorPosition(unsigned motor);
/* One control producer; atomically publishes a complete leased command set. */
bool RmBringup_SubmitMotorCommands(const RmDjiCommand *,unsigned count,uint32_t epoch);
#endif
