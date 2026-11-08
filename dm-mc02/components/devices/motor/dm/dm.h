#ifndef RM_DM_H
#define RM_DM_H
#include "rm_platform.h"
typedef struct { float position, velocity, torque; } RmDmLimits;
typedef struct {
    uint64_t timestamp_us;
    uint32_t generation;
    float position_rad, velocity_rad_s, torque_nm;
    uint8_t state, mos_temperature, rotor_temperature;
} RmDmFeedback;
bool RmDm_LimitsValid(const RmDmLimits *limits);
bool RmDm_Decode(const RmCanFrame *frame, uint16_t master_id, uint8_t motor_id,
                 const RmDmLimits *limits, RmDmFeedback *out);
bool RmDm_TorqueFrame(RmCanFrame *frame, RmCanId bus, uint16_t motor_id,
                      const RmDmLimits *limits, float torque_nm);
void RmDm_SpecialFrame(RmCanFrame *frame, RmCanId bus, uint16_t motor_id, bool enable);
void RmDm_ReadRegister(RmCanFrame *frame, RmCanId bus, uint16_t motor_id, uint8_t reg);
bool RmDm_RegisterReply(const RmCanFrame *frame, uint16_t master_id, uint16_t motor_id,
                        uint8_t *reg, uint32_t *bits);
/* Exact whitelist for gate-independent disable and read-only parameter requests. */
bool RmDm_ServiceFrameSafe(const RmCanFrame *frame);
/* The only commissioned write: volatile TIMEOUT=1000 counts (50 ms). */
void RmDm_TimeoutFrame(RmCanFrame *frame,RmCanId bus,uint16_t motor_id);
bool RmDm_TimeoutFrameSafe(const RmCanFrame *frame);
#endif
