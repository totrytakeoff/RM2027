#ifndef RM_DJI_H
#define RM_DJI_H
#include "rm_platform.h"
#include "controller.h"
#define RM_DJI_MAX_MOTORS 12U
#define RM_DJI_MAX_GROUPS 9U
typedef enum { RM_DJI_M2006,RM_DJI_M3508,RM_DJI_GM6020_VOLTAGE } RmDjiModel;
enum { RM_DJI_LOOP_CURRENT=1U,RM_DJI_LOOP_SPEED=2U,RM_DJI_LOOP_ANGLE=4U };
enum { RM_DJI_EXTERNAL_ANGLE=1U,RM_DJI_EXTERNAL_SPEED=2U };
typedef struct {
    RmCanId bus;
    RmDjiModel model;
    uint8_t id,loops;
    int8_t reference_sign,output_sign,angle_sign,speed_sign;
    bool speed_in_rad_s,allow_direct;
    uint32_t feedback_timeout_ms,command_timeout_ms;
    PID_Init_Config_s angle_pid,speed_pid,current_pid;
} RmDjiConfig;
typedef struct {
    uint64_t timestamp_us;
    uint32_t generation,sequence;
    uint16_t encoder;
    int16_t rpm,current_raw,current_filtered;
    uint8_t temperature;
    float speed_deg_s,speed_rad_s,angle_deg,angle_rad;
    int64_t encoder_total;
    bool valid,position_valid;
} RmDjiFeedback;
typedef struct { RmDjiFeedback value; bool initialized; } RmDjiFeedbackState;
/* Decodes a standard 8-byte candidate; model/ID matching belongs to registry. */
bool RmDji_Decode(RmDjiFeedbackState *,const RmCanFrame *,uint32_t max_gap_ms);
typedef struct {
    float reference,speed_ff,current_ff,external_angle,external_speed;
    uint32_t epoch,deadline_ms,external_time_ms;
    uint8_t outer_loop,external_mask,reset_mask;
    bool enabled;
} RmDjiCommand;
typedef struct {
    RmDjiConfig config;
    uint32_t feedback_id,command_id;
    uint8_t command_slot;
    RmDjiFeedbackState feedback;
    PIDInstance angle_pid,speed_pid,current_pid;
    float output;
    bool active;
} RmDjiMotor;
typedef struct { RmCanFrame frame; uint32_t epoch,deadline_ms; } RmDjiGroup;
typedef struct {
    RmDjiMotor motors[RM_DJI_MAX_MOTORS];
    unsigned count;
    uint32_t bad_feedback;
} RmDjiRegistry;
/* Assembly initializes once. Each mutation is serialized by its task owner. */
bool RmDji_Init(RmDjiRegistry *,const RmDjiConfig *,unsigned count);
bool RmDji_Receive(RmDjiRegistry *,const RmCanFrame *);
bool RmDji_Read(const RmDjiRegistry *,unsigned motor,uint64_t now_us,RmDjiFeedback *);
bool RmDji_AllOnline(const RmDjiRegistry *,uint64_t now_us);
/* Explicit new position reference after interrupted feedback; stopped owner only. */
bool RmDji_RebasePosition(RmDjiRegistry *,unsigned motor,int64_t encoder_total,uint64_t now_us);
unsigned RmDji_Step(RmDjiRegistry *,const RmDjiCommand *,unsigned command_count,
    uint64_t now_us,float dt_s,bool permit,uint32_t epoch,uint32_t gate_deadline_ms,
    RmDjiGroup groups[RM_DJI_MAX_GROUPS]);
#endif
