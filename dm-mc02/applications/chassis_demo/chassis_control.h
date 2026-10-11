#ifndef RM_CHASSIS_CONTROL_H
#define RM_CHASSIS_CONTROL_H
#include "remote_control_state.h"
#include "ins.h"
typedef enum { RM_CHASSIS_FOLLOW,RM_CHASSIS_AUTO_AIM,RM_CHASSIS_SPIN } RmChassisMode;
typedef struct {
    bool valid,enable,safe_position,arm_event;
    RmChassisMode mode;
    float x,y,yaw;
} RmChassisIntent;
typedef struct {
    bool active,recovery_pending;
    RmChassisMode mode;
    float target_yaw_rad,filtered_vx,filtered_vy,filtered_wz,integral;
} RmChassisController;
typedef struct {
    float yaw_rad,target_yaw_rad,yaw_error_rad,yaw_rate_rad_s;
    float vx_m_s,vy_m_s,wz_rad_s,translation_scale,wheel_ref_rad_s[4];
    RmChassisMode mode;
} RmChassisOutput;
bool RmChassis_MapEt08(const RemoteControlState *,RmChassisIntent *);
bool RmChassis_ConfigValid(void);
void RmChassis_Reset(RmChassisController *);
/* Platform-independent yaw + omniwheel reference calculation. */
bool RmChassis_Step(RmChassisController *,const RmChassisIntent *,const RmInsSnapshot *,float dt_s,RmChassisOutput *);
#endif
