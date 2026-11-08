#ifndef RM_MECANUM_CONTROL_H
#define RM_MECANUM_CONTROL_H
#include "remote_control_state.h"
#include "ins.h"
typedef enum { RM_MECANUM_FOLLOW,RM_MECANUM_AUTO_AIM,RM_MECANUM_SPIN } RmMecanumMode;
typedef struct {
    bool valid,enable,safe_position,arm_event;
    RmMecanumMode mode;
    float x,y,yaw;
} RmMecanumIntent;
typedef struct {
    bool active,recovery_pending;
    RmMecanumMode mode;
    float target_yaw_rad,filtered_vx,filtered_vy,filtered_wz,integral;
} RmMecanumController;
typedef struct {
    float yaw_rad,target_yaw_rad,yaw_error_rad,yaw_rate_rad_s;
    float vx_m_s,vy_m_s,wz_rad_s,translation_scale,wheel_ref_rad_s[4];
    RmMecanumMode mode;
} RmMecanumOutput;
bool RmMecanum_MapEt08(const RemoteControlState *,RmMecanumIntent *);
bool RmMecanum_ConfigValid(void);
void RmMecanum_Reset(RmMecanumController *);
/* Platform-independent yaw + omniwheel reference calculation. */
bool RmMecanum_Step(RmMecanumController *,const RmMecanumIntent *,const RmInsSnapshot *,float dt_s,RmMecanumOutput *);
#endif
