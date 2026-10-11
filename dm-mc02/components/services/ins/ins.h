#ifndef RM_INS_H
#define RM_INS_H
#include "bmi088.h"
#include "QuaternionEKF.h"
typedef enum { RM_INS_CALIBRATING,RM_INS_RUNNING,RM_INS_FAILED } RmInsState;
typedef enum { RM_INS_ERROR_NONE,RM_INS_ERROR_CONFIG,RM_INS_ERROR_TIMEOUT,
    RM_INS_ERROR_SENSOR,RM_INS_ERROR_TIME,RM_INS_ERROR_NUMERIC } RmInsError;
typedef struct {
    /* Signed permutation: {1,2,3} is identity. Must be a proper rotation. */
    int8_t axes[3];
    uint32_t calibration_samples,calibration_timeout_us,freshness_us;
    float min_dt_s,max_dt_s,stationary_gyro_rad_s,gravity_tolerance;
    float q1,q2,r,lambda,accel_lpf_s,motion_lpf_s;
} RmInsConfig;
typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence,calibration_count;
    RmInsState state;
    RmInsError error;
    bool valid,converged;
    float quaternion[4],gyro[3],accel[3],gyro_bias[3];
    float yaw_deg,pitch_deg,roll_deg,yaw_total_deg,temperature_c,dt_s;
    float motion_body[3],motion_earth[3];
} RmInsSnapshot;
typedef struct {
    RmInsConfig config;
    QEKF_INS_t filter;
    RmInsSnapshot snapshot;
    uint64_t start_us,last_us;
    uint32_t raw_sequence;
    bool have_raw;
    float accel_sum[3],gyro_sum[3],bias[3];
} RmIns;
/* Single owner and stable address (workspace contains internal pointers).
   Assembly layer publishes coherent snapshots; do not copy initialized instances. */
bool RmIns_Init(RmIns *,const RmInsConfig *,uint64_t now_us);
void RmIns_Push(RmIns *,const RmImuSample *,uint64_t now_us);
void RmIns_Poll(RmIns *,uint64_t now_us);
void RmIns_Fail(RmIns *,RmInsError);
bool RmIns_Read(const RmIns *,uint64_t now_us,RmInsSnapshot *);
#endif
