#ifndef ROBOT_GIMBAL_UNITS_H
#define ROBOT_GIMBAL_UNITS_H

#include <math.h>
#include <stdint.h>
#include <string.h>

/**
 * @brief 判定 float 是否为有限值（IEEE-754 位模式）。
 *
 * 控制安全路径使用纯整数运算，不依赖编译器对浮点有限值的假设。
 * 即使后续构建选项改变，外部数据和传感器数据仍能可靠拒绝 NaN/Inf。
 */
static inline int Gimbal_FloatIsFinite(float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000U) != 0x7F800000U;
}

/** 安静的 NaN，用于表示"本帧解算结果无定义"。 */
static inline float Gimbal_NaNF(void)
{
    float v;
    const uint32_t bits = 0x7FC00000U;

    memcpy(&v, &bits, sizeof(v));
    return v;
}

/** Convert the INS angular-rate contract (rad/s) to motor-loop deg/s. */
static inline float Gimbal_RadPerSecToDegPerSec(float rate_rad_s)
{
    return rate_rad_s * 57.29577951308232f;
}

/** 视觉帧中的目标角速度单位为 rad/s，转成云台环使用的 deg/s。 */
static inline float Gimbal_RadToDeg(float angle_rad)
{
    return angle_rad * 57.29577951308232f;
}

/**
 * @brief 把视觉下发的"相对目标方位"换算成本车 IMU 累计 Yaw 参考。
 *
 * 上位机 tracker 的 target_frame 必须为 gimbal_link（见 docs/vision/
 * vision_protocol.md §1.3），因此 (x, y) 就是目标相对云台**当前指向**的
 * 方位。绕 Z 轴俯视逆时针为正，与 IMU 累计 Yaw 同向。
 *
 * gimbal.c 只用其差值（yaw_target - yaw_now），因此传 yaw_hold_ref 进来自洽。
 *
 * @param imu_yaw_now_deg 当前 IMU 累计 Yaw，deg
 * @param target_x        云台系目标 X，前为正，m
 * @param target_y        云台系目标 Y，左为正，m
 * @return 本车 IMU 累计 Yaw 参考，deg；输入退化时返回当前值
 */
static inline float Gimbal_TargetYawReferenceDeg(float imu_yaw_now_deg,
                                                 float target_x,
                                                 float target_y)
{
    if (!Gimbal_FloatIsFinite(imu_yaw_now_deg) ||
        !Gimbal_FloatIsFinite(target_x) ||
        !Gimbal_FloatIsFinite(target_y)) {
        return imu_yaw_now_deg;
    }
    if ((target_x == 0.0f) && (target_y == 0.0f)) {
        return imu_yaw_now_deg; /* 方位无定义，保持当前指向 */
    }
    return imu_yaw_now_deg + atan2f(target_y, target_x) * 57.29577951308232f;
}

/**
 * @brief 以限幅后的增量推进一个**多圈累计角**参考，deg。
 *
 * 刻意不做任何角度归一化：`yaw_hold_ref` 与 `attitude.YawTotalAngle` 同为
 * 多圈累计角，若在此折到 [-180,180] 会在跨越边界时产生 360 deg 阶跃。
 * 增量本身无界问题由 limit_deg 限制。
 *
 * @param ref_deg   当前参考，deg（多圈累计）
 * @param delta_deg 期望增量，deg
 * @param limit_deg 单次增量绝对值上限，deg（<=0 或非有限则视为 0）
 */
static inline float Gimbal_AdvanceAngleRefDeg(float ref_deg, float delta_deg,
                                              float limit_deg)
{
    if (!Gimbal_FloatIsFinite(ref_deg) || !Gimbal_FloatIsFinite(delta_deg)) {
        return ref_deg;
    }
    if (!Gimbal_FloatIsFinite(limit_deg) || limit_deg <= 0.0f) {
        return ref_deg;
    }
    if (delta_deg > limit_deg) {
        delta_deg = limit_deg;
    } else if (delta_deg < -limit_deg) {
        delta_deg = -limit_deg;
    }
    return ref_deg + delta_deg;
}

/**
 * @brief 云台系目标仰角相对修正量（本车 IMU Pitch 符号：抬头为负），deg。
 *
 * 上位机 Z 轴向上为正，本车 IMU 抬头为负，故取反。
 * 目标坐标处于当前云台系，因此返回的是相对当前指向的 Pitch 增量，
 * 由调用方以限速方式累加到保持参考，而不是绝对 IMU 角度。
 * 水平距离过小时返回 NaN 供调用方判定放弃该帧。
 * 用 Gimbal_NaNF() 表示本帧解算无定义。
 * 函数名保留 Reference 以兼容现有主机测试；返回值语义是相对增量。
 */
static inline float Gimbal_TargetPitchReferenceDeg(float target_x,
                                                   float target_y,
                                                   float target_z,
                                                   float min_horizontal_m)
{
    float horizontals_sq;

    if (!Gimbal_FloatIsFinite(target_x) || !Gimbal_FloatIsFinite(target_y) ||
        !Gimbal_FloatIsFinite(target_z)) {
        return Gimbal_NaNF();
    }

    horizontals_sq = (target_x * target_x) + (target_y * target_y);
    if (horizontals_sq <= (min_horizontal_m * min_horizontal_m)) {
        return Gimbal_NaNF(); /* 方位退化 */
    }
    return -atan2f(target_z, sqrtf(horizontals_sq)) * 57.29577951308232f;
}

/** 世界系云台角速度 = 底盘基座角速度 + 云台相对底盘角速度。 */
static inline float Gimbal_EstimateBaseRateRadS(
    float imu_yaw_rate_rad_s,
    float motor_relative_rate_rad_s)
{
    if (!Gimbal_FloatIsFinite(imu_yaw_rate_rad_s) ||
        !Gimbal_FloatIsFinite(motor_relative_rate_rad_s)) {
        return 0.0f;
    }
    return imu_yaw_rate_rad_s - motor_relative_rate_rad_s;
}

/**
 * Calculate gravity current feedforward from the gravity-referenced IMU pitch.
 * horizontal_deg is the IMU reading when the barrel is mechanically level.
 */
static inline float Gimbal_GravityFeedforward(float pitch_imu_deg,
                                                       float horizontal_deg,
                                                       float gain,
                                                       float max_abs)
{
    float output;

    if (!Gimbal_FloatIsFinite(pitch_imu_deg) ||
        !Gimbal_FloatIsFinite(horizontal_deg) ||
        !Gimbal_FloatIsFinite(gain) || !Gimbal_FloatIsFinite(max_abs) ||
        max_abs < 0.0f) {
        return 0.0f;
    }
    output = gain * cosf((pitch_imu_deg - horizontal_deg) *
                         0.01745329251994329577f);
    if (output > max_abs) {
        return max_abs;
    }
    if (output < -max_abs) {
        return -max_abs;
    }
    return output;
}

/**
 * 根据已估算的底盘基座角速度计算扰动补偿电流。
 * 正基座角速度需要负电流前馈抵消。
 */
static inline float Gimbal_BaseRateCurrentFeedforward(
    float base_rate_rad_s,
    float gain,
    float deadband_rad_s,
    float max_abs)
{
    float output;

    if (!Gimbal_FloatIsFinite(base_rate_rad_s) ||
        !Gimbal_FloatIsFinite(gain) || gain < 0.0f ||
        !Gimbal_FloatIsFinite(deadband_rad_s) || deadband_rad_s < 0.0f ||
        !Gimbal_FloatIsFinite(max_abs) || max_abs < 0.0f) {
        return 0.0f;
    }
    if (fabsf(base_rate_rad_s) <= deadband_rad_s) {
        return 0.0f;
    }
    output = -gain * base_rate_rad_s;
    if (output > max_abs) {
        return max_abs;
    }
    if (output < -max_abs) {
        return -max_abs;
    }
    return output;
}

#endif /* ROBOT_GIMBAL_UNITS_H */
