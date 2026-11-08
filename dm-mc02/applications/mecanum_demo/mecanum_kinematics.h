#ifndef ROBOT_MECANUM_KINEMATICS_H
#define ROBOT_MECANUM_KINEMATICS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 将云台坐标系平移指令旋转到底盘坐标系。 */
void Mecanum_RotateToBody(float gimbal_vx,
                                  float gimbal_vy,
                                  float yaw_offset_rad,
                                  float *body_vx,
                                  float *body_vy);

/** 按向量等比例限制平移速度，避免对角输入超过执行层总速度。 */
void Mecanum_LimitTranslation(float *vx,
                                      float *vy,
                                      float max_speed);

/** X 型麦轮逆运动学，输出 FR/FL/BR/BL 向前为正的轮端 rad/s；wheel_base=长+宽。 */
void Mecanum_Inverse(float vx,
                                 float vy,
                                 float wz,
                                 float wheel_base,
                                 float wheel_radius,
                                 float output[4]);

/** 将轮端 rad/s 按减速比换算为电机转子 rad/s。 */
float Mecanum_WheelToMotorSpeedRadS(float wheel_speed_rad_s,
                                            float reduction_ratio);

/** 四轮共同等比例缩放，保持合成运动方向不被逐轮削顶破坏。 */
void Mecanum_NormalizeWheelSpeeds(float speeds[4],
                                          float max_abs_speed);

/**
 * 保留旋转轮速，并按四轮剩余余量统一缩放平移轮速。
 * translation_scale 返回实际保留的平移比例，范围 0~1。
 */
bool Mecanum_CombineWheelSpeedsPreserveRotation(
    const float translation[4],
    const float rotation[4],
    float max_abs_speed,
    float output[4],
    float *translation_scale);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_MECANUM_KINEMATICS_H */
