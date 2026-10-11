# VOFA+ JustFloat 通道说明（infantry_chassis_gimbal_et08_demo）

文档更新日期：2026-07-19

当前固件在 [main.c](main.c) 的 `SendVofaFrame()` 以 **JustFloat** 协议输出 32 通道，顺序固定如下：

1. `time_ms`：系统时间戳（ms）
2. `et08_online`：遥控在线状态（0/1）
3. `chassis_mode`：底盘模式（0=FOLLOW, 1=SEPARATE）
4. `yaw_hold_ref`：云台 Yaw 角目标，世界系累计角（deg）
5. `imu_yaw_total`：云台 IMU 累计 Yaw（deg）
6. `imu_yaw`：云台 IMU 单圈 Yaw（deg）
7. `imu_gyro_z`：云台 IMU Z 轴角速度（deg/s）
8. `yaw_motor_total_angle`：Yaw 电机总角度（deg）
9. `yaw_motor_single_round`：Yaw 电机单圈角度（deg）
10. `yaw_offset_deg`：云台相对底盘夹角（deg）
11. `gimbal_vx_cmd`：左摇杆给出的云台坐标系前后速度命令（m/s）
12. `gimbal_vy_cmd`：左摇杆给出的云台坐标系左右速度命令（m/s）
13. `body_vx_cmd`：旋转到底盘坐标系后的前后速度命令（m/s）
14. `body_vy_cmd`：旋转到底盘坐标系后的左右速度命令（m/s）
15. `chassis_wz_cmd`：底盘角速度命令（rad/s）
16. `filtered_vx`：滤波后的底盘前后速度（m/s）
17. `filtered_vy`：滤波后的底盘左右速度（m/s）
18. `filtered_wz`：滤波后的底盘角速度（rad/s）
19. `wheel_fr_ref`：前右轮速度参考（deg/s）
20. `wheel_fr_fdb`：前右轮速度反馈（deg/s）
21. `wheel_fl_ref`：前左轮速度参考（deg/s）
22. `wheel_fl_fdb`：前左轮速度反馈（deg/s）
23. `wheel_br_ref`：后右轮速度参考（deg/s）
24. `wheel_br_fdb`：后右轮速度反馈（deg/s）
25. `wheel_bl_ref`：后左轮速度参考（deg/s）
26. `wheel_bl_fdb`：后左轮速度反馈（deg/s）
27. `pitch_speed_ref`：Pitch 速度参考（deg/s）
28. `pitch_hold_ref`：Pitch 保持目标角（deg）
29. `pitch_motor_total_angle`：Pitch 电机总角度（deg）
30. `imu_pitch`：云台 IMU Pitch（deg）
31. `imu_gyro_x`：云台 IMU X 轴角速度（deg/s）
32. `pitch_current_ff`：Pitch 重力前馈输出

帧尾固定：`0x00 0x00 0x80 0x7F`

## 重点观察组合

### 1. 验证云台 IMU 闭环

看：

- `yaw_hold_ref`
- `imu_yaw_total`
- `imu_gyro_z`

期望：

- 推右摇杆左右时，`yaw_hold_ref` 变化
- `imu_yaw_total` 跟随 `yaw_hold_ref`
- 松杆后 `imu_gyro_z` 很快回到接近 0

### 2. 验证底盘相对角定义

看：

- `yaw_motor_single_round`
- `yaw_offset_deg`
- `body_vx_cmd`
- `body_vy_cmd`

期望：

- 只转云台不动底盘时，`yaw_offset_deg` 跟随变化
- 云台偏 90 度时，前推左摇杆，`body_vx_cmd/body_vy_cmd` 应发生明显坐标旋转

### 3. 验证 FOLLOW

看：

- `chassis_mode`
- `yaw_offset_deg`
- `chassis_wz_cmd`
- `filtered_wz`

期望：

- `FOLLOW` 下，只要 `yaw_offset_deg` 非零，`chassis_wz_cmd` 就应出现补偿
- 底盘跟上后，`yaw_offset_deg` 逐步收敛到 0 附近

### 4. 验证 SEPARATE

看：

- `chassis_mode`
- `yaw_offset_deg`
- `chassis_wz_cmd`
- `body_vx_cmd/body_vy_cmd`

期望：

- `SEPARATE` 下 `chassis_wz_cmd` 基本为 0
- 但 `yaw_offset_deg` 仍参与平移变换，`body_vx_cmd/body_vy_cmd` 会随云台朝向变化

### 5. 验证轮速链路

看：

- `filtered_vx/filtered_vy/filtered_wz`
- 四个 `wheel_*_ref`
- 四个 `wheel_*_fdb`

期望：

- `ref` 和 `fdb` 趋势一致
- `FOLLOW` 下只有转向时，四轮参考应呈现明显差速
- 纯平移时，轮速组合应符合全向轮逆解的对称关系

## 建议的 VOFA 页面

建议分 4 组曲线：

1. 姿态组  
   `yaw_hold_ref`, `imu_yaw_total`, `yaw_offset_deg`

2. 底盘控制组  
   `gimbal_vx_cmd`, `gimbal_vy_cmd`, `body_vx_cmd`, `body_vy_cmd`, `chassis_wz_cmd`

3. 轮速组  
   四个 `wheel_*_ref` 和四个 `wheel_*_fdb`

4. Pitch 组  
   `pitch_speed_ref`, `pitch_hold_ref`, `pitch_motor_total_angle`, `imu_pitch`, `pitch_current_ff`

## 使用提醒

1. 这份通道顺序已经固定，后续扩展只允许在末尾追加。
2. 若要同时看文本日志和 VOFA，建议优先用 VOFA，文本只保留低频状态确认。
3. 如果 `yaw_offset_deg` 方向与预期相反，先不要急着改 PID，先校正 `YAW_CHASSIS_ALIGN_ECD` 或机械方向定义。
