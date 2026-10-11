# VOFA+ JustFloat 通道说明（infantry_gimbal_et08_demo）

当前固件在 `main.c` 的 `SendVofaFrame()` 以 **JustFloat** 协议输出 39 通道，顺序固定如下：

1. `time_ms`：系统时间戳（ms）
2. `et08_online`：遥控在线状态（0/1）
3. `gimbal_mode`：模式（0=FOLLOW, 1=SEPARATE）
4. `yaw_loop_mode`：Yaw环路（0=ANGLE保持, 1=SPEED控制, 2=BRAKE刹车过渡）
5. `pitch_loop_mode`：Pitch环路（0=ANGLE保持, 1=SPEED控制, 2=BRAKE刹车过渡）
6. `yaw_speed_ref`：Yaw速度参考（deg/s）
7. `yaw_speed_fdb`：Yaw速度反馈（deg/s）
8. `yaw_hold_ref`：Yaw保持角目标（deg）
9. `pitch_speed_ref`：Pitch速度参考（deg/s）
10. `pitch_speed_fdb`：Pitch速度反馈（deg/s）
11. `pitch_hold_ref`：Pitch保持角目标（deg）
12. `pitch_total_angle`：Pitch电机总角度反馈（deg）
13. `pitch_angle_ref`：Pitch角度环参考（deg）
14. `pitch_angle_fdb`：Pitch角度环反馈（deg）
15. `pitch_angle_err`：Pitch角度环误差（deg）
16. `pitch_angle_out`：Pitch角度环输出，实际是给速度环的参考（deg/s）
17. `pitch_angle_max_out`：Pitch角度环 `MaxOut`
18. `pitch_speed_pid_ref`：Pitch速度环参考（deg/s）
19. `pitch_speed_pid_fdb`：Pitch速度环反馈（deg/s）
20. `pitch_speed_pid_err`：Pitch速度环误差（deg/s）
21. `pitch_speed_pid_out`：Pitch速度环输出
22. `pitch_angle_sat_ratio`：Pitch角度环输出占上限比例，`1.0` 附近表示快撞上限
23. `yaw_total_angle`：Yaw电机总角度反馈（deg）
24. `yaw_angle_ref`：Yaw角度环参考（deg）
25. `yaw_angle_fdb`：Yaw角度环反馈（deg）
26. `yaw_angle_err`：Yaw角度环误差（deg）
27. `yaw_angle_out`：Yaw角度环输出，实际是给速度环的参考（deg/s）
28. `yaw_angle_max_out`：Yaw角度环 `MaxOut`
29. `yaw_speed_pid_ref`：Yaw速度环参考（deg/s）
30. `yaw_speed_pid_fdb`：Yaw速度环反馈（deg/s）
31. `yaw_speed_pid_err`：Yaw速度环误差（deg/s）
32. `yaw_speed_pid_out`：Yaw速度环输出
33. `yaw_angle_sat_ratio`：Yaw角度环输出占上限比例
34. `imu_yaw_total`：IMU累计Yaw角（deg）
35. `imu_yaw`：IMU当前Yaw角（deg）
36. `imu_pitch`：IMU当前Pitch角（deg）
37. `imu_gyro_z`：IMU Z轴角速度（deg/s）
38. `imu_gyro_x`：IMU X轴角速度（deg/s）
39. `pitch_current_ff`：Pitch当前重力前馈输出

帧尾固定：`0x00 0x00 0x80 0x7F`。

## 手调建议（按顺序）

1. 摇杆不动：确认 `yaw_loop_mode=0`、`pitch_loop_mode=0`，且 `*_hold_ref` 稳定。
2. 轻推 Yaw：确认 `yaw_loop_mode` 立刻变 1，松杆后先进入 2，再回 0，`yaw_hold_ref` 在刹车结束后锁定。
3. 轻推 Pitch：确认 `pitch_loop_mode` 立刻变 1，松杆后先进入 2，再回 0，`pitch_hold_ref` 在刹车结束后锁定。
4. 调 Yaw：先看 `yaw_speed_pid_ref / yaw_speed_pid_fdb`，再看 `yaw_angle_err / yaw_angle_out`，最后结合 `imu_yaw_total / imu_yaw / imu_gyro_z` 判断分离模式下的参考系切换是否正确。
5. 调 Pitch：先看 `pitch_current_ff` 是否方向正确，再看 `pitch_speed_ref` 与 `pitch_speed_fdb` 是否明显滞后、超调，最后看 `pitch_angle_err / pitch_angle_out / pitch_angle_sat_ratio`。

## 注意

- 通道顺序不要改；后续扩展仅允许末尾追加。
- 若串口同时看文本日志，建议降低文本频率，避免观察干扰。
- 重力前馈接入后，优先观察保持状态下 `pitch_speed_pid_out` 是否明显减小，而不是只看角度误差瞬时值。
- `pitch_angle_out` 如果长期贴着 `pitch_angle_max_out`，说明角度环已经在顶输出，继续加 `PITCH_ANGLE_KP` 意义不大，应先检查 `MaxOut` 或速度环能力。
- `pitch_speed_pid_ref` 明显变化而 `pitch_speed_pid_fdb` 严重滞后，优先调 `PITCH_SPEED_KP/KI`，不是先加角度环。
