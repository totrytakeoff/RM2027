# VOFA+ JustFloat 通道说明（pitch_comp_scan_demo）

当前固件通过 `USART6` 输出 `JustFloat` 帧，通道顺序固定如下：

1. `time_ms`：系统时间戳，单位 `ms`
2. `step_idx`：当前步进序号，从 `0` 开始
3. `pass_idx`：扫描趟次，`0=首趟`，`1=回扫`
4. `dir_sign`：当前扫描方向，`1=正向`，`-1=反向`
5. `frame_kind`：帧类型，`0=实时采样`，`1=当前步稳态统计`
6. `stage_elapsed_ms`：当前步已持续时间
7. `current_cmd`：当前下发给 Pitch 6020 的开环电流/力矩命令
8. `motor_total_angle`：电机总角度反馈，单位 `deg`
9. `motor_speed_aps`：电机速度反馈，单位 `deg/s`
10. `motor_real_current`：电机回读实际电流
11. `imu_pitch`：IMU Pitch 角，单位 `deg`
12. `imu_gyro_x`：IMU X 轴角速度，单位 `deg/s`
13. `steady_motor_angle_avg`：当前步稳态窗口内电机角度均值
14. `steady_motor_current_avg`：当前步稳态窗口内实际电流均值
15. `steady_imu_pitch_avg`：当前步稳态窗口内 IMU Pitch 均值
16. `steady_sample_count`：当前步已纳入稳态统计的样本数

帧尾固定：`0x00 0x00 0x80 0x7F`。

## 使用建议

1. VOFA+ 协议选择 `JustFloat`。
2. 观察实时变化时看 `frame_kind=0` 的帧。
3. 做拟合时优先筛 `frame_kind=1` 的帧，这些是每一步结束时输出的稳态统计结果。
4. 如果要比较迟滞，开启双向扫描后同时看 `pass_idx` 和 `dir_sign`。

## 扫描参数

- `g_scan_current_start`：扫描起始电流
- `g_scan_current_step`：每步增量
- `g_scan_step_count`：步数
- `g_scan_hold_ms`：每步保持时长
- `g_scan_sample_period_ms`：实时采样周期
- `g_scan_settle_ms`：稳态统计开始时间，早于该时刻的样本不纳入均值
- `g_scan_direction`：初始方向，`1` 或 `-1`
- `g_scan_bidirectional`：`0=单向`，`1=正反各一趟`
- `g_scan_repeat`：`0=扫完停止`，`1=循环`
