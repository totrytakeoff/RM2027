# infantry_minimal VOFA+ JustFloat 默认通道（核心16）

固定顺序如下，后续扩展只能在末尾追加，不允许改动前16项顺序：

1. `time_ms`
2. `input_active`
3. `input_online`
4. `chassis_vx_cmd`
5. `chassis_vy_cmd`
6. `chassis_wz_cmd`
7. `chassis_fr_speed_ref`
8. `chassis_fr_speed_fdb`
9. `yaw_speed_ref`
10. `yaw_speed_fdb`
11. `pitch_speed_ref`
12. `pitch_speed_fdb`
13. `pitch_target_angle`
14. `loader_ref`
15. `loader_fdb`
16. `shoot_state`

协议：16个`float`小端连续输出，帧尾固定`00 00 80 7F`。
