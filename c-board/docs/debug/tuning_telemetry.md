# 机器人正式固件调参遥测

更新日期：2026-10-02

当前视觉联调默认配置为 `MINIMAL_DEBUG_ENABLE=1`、
`ROBOT_TUNING_TELEMETRY_ENABLE=0`，让 UART6 输出低频文本状态。要使用本页的
VOFA 遥测，需关闭文本日志并打开 `ROBOT_TUNING_TELEMETRY_ENABLE`，避免同一串口混流。

各速度、机械限位、PID 和发射射频的集中说明见
[机器人核心控制参数索引](../motor/control_parameter_reference.md)。

## 1. 通道 Getter 与开关

正式固件将不同模块的字段封装为独立 Getter。组帧入口直接调用当前需要的 Getter，返回值
就是本帧实际通道数，不使用调参类型或目标电机选择宏：

- `MINIMAL_DEBUG_ENABLE=0`：关闭原来的周期文本/RTT镜像；
- `ROBOT_TUNING_TELEMETRY_ENABLE=1`：启用独立 `tuning_uart` 任务；
- UART6，115200，8N1，20 ms 周期；
- VOFA+ `JustFloat`，DMA 非阻塞发送，UART 忙时丢弃本帧。

当前组帧入口只有一条有效选择语句：

```c
return TuningTelemetry_GetShootStateChannels(channels);
```

即当前 UART6 只发送发射输入与状态机数据，用于验证单发触发链路，不包含电机 PID、
IMU、Yaw、Pitch 或底盘通道。需要调其他模块时，只替换为对应 Getter 调用。

UART 配置位于 `applications/robot/config/robot_config.h`。VT 后端占用 UART6，
编译期互斥检查会禁止 VT 与当前调参任务同时启用。

## 2. 发射与电机 Getter

DJI 电机通道只读取设备层的 `DJIMotorTuningSnapshot`，不包含任何发射状态。发射状态机
由 `GetShootStateChannels()` 单独输出。因此后续底盘 M3508 也可以复用速度电机字段，
不会依赖 Shoot 模块。

### 2.1 摩擦轮的 11 个通道

字段顺序与[摩擦轮字段 CSV](friction_tuning_fields.csv)一致：

| 通道 | 名称 | 单位/说明 |
| ---: | --- | --- |
| 0 | `time_ms` | 系统时间，ms |
| 1 | `motor_output_active` | 当前是否实际输出 |
| 2 | `speed_target_rad_s` | 电机任务实际速度目标，rad/s |
| 3 | `speed_fdb_rad_s` | 电机转子速度反馈，rad/s |
| 4 | `speed_error_rad_s` | 速度误差，rad/s |
| 5~7 | `speed_p/i/d_current` | 速度 PID 各项，电流指令单位 |
| 8 | `speed_output_current` | 速度 PID 总输出 |
| 9 | `final_output_current` | 方向处理和型号限幅后的实际 CAN 指令 |
| 10 | `current_fdb` | 电调原始反馈电流 |

每帧 11 个 `float` 加帧尾，共 48 字节。左右摩擦轮分别调用
`GetFrictionLeftChannels()` 和 `GetFrictionRightChannels()`，字段完全相同。

### 2.2 Loader 的 19 个通道

字段顺序与[Loader 字段 CSV](loader_tuning_fields.csv)一致。通道 0~2 是时间、
输出状态和外环类型；3~9 是电机侧角度目标/反馈/误差及角度 PID；10~16 是 `deg/s`
速度目标/反馈/误差及速度 PID；17~18 是最终 CAN 电流指令和电流反馈。角度模式下通道 4
是位置 PID 实际采用的多圈角反馈；速度模式下未执行角度环，3、5~9 输出 0，通道 4 保留
原始累计编码器角。每帧共 80 字节。

### 2.3 发射状态的 15 个通道

字段顺序与[发射状态字段 CSV](shoot_state_tuning_fields.csv)一致：

| 通道 | 名称 | 说明 |
| ---: | --- | --- |
| 0 | `time_ms` | 系统时间，ms |
| 1 | `input_fire_mode` | 执行层实际收到的模式：`0=DISABLED, 1=SINGLE, 2=CONTINUOUS` |
| 2 | `fire_trigger_down` | 执行层实际收到的 SD 稳定电平 |
| 3 | `fire_trigger_pressed` | 遥控适配器产生的一帧上拨边沿，仅作诊断 |
| 4 | `single_trigger_consumed` | 当前 SD 上位是否已经消费，回到下位或离开单发模式时清零 |
| 5 | `single_trigger_activation_count` | 执行层累计识别到的单发扳机次数 |
| 6 | `shoot_state` | `0=OFF, 1=FRICTION_ON, 2=SINGLE, 3=CONTINUOUS` |
| 7 | `friction_ready` | 双轮误差进入稳定窗口后置 1 |
| 8 | `single_active` | 单发位置动作正在执行 |
| 9 | `pending_shots` | 待执行单发请求数 |
| 10 | `single_start_count` | Loader 累计开始执行的单发次数 |
| 11 | `single_timeout_count` | 单发累计超时次数 |
| 12 | `loader_jam_state` | `0=IDLE, 1=REVERSING, 2=LOCKED` |
| 13 | `loader_jam_retry_count` | 本次连续触发内的退弹次数 |
| 14 | `loader_jam_fault_count` | 累计卡弹锁止次数 |

每帧 15 个 `float` 加帧尾，共 64 字节。单发不再依赖通道 3 只持续一帧的边沿；执行层
根据通道 2 的稳定电平维护通道 4 的消费锁存。SD 保持上位只产生一次请求，回到下位后
才能再次触发；在 SD 已经上位时切入单发模式，也会产生一次请求。

### 2.4 当前发射基线与上机顺序

摩擦轮 M3508 为直驱，不使用任何减速比。当前目标 `-523.5988 rad/s` 等于 `-5000 rpm`；
右轮通过电机反向配置得到相反的实际转向。速度环基线为 `Kp=47.74648`、`Ki=0`、
`Kd=0`、`MaxOut=8000`，其中 Kp 等价于参考工程的 `5 command/RPM`。双轮目标误差均
进入 8% 窗口并持续 150 ms 后，`friction_ready` 才置 1；误差超过 25% 持续 60 ms
后撤销。

Loader 是 M2006 P36。八槽拨盘每发输出轴转过 45°，对应电机侧 `45*36=1620°`；
5 发/s 对应 `8100 deg/s=1350 rpm`。速度环基线为 `Kp=1.666667`、`Ki=0`、`Kd=0`、
`MaxOut=4000`，Kp 等价于参考工程的 `10 command/RPM`。当前先用纯 P 确认不再正反
饱和，出现可重复的带载静差后再逐步增加 Ki，不能直接复制参考工程不含 dt 的积分参数。

连发时，正向命令达到 60%、反馈低于 `360 deg/s` 且最终输出超过 80% 限幅，持续
120 ms 才判定卡弹。系统反转 `8100 deg/s` 运行 120 ms 后从 0 重新升速；同一次扣住
扳机最多恢复两次，第三次锁止 Loader。松开 SD 后解除本轮锁止，累计故障计数不清零。

当前入口先验证单发状态链路：SC 中位后等待 `friction_ready=1`，将 SD 从下位拨到上位，
确认 `fire_trigger_down` 和 `single_trigger_consumed` 变为 1、
`single_trigger_activation_count` 增加一次，随后 `shoot_state=2` 且
`single_start_count` 增加一次。SD 保持上位时计数不得继续增加；回到下位后再次上拨，计数
才应再次增加。状态链路确认后，将入口切到 `GetLoaderChannels()`，验证单发进入
`outer_loop=4`、电机侧目标累计推进 1620°，再调位置串级 PID。

## 3. 云台 Getter 的 31 个通道

通道从 0 开始编号，顺序与
[字段 CSV](gimbal_tuning_fields.csv)完全一致。字段中的 `axis` 表示当前调用
`GetYawChannels()` 或 `GetPitchChannels()` 选择的主调试轴：

| 通道 | 名称 | 单位/说明 |
| ---: | --- | --- |
| 0 | `time_ms` | 系统时间，ms |
| 1 | `axis_ctrl_mode` | `0=ANGLE, 1=SPEED, 2=BRAKE` |
| 2 | `axis_operator_speed_cmd_deg_s` | 操作者速度指令，deg/s |
| 3 | `axis_hold_target_deg` | 角度保持目标，deg |
| 4 | `axis_imu_angle_deg` | 所选轴 IMU 角反馈，deg |
| 5 | `axis_angle_error_deg` | 电机任务实际角度 PID 误差，deg |
| 6 | `axis_imu_gyro_deg_s` | 角速度环实际使用的 IMU 反馈，deg/s |
| 7 | `axis_angle_p_deg_s` | 角度 PID 的 P 项，deg/s |
| 8 | `axis_angle_i_deg_s` | 角度 PID 的 I 项，deg/s |
| 9 | `axis_angle_d_deg_s` | 角度 PID 的 D 项，deg/s |
| 10 | `axis_angle_output_deg_s` | 角度 PID 最终输出，即速度环参考，deg/s |
| 11 | `axis_angle_output_limit_ratio` | 角度环输出限幅比例，到 ±1 即限幅 |
| 12 | `axis_speed_ref_deg_s` | 电机任务实际速度 PID 参考，deg/s |
| 13 | `axis_speed_fdb_deg_s` | 电机任务实际速度 PID 反馈，deg/s |
| 14 | `axis_speed_error_deg_s` | 速度 PID 误差，deg/s |
| 15 | `axis_speed_p_current` | 速度 PID 的 P 项，GM6020 电流命令单位 |
| 16 | `axis_speed_i_current` | 速度 PID 的 I 项，GM6020 电流命令单位 |
| 17 | `axis_speed_d_current` | 速度 PID 的 D 项，GM6020 电流命令单位 |
| 18 | `axis_speed_output_current` | 速度 PID 输出；尚未叠加前馈和最终电机方向变换 |
| 19 | `axis_speed_output_limit_ratio` | 速度环输出限幅比例，到 ±1 即限幅 |
| 20 | `axis_motor_current_fdb` | GM6020 电调原始反馈电流；Pitch 电机反装时符号不同于 IMU 控制轴 |
| 21 | `axis_encoder_ecd` | GM6020 原始编码器值，0~8191 |
| 22 | `axis_current_feedforward` | 控制轴坐标中的电流前馈；Yaw 小陀螺补偿或 Pitch 重力补偿 |
| 23 | `yaw_offset_logic_deg` | Yaw 相对底盘标定零位误差，deg |
| 24 | `chassis_wz_cmd_rad_s` | 底盘最终旋转命令，rad/s |
| 25 | `yaw_base_rate_estimate_rad_s` | `IMU角速度-Yaw相对角速度` 得到的底盘基座角速度估计 |
| 26 | `chassis_input_x_intent` | 云台坐标系横移输入，左正，归一化值 |
| 27 | `chassis_input_y_intent` | 云台坐标系前进输入，前正，归一化值 |
| 28 | `chassis_body_vx_cmd_m_s` | 坐标变换后的车体横移命令，左正，m/s |
| 29 | `chassis_body_vy_cmd_m_s` | 坐标变换后的车体前进命令，前正，m/s |
| 30 | `chassis_spin_translation_scale` | 小陀螺为旋转保留轮速后实际平移比例，0~1 |

每帧为 31 个小端 `float`，帧尾固定为 `00 00 80 7F`。一帧共 128 字节，
50 Hz 时串口线速约 64 kbit/s，低于 115200 bit/s。

正式云台接口使用统一的 `GimbalTuningSnapshot`，其中 `yaw`、`pitch` 都是相同的
`GimbalAxisTuningSnapshot`。调参入口直接调用 Yaw 或 Pitch Getter，编码器、IMU、角度
PID、速度 PID、电流反馈和电流前馈不需要再增加底层接口。

## 4. Yaw 标定顺序

1. SA 下位，架空底盘，将云台机械中心线与底盘正前方精确对齐；
2. 保持主调试轴为 Yaw，记录 `axis_encoder_ecd`；
3. 将 `YAW_CHASSIS_ALIGN_ECD` 改为记录到的整数编码器值；
4. 首轮将 `YAW_OFFSET_LOGIC_ZERO_DEG` 设为 `0.0f`，重新编译烧录；
5. 对正时确认 `axis_encoder_ecd` 接近标定值、`yaw_offset_logic_deg` 接近 0；
6. 若只有小于一个编码器刻度的安装残差，再使用 `YAW_OFFSET_LOGIC_ZERO_DEG` 微调。

`/home/myself/Downloads/vofa+.csv` 中原先选择的 `ECD=25` 姿态经实机复核存在机械
偏差，不再作为正式零位。2026-07-20 将云台重新摆到确认后的机械正前方，并从 UART6
连续采集 6 秒、共 300 帧：`ECD=7821` 占 283 帧，`ECD=7822` 占 17 帧，平均
`7821.0567`；编码器单圈角平均 `343.69634°`。正式配置改为：

```c
#define YAW_CHASSIS_ALIGN_ECD     7821U
#define YAW_OFFSET_LOGIC_ZERO_DEG 0.0f
```

按新配置重新烧录后，当前机械对正位置的 raw/logic offset 理论均值约
`0.00249°`；允许因编码器一格跳变出现约 `0.043945°` 的量化波动。

完成零位标定后，先使用 1~22 通道调稳 Yaw，再看 23~30 通道分析底盘耦合和
小陀螺平移解算。
Yaw 三态控制的预期顺序是：

```text
摇杆活动: axis_ctrl_mode=1，axis_speed_ref 跟随操作者速度指令
摇杆释放: axis_ctrl_mode=2，axis_speed_ref=0 主动制动
速度稳定: axis_ctrl_mode=0，axis_hold_target 锁定当前 IMU 角度
```

Pitch 始终保持 `ANGLE(0)`，不再进入 SPEED/BRAKE，也不会因摇杆活动或松开而重置 PID。
`axis_operator_speed_cmd_deg_s` 是执行层按摇杆计算的目标推进速率；控制任务按实际经过时间
将其累计到 `axis_hold_target_deg`，松杆后目标立即冻结。

若通道 11 到达 ±1，是角度外环输出受限；若通道 19 到达 ±1，是速度环电流输出
受限。小陀螺下还应同时观察通道 22 的基座电流前馈、通道 25 的基座角速度估计和
通道 30 的平移保留比例。

## 5. Pitch 坐标与调参判据

Pitch 主调试轴下，通道 3~19 已统一为 IMU Pitch 坐标：

```text
正角度/正速度参考 -> 正 IMU Pitch/正 GyroX
ANGLE 模式          -> hold_target 与 imu_angle 直接比较
axis_speed_ref       -> 角度外环产生的内环速度参考
```

电机安装反向只在最终执行输出处理，所以通道 20 仍是电调原始电流；它可与控制轴电流
通道符号相反。本车当前 Pitch 的近似关系为
`motor_command = -(axis_speed_output_current + axis_current_feedforward)`。
2026-07-20 修复前 `hold_target` 使用电机编码器角，曾与 IMU Pitch 满足
约 `motor+imu=64°` 的镜像关系，该行为已废止，不得再以编码器角作为 Pitch 保持目标。

当前 `PITCH_GRAVITY_FF_K=0` 是去掉重力补偿后的临时调试值。稳定保持时重点确认通道 16
不长期卡在 ±`PITCH_SPEED_I_MAX`；否则应恢复并标定重力前馈，而不是持续提高角度 P。

## 6. 自定义入口

字段唯一组帧入口是 `TuningTelemetry_FillChannels()`。当前可直接调用：

- `TuningTelemetry_GetYawChannels(channels)`；
- `TuningTelemetry_GetPitchChannels(channels)`；
- `TuningTelemetry_GetLoaderChannels(channels)`；
- `TuningTelemetry_GetFrictionLeftChannels(channels)`；
- `TuningTelemetry_GetFrictionRightChannels(channels)`；
- `TuningTelemetry_GetShootStateChannels(channels)`。

Getter 使用递增下标填充并返回实际通道数，发送层据此产生变长 JustFloat 帧。新增模块时
只新增 Getter 并在入口调用，不复制其他 Getter，也不增加选择宏。修改字段顺序时同步更新
本文档和对应 CSV。
