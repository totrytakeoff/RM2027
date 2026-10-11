# 机器人底盘与云台调参说明

更新日期：2026-10-02

当前全部核心数值、单位、派生公式和发射参数统一收录在
[机器人核心控制参数索引](control_parameter_reference.md)。本文重点解释底盘与云台
控制逻辑和调参判据。

## 1. 分层约定

遥控设备层和机器人输入 Profile 只发布 `[-1,1]` 无量纲意图：

- `chassis_x_intent`、`chassis_y_intent`、`chassis_rotate_intent`；
- `gimbal_yaw_intent`、`gimbal_pitch_intent`。

遥控配置中只允许存在中位、量程、死区、串口和在线截止等设备参数。m/s、rad/s、
机械角度、电机减速比和输出饱和只能存在于底盘或云台执行层。

## 2. 底盘关键参数

参数位于 `applications/robot/config/robot_config.h`。

| 参数 | 当前值/含义 | 调整建议 |
| --- | --- | --- |
| `M3508_ROTOR_SPEED_LIMIT_RAD_S` | `523.599 rad/s`（5000 rpm），M3508转子速度参考上限 | 满量程总上限；确认电机温升后再改 |
| `CHASSIS_MOTOR_REDUCTION_RATIO` | `19.20320856` | 标准 M3508 减速比；非标准传动必须实测修改 |
| `CHASSIS_WHEEL_RADIUS` | `0.075 m` | 使用有效滚动半径，不只量外径 |
| `CHASSIS_MAX_TRANSLATION_SPEED` | 由上述三项推导，约 `2.045 m/s` | 不单独手填，避免物理量互相矛盾 |
| `CHASSIS_MAX_ROTATION_SPEED_RAD_S` | 由平移能力和半轮距推导，约 `12.03 rad/s` | 纯旋转满意图上限 |
| `CHASSIS_SPEED_FILTER_COEF` | `0.15` | `0` 最直接；增大更平滑也更迟钝 |
| `CHASSIS_FOLLOW_WZ_KP/KI/KD` | 当前 `30/0/0.80`；误差和输出使用 rad、rad/s | 当前实测跟随已基本稳定，后续小步调整 |
| `CHASSIS_FOLLOW_WZ_MAX` | 当前 `10 rad/s`，仅限制跟随外环 | 只在跟随大角度追赶时生效 |
| `CHASSIS_SPIN_SPEED_RATIO` | 当前 `0.60`，对应约 `7.22 rad/s` | 同时决定旋转速度和平移轮速余量，建议在 `0.55~0.70` 内试车 |
| `CHASSIS_SPEED_KP/KI/KD` | 四个 M3508 速度环 | 先验证参考和反馈单位/方向，再调 PID |

普通跟随模式下，平移和旋转叠加超限时仍共同等比例缩放。小陀螺模式下优先保留
旋转轮速，再按照四轮剩余余量统一缩放平移分量；`spin_translation_scale` 用于观察
实际保留的平移比例。底盘 M3508 的速度参考、速度反馈、PID 误差和调试量统一使用
转子 `rad/s`，不再换算成 `deg/s`。

云台坐标系到底盘坐标系的执行层变换使用
`yaw_offset = yaw_gimbal - yaw_chassis`。在左正、前正的坐标约定下：

```text
body_vx = gimbal_vx*cos(offset) + gimbal_vy*sin(offset)
body_vy = -gimbal_vx*sin(offset) + gimbal_vy*cos(offset)
```

## 3. 云台关键参数

| 参数 | 当前值/含义 | 调整建议 |
| --- | --- | --- |
| `GIMBAL_YAW_MAX_SPEED_DEG_S` | `828 deg/s` | Yaw 满摇杆速度，只在执行层生效 |
| `GIMBAL_PITCH_MAX_SPEED_DEG_S` | `540 deg/s` | Pitch 满摇杆对应的角度目标推进速率，只在执行层生效 |
| `GIMBAL_PITCH_MIN/MAX_DEG` | `-30/20 deg`（抬头/低头） | 机械限位外再留足软限位余量 |
| `YAW_ANGLE_*`、`YAW_SPEED_*` | Yaw 速度/保持 PID | 手动与制动只运行速度环，静止保持才运行角度→速度串级 |
| `YAW_BASE_RATE_CURRENT_FF_K/MAX` | 小陀螺基座角速度电流前馈，当前 `900/10000` | 先看通道 25 的基座估计，再按通道 22 的前馈逐步调节 |
| `YAW_SPEED_I_MAX` | Yaw 速度积分输出上限，当前 `4000` | 只负责前馈残差，不应代替基座补偿 |
| `PITCH_ANGLE_*`、`PITCH_SPEED_*` | Pitch 保持/手动串级 PID | 不要用积分掩盖重力前馈错误 |
| `PITCH_MOTOR_OUTPUT_REVERSED` | 当前 `1` | 本车执行输出反向；遥控层不得据此反向 |
| `PITCH_TARGET_INTEGRATION_MAX_DT_MS` | 当前 `40 ms` | 控制任务异常延迟时限制单次目标推进时间，避免目标跳变 |

Yaw 不再把满意图积分成持续超前的位置目标，正式执行路径为：

```text
摇杆活动 -> IMU速度环
松杆       -> IMU速度环参考清零，主动制动
速度稳定   -> 锁定当前IMU累计Yaw，进入角度保持
```

因此手动转动的响应主要看 `YAW_SPEED_*`；`YAW_ANGLE_KP/KD` 只影响松杆后的世界系
角度保持和外界扰动恢复。`YAW_BRAKE_SPEED_EPS/STABLE_COUNT/TIMEOUT_MS` 决定速度环
切回角度保持的门槛，不应通过增大角度 P 来补偿手动速度响应。

小陀螺模式额外估算：

```text
base_rate = imu_yaw_rate - yaw_motor_relative_rate
current_ff = clamp(-Kff * base_rate, ±FF_MAX)
```

该前馈只在 SB 上位启用，进出小陀螺会重置 Yaw 速度积分；普通跟随模式不会继承
小陀螺补偿。`YAW_SPEED_KI=10` 和 `YAW_SPEED_I_MAX=4000` 用于消除模型误差。

## 4. Pitch 重力补偿

Pitch 正式控制坐标统一为 IMU 物理坐标：角度保持目标、角度反馈、角速度参考、角速度
反馈和机械限位都以 IMU Pitch 正方向为正。2026-07-21 实机数据确认本车最终执行输出
必须反向一次，`PITCH_MOTOR_OUTPUT_REVERSED=1`；遥控适配层仍只发布规范化 Pitch 意图。

2026-07-20 修复前的稳定数据满足
`motor_encoder_angle + imu_pitch = 64.006° ± 0.077°`，说明两者方向相反。旧实现把电机
编码器角作为外环反馈、却把未反向的 IMU GyroX 作为内环反馈，已经废止。修复后
VOFA+ 的 `axis_hold_target_deg` 和 `axis_imu_angle_deg` 应处于同一坐标，通道 5 的角度
误差也由这套 IMU 反馈计算，不再关于约 32° 镜像。

当前执行层使用重力参考的 IMU Pitch：

```text
ff = clamp(K × cos(imu_pitch - horizontal_angle), ±FF_MAX)
```

Pitch 不再切换独立速度环。摇杆只提供无量纲速度意图，执行层按时间推进有限角度目标：

```text
hold_target += pitch_intent × GIMBAL_PITCH_MAX_SPEED_DEG_S × dt
hold_target = clamp(hold_target, -30°, 20°)
```

电机始终运行 `ANGLE -> SPEED` 串级；摇杆活动和松开都不重置 PID，松杆只停止目标推进。
因此轻推时位置误差会正常建立，速度积分也能连续承担负载。

关键参数：

- `PITCH_GRAVITY_FF_K=0`：当前去掉重力补偿后的临时调试值；
- `PITCH_GRAVITY_FF_MAX=4000`：前馈绝对上限；
- `PITCH_GRAVITY_HORIZONTAL_DEG=0`：炮管机械水平时的 IMU Pitch 实测值；
- `PITCH_FF_LPF=0.8`：越大越平滑、滞后越明显。

建议顺序：

1. 架空并允许快速断电，先令 `K=0`，确认正速度参考会产生正 IMU Pitch 角速度，且
   `hold_target` 与 `imu_angle` 同向、同值；
2. 将炮管机械调平，记录日志中的 `pitch_imu`，填入 `PITCH_GRAVITY_HORIZONTAL_DEG`；
3. 从 `K=-1000` 开始按 `-100~-200` 的步长调节，以稳定段速度积分残差判断幅值；
4. 稳态时 `axis_speed_i_current` 应接近 0，不应长期卡在 ±`PITCH_SPEED_I_MAX`；若
   云台自行抬头或向下运动明显吃力，立即降低前馈绝对值；
5. 最后再调 `PITCH_ANGLE_*` 和 `PITCH_SPEED_*`，不要先堆积分。

Pitch 与 Yaw 的策略不同：Pitch 全程角度串级，Yaw 仍使用“速度环主动制动后再锁角”的
三态策略。上电或安全门重新放行时，Pitch 执行层会将目标设为
`PITCH_STARTUP_CENTER_DEG=0`，
让云台回到 IMU 0°；遥控输入只推进该角度目标，不切换控制环。

视觉联调配置默认开启低频通用文本日志并关闭 VOFA 遥测；连续调参时切换为关闭文本日志、
开启独立 UART6/VOFA+ 任务。通道定义见
[机器人正式固件 UART 调参遥测](../debug/tuning_telemetry.md)。Yaw/Pitch 已使用相同的
`GimbalAxisTuningSnapshot`；需要 Pitch 专项变量时，在遥测入口选择 `gimbal.pitch` 即可，
无需再访问电机内部对象。

## 5. 底盘跟随控制律

当前控制律已经统一为直观正增益形式：

```text
wz = Kp × error + Ki × integral(error) + Kd × error_rate
```

当正误差正在收敛时，`error_rate<0`，所以正 `Kd` 会产生负的 D 项并降低追赶速度。
旧代码的外层整体负号已移除，禁止再使用负 `Kp/Kd` 维持双重符号。

跟随误差会在底盘执行层再次归一化到 `[-pi, pi]`，所以恢复到
`YAW_CHASSIS_ALIGN_ECD` 标定正姿态时总是选择最近方向。以下两个边沿会立即进入恢复：

- 上电、遥控重连或 SA 安全门重新放行后的首个 FOLLOW 控制周期；
- SB 从小陀螺上位切换到中位或下位后的首个控制周期。

恢复首拍的 `wz` 不经过 `CHASSIS_SPEED_FILTER_COEF` 削弱，后续周期仍按正常低通和
跟随 PID 工作。这里的“上电立即恢复”服从 SA 安全门：SA 未上拨、遥控离线或系统停机时
电机保持掉电，只记录待恢复状态；允许输出后才开始运动。云台继续保持当前世界系 Yaw
目标，恢复动作由底盘旋转完成，不会让云台自行回头破坏瞄准方向。
