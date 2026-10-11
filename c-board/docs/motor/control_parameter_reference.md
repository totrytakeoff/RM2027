# 机器人核心控制参数索引

更新日期：2026-07-23

本文对应正式 FreeRTOS 固件的
`applications/robot/config/robot_config.h`。它回答三个问题：哪个参数决定整车实际
速度，哪个参数只能用于标定或保护，以及调参时应该观察什么。控制原理和波形判据分别见
[底盘与云台调参说明](chassis_gimbal_tuning.md)和
[UART6/VOFA+ 调参遥测](../debug/tuning_telemetry.md)。

## 1. 参数标记与修改原则

配置头中的参数按以下类别标记：

| 标记 | 含义 | 修改原则 |
| --- | --- | --- |
| `[核心]` | 直接决定速度、射频、机械范围或安全策略 | 上机前确认，修改后重新完成对应模块测试 |
| `[标定]` | 与本车尺寸、安装方向、编码器零位绑定 | 更换机构或重新装配后实测，不能照抄其他车辆 |
| `[PID]` | 闭环增益 | 一次只改一层环路，必须保留目标、反馈和输出波形 |
| `[保护]` | 限幅、超时、就绪和故障阈值 | 不能用放宽保护掩盖控制或机械问题 |
| `[派生]` | 由其他参数按公式计算 | 不要改成独立魔数，应修改公式的源参数 |
| `[固定]` | 协议、接线、任务拓扑或闭环类型 | 只有硬件/架构变化时才修改 |

所有速度均属于执行层。ET08/DT7/VT 输入层只发布 `[-1,1]` 无量纲意图，不允许再设置
底盘 m/s、云台 deg/s、电机转速或 Pitch 机械限位。

## 2. 当前核心值总览

| 模块 | 参数 | 当前实际含义 |
| --- | --- | --- |
| 控制时序 | `MAIN_LOOP_PERIOD_MS` | 应用控制 20 ms，50 Hz |
| 电机时序 | `ROBOT_MOTOR_TASK_PERIOD_MS` | DJI 电机闭环 5 ms，200 Hz |
| 底盘轮速 | `M3508_ROTOR_SPEED_LIMIT_RAD_S` | 转子 `523.599 rad/s = 5000 rpm` |
| 底盘平移 | `CHASSIS_MAX_TRANSLATION_SPEED` | 公式推导约 `2.045 m/s` |
| 底盘旋转 | `CHASSIS_MAX_ROTATION_SPEED_RAD_S` | 公式推导约 `12.029 rad/s` |
| 小陀螺 | `CHASSIS_SPIN_SPEED_RATIO` | `0.60`，实际约 `7.218 rad/s` |
| Yaw 手动速度 | `GIMBAL_YAW_MAX_SPEED_DEG_S` | 满摇杆 `828 deg/s` |
| Pitch 目标推进 | `GIMBAL_PITCH_MAX_SPEED_DEG_S` | 满摇杆 `540 deg/s` |
| Pitch 软限位 | `GIMBAL_PITCH_MIN/MAX_DEG` | 抬头 `-30 deg`，低头 `20 deg` |
| 摩擦轮 | `FRICTION_TARGET_SPEED_RAD_S` | M3508 直驱 `-523.599 rad/s = -5000 rpm` |
| 连发射频 | `LOADER_CONTINUOUS_SHOTS_PER_S` | `5 发/s` |
| 连发电机速度 | `LOADER_CONTINUOUS_SPEED_DEG_S` | M2006 转子 `8100 deg/s = 1350 rpm` |
| 单发步进 | `LOADER_ANGLE_STEP` | P36 电机侧 `1620 deg/发` |

## 3. 系统时序与安全参数

| 参数 | 作用 | 重点 |
| --- | --- | --- |
| `MAIN_LOOP_PERIOD_MS=20` | 输入映射、底盘、云台、发射状态机周期 | 改动会影响目标积分、稳定计数和斜坡步长 |
| `ROBOT_INS_TASK_PERIOD_MS=1` | IMU 更新周期 | Yaw/Pitch 使用该任务发布的姿态和角速度 |
| `ROBOT_MOTOR_TASK_PERIOD_MS=5` | 所有 DJI PID 计算周期 | PID 的 Ki/Kd 使用实际 `dt`，不能复制无 `dt` 工程参数 |
| `ROBOT_MOTOR_COMMAND_TIMEOUT_MS=100` | 应用命令租约 | 控制任务超过 100 ms 不续命时，电机任务独立清零 |
| `ROBOT_SAFETY_REQUIRE_EXPLICIT_REARM=0` | SA 门策略 | 当前调试阶段采用直接电平恢复，不要求完整重新解锁动作 |
| `ROBOT_SAFETY_GATE_ON_MOTOR_HEALTH=0` | 电机健康策略 | 单电机掉线报警并隔离，不阻止其他电机工作 |
| `REFEREE_ENABLE=0` | 裁判系统联锁 | 当前关闭；启用后功率和热量限制才生效 |

遥控离线、初始化失败、INS 无效和关键任务故障仍会关闭全局电机输出，上述两个调试开关
不会绕过这些故障。

## 4. 底盘参数

### 4.1 物理参数和最大速度

底盘使用 M3508 标准减速箱：

```text
motor_limit = 523.5988 rad/s = 5000 rpm
wheel_radius = 0.075 m
reduction_ratio = 19.20320856
wheel_base = 0.34 m

max_translation = motor_limit * wheel_radius / reduction_ratio
                = 2.04497 m/s

max_rotation = max_translation / (wheel_base / 2)
             = 12.02921 rad/s
```

核心源参数如下：

| 参数 | 类型 | 调大后的直接影响 |
| --- | --- | --- |
| `M3508_ROTOR_SPEED_LIMIT_RAD_S` | 核心 | 提高所有底盘轮的最高转子速度，同时增加功耗、温升和机械风险 |
| `CHASSIS_WHEEL_RADIUS` | 标定 | 相同电机转速换算出更高线速度；必须使用有效滚动半径 |
| `CHASSIS_MOTOR_REDUCTION_RATIO` | 标定 | 数值越大，推导线速度越低、轮端理论扭矩越高 |
| `CHASSIS_WHEEL_BASE` | 标定 | 数值越大，相同轮速对应的纯旋转角速度越低 |

`CHASSIS_MAX_TRANSLATION_SPEED` 和 `CHASSIS_MAX_ROTATION_SPEED_RAD_S` 是派生量，不能脱离
上述物理参数单独改值。平移与旋转叠加超出 5000 rpm 时，执行层按四轮最大值统一缩放。

### 4.2 小陀螺和跟随

`CHASSIS_SPIN_SPEED_RATIO=0.60` 将小陀螺设为理论旋转能力的 60%，即约
`7.218 rad/s`。比例越高，留给平移的轮速余量越少；当前建议只在 `0.55~0.70` 内小步
试车，观察遥测中的 `spin_translation_scale`。

跟随外环参数：

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `CHASSIS_FOLLOW_WZ_KP` | `30` | Yaw 相对角误差到追赶角速度的主增益 |
| `CHASSIS_FOLLOW_WZ_KD` | `0.80` | 根据相对角速度抑制超调和往复摆动 |
| `CHASSIS_FOLLOW_WZ_KI` | `0` | 当前关闭；除非存在可重复静差，否则不启用 |
| `CHASSIS_FOLLOW_WZ_MAX` | `10 rad/s` | 只限制跟随追赶，不限制小陀螺公式本身 |

调跟随前必须先确认 Yaw IMU 闭环稳定。云台自身超调会改变相对夹角，底盘会正确地跟着
这个错误反馈摆动，此时继续调底盘 Kp/Kd 只会掩盖根因。

### 4.3 四轮速度环

`CHASSIS_SPEED_KP/KI/KD=229.18312/2.291831/0`，误差单位为转子 `rad/s`，输出进入
C620 电流环。`CHASSIS_SPEED_MAX_OUT=15000` 是速度 PID 输出限制，不是 rpm。

调参顺序：先确认四轮目标和反馈符号一致，再逐步提高 Kp；出现快速正负抖动时回退 Kp；
最后才增加 Ki 消除带载静差。Kd 对速度反馈噪声敏感，当前保持 0。

## 5. 云台参数

### 5.1 手动速度与机械范围

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `GIMBAL_YAW_MAX_SPEED_DEG_S` | `828 deg/s` | Yaw 满摇杆速度，直接进入 IMU 速度环 |
| `GIMBAL_PITCH_MAX_SPEED_DEG_S` | `540 deg/s` | Pitch 满摇杆时角度目标的推进速率 |
| `GIMBAL_PITCH_MIN_DEG` | `-30 deg` | IMU 坐标中的抬头软限位 |
| `GIMBAL_PITCH_MAX_DEG` | `20 deg` | IMU 坐标中的低头软限位 |
| `PITCH_STARTUP_CENTER_DEG` | `0 deg` | 上电或安全门重新放行后的 Pitch 回中目标 |

Pitch 始终使用 `ANGLE -> SPEED` 串级；摇杆只推进有限角度目标。Yaw 则使用“手动速度、
松杆速度刹停、稳定后角度保持”三态，因此两个轴不能照搬同一套 PID 调法。

### 5.2 Yaw PID 与小陀螺前馈

| 参数 | 当前值 | 所属环路 |
| --- | ---: | --- |
| `YAW_ANGLE_KP/KI/KD` | `15/0/2` | IMU 角误差(deg) -> 速度目标(deg/s) |
| `YAW_ANGLE_MAX_OUT` | `5000 deg/s` | 角度外环输出限幅 |
| `YAW_SPEED_KP/KI/KD` | `18/10/0` | IMU GyroZ 误差 -> GM6020 电流命令 |
| `YAW_SPEED_MAX_OUT` | `15000` | 速度 PID 总输出限幅 |
| `YAW_SPEED_I_MAX` | `4000` | 速度积分项限幅 |
| `YAW_BASE_RATE_CURRENT_FF_K` | `900` | 小陀螺基座角速度电流前馈增益 |
| `YAW_BASE_RATE_CURRENT_FF_MAX` | `10000` | 小陀螺前馈绝对限幅 |

手动响应迟钝首先看 `GIMBAL_YAW_MAX_SPEED_DEG_S` 和 `YAW_SPEED_*`；松杆后的角度超调才看
`YAW_ANGLE_*`。小陀螺顺逆向手感不一致时先检查基座角速度估计和前馈符号，不能用增大
速度积分强行抵消错误前馈。

### 5.3 Pitch PID 与重力补偿

| 参数 | 当前值 | 所属环路 |
| --- | ---: | --- |
| `PITCH_ANGLE_KP/KI/KD` | `100/0/0.5` | IMU Pitch 角误差 -> 速度目标 |
| `PITCH_SPEED_KP/KI/KD` | `10/15/0` | IMU GyroX 误差 -> GM6020 电流命令 |
| `PITCH_SPEED_I_MAX` | `5000` | 承担未被重力前馈覆盖的稳态负载 |
| `PITCH_GRAVITY_FF_K` | `0` | 当前轻负载调试阶段关闭，后续负载增加后重新标定 |
| `PITCH_GRAVITY_HORIZONTAL_DEG` | `0 deg` | 炮管机械水平时的 IMU Pitch 实测值 |
| `PITCH_GRAVITY_FF_MAX` | `4000` | 重力补偿绝对电流上限 |

Pitch 抬头后缓慢下掉时，先观察速度积分是否长期接近 `PITCH_SPEED_I_MAX`。负载明显增加后
应重新标定重力前馈，而不是持续提高位置 P 或积分上限。

## 6. 发射参数

### 6.1 摩擦轮

摩擦轮 M3508 没有减速箱，配置直接使用转子速度：

```text
FRICTION_TARGET_SPEED_RAD_S = -523.5988 rad/s = -5000 rpm
```

负号只对应本车安装方向，弹速由绝对值决定。`FRICTION_SPEED_SLEW_RAD_S_PER_MS=0.75`
使目标约用 `698 ms` 从 0 升到 5000 rpm。

当前速度环为 `Kp=47.74648, Ki=0, Kd=0, MaxOut=8000`。实机数据平均约 4969 rpm，
波动较小且无输出饱和；现阶段不需要为不到 1% 的静差增加积分。

拨弹就绪条件为：命令达到最终值 98%，双轮速度误差均不超过 8%，持续 150 ms。已经
就绪后，误差超过 25% 持续 60 ms 才撤销，避免弹丸冲击造成单帧误停。

### 6.2 M2006 P36 拨弹

当前机构是 P36、八槽拨盘：

```text
每发输出轴角度 = 360 / 8 = 45 deg
每发电机侧角度 = 45 * 36 = 1620 deg
5 发/s 电机速度 = 1620 * 5 = 8100 deg/s = 1350 rpm
```

所以：

- 改连发快慢，优先改 `LOADER_CONTINUOUS_SHOTS_PER_S`；
- 改拨盘槽数或减速箱，改 `LOADER_OUTPUT_STEP_DEG` 或 `LOADER_GEAR_RATIO`；
- 方向错误只改 `LOADER_DIRECTION_SIGN`，不要同时反向 PID 或遥控输入；
- `LOADER_CONTINUOUS_SPEED_DEG_S` 和 `LOADER_ANGLE_STEP` 都是派生量。

连发速度环当前为 `Kp=1.666667, Ki=0, Kd=0, MaxOut=4000`。实机稳定反馈约
`7927 deg/s`，约 `4.89 发/s`；高频正负修正偏多但没有饱和。若继续优化，先将 Kp 小步
降到 `1.4~1.5` 观察机械抖动，再决定是否以很小 Ki 补偿静差，不能同时大改两项。

单发使用 `ANGLE -> SPEED` 串级。位置目标以 `5000 deg/s` 推进，理论 `324 ms` 完成一发；
最终角误差不超过 `15 deg`、速度不超过 `180 deg/s` 并持续 60 ms 后完成，再保持位置
150 ms。总动作超过 1000 ms 会超时停机。

### 6.3 单发触发与卡弹保护

单发的一次性语义属于发射执行层：SC 中位且 SD 上位时消费一次稳定电平；SD 保持上位
不会重复，回到下位后才重新允许下一发。遥控适配器的单帧 `pressed` 只用于诊断。

连发卡弹判定同时要求：目标超过正常值 60%、实际速度低于 `360 deg/s=60 rpm`、输出超过
`LOADER_SPEED_MAX_OUT` 的 80%，并持续 120 ms。确认后反转 120 ms，同一次持续扳机最多
恢复两次，第三次锁止。不能仅因正常速度抖动就放宽这些阈值。

## 7. 推荐上机调参顺序

1. 确认 CAN ID、电机在线状态、安装方向和 SA 安全门。
2. 确认遥控只输出无量纲意图，没有任何执行层速度或机械限位。
3. 单独调稳四轮 M3508 速度环，再测试底盘平移、旋转和功率缩放。
4. 单独调稳 Yaw IMU 速度环和角度保持，再调底盘跟随与小陀螺前馈。
5. Pitch 先确认方向、软限位和 IMU 目标一致，再调串级 PID，最后标定重力补偿。
6. 分别验证左右摩擦轮，再验证 Loader 空载连发。
7. 用发射状态通道确认单发请求进入状态机，再切 Loader 通道调单发位置串级。
8. 最后带弹验证射频、摩擦轮掉速、卡弹反转、温升和长时间运行。

当前 UART6 组帧入口及各 Getter 的字段定义见
[机器人正式固件调参遥测](../debug/tuning_telemetry.md)。调某个模块时只切换 Getter，
不要把发射状态、IMU 和通用电机 PID 混入同一个可复用电机通道函数。
