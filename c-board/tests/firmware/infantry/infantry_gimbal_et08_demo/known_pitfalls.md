# infantry_gimbal_et08_demo 已确认坑点记录

文档更新日期：2026-07-23

本文记录当前最小/独立 demo 调试中已经明确定位过的两个高频坑：

1. `DaemonTask()` 调度频率错误，导致 `ET08_IsOnline()` 抖动，进而触发 `hold_ref` 被反复重锁。
2. `DJIMotorControl()` / `CANTransmit()` 调用频率过高，导致 CAN mailbox 满、反馈掉帧、部分电机异常。

目标不是复盘过程，而是给后续迁移和现场排障提供可直接执行的判断标准。

---

## 1. 守护任务频率坑：`hold_ref` 自己变化

### 现象

- 摇杆松手后，理论上应该进入 `ANGLE_LOOP` 保持当前位置。
- 实际上 `yaw_hold_ref` / `pitch_hold_ref` 会自己变化。
- 外力拨动云台后，目标像是“跟着当前角度跑了”。
- 串口日志里可能反复出现：
  - `remote offline/failsafe -> stop`
  - `remote online -> run`
  - `motors re-enabled hold(...)`

### 根因

`DaemonTask()` 当前实现不是按“毫秒”减计数，而是按“被调用次数”减计数。

相关实现见：

- `components/services/device_health/daemon.c`
- `components/devices/remote/et08/et08_remote.c`

关键点：

- `DaemonReload()` 只是把 `temp_count = reload_count`
- `DaemonTask()` 每被调用一次，就对 `temp_count--`
- `ET08_Init()` 中 `reload_count = 400`

这意味着 `400` 的语义其实不是 `400ms`，而是“允许 400 次 `DaemonTask()` 调用没收到新帧”。

如果主循环裸跑，没有节流，那么：

- `DaemonTask()` 调用频率远高于 SBUS 帧率
- `ET08_IsOnline()` 会在两帧之间被错误打成离线
- 上层控制逻辑看到离线后执行 `GimbalStop()`
- 下一次重新上线时，如果重新使能路径里又把 `hold_ref` 设为当前角度，那么目标就被重锁

### 本次定位证据

#### 1) `DaemonTask()` 是“按调用次数减计数”

`components/services/device_health/daemon.c`

- `temp_count > 0` 时每次调用直接 `temp_count--`

#### 2) ET08 在线判断依赖 daemon 计数

`components/devices/remote/et08/et08_remote.c`

- `reload_count = 400`
- `ET08_IsOnline()` 直接返回 `DaemonIsOnline(et08_daemon)`

#### 3) 原 `gimbal_et08_demo` 主循环裸跑

原逻辑中：

- `DaemonTask()`
- `DJIMotorControl()`
- 无固定 `HAL_Delay()` / 无固定任务周期约束

这会把 daemon 计数语义彻底打乱。

#### 4) `omni_demo` 和 RTOS 基线并不是裸跑

参考：

- `tests/firmware/infantry/infantry_omni_demo/main.c`
- `firmware/rm_f407/freertos/robot_tasks.c`

基线中：

- `omni_demo` 主循环末尾有 `HAL_Delay(5)`
- FreeRTOS 主框架里 `DaemonTask()` 约 100Hz
- `MotorControlTask()` 约 500Hz

也就是说，原框架默认假设 daemon 和 motor 都在“受控频率”下运行。

### 修复措施

本次在 `tests/firmware/infantry/infantry_gimbal_et08_demo/main.c` 中采取了两条修复：

1. 给 `DaemonTask()` 固定周期
   - `DAEMON_TASK_PERIOD_MS = 1`
2. 给 `DJIMotorControl()` 固定周期
   - `MOTOR_CONTROL_PERIOD_MS = 2`
3. 主循环末尾增加 `HAL_Delay(1)`
4. 电机重新 enable 时不再重写 `yaw_hold_ref/pitch_hold_ref`

### 后续规避规则

- 任何使用 `DaemonTask()` 的 demo，必须先确认它的调用频率是受控的。
- 不要把 `reload_count` 当作绝对毫秒，除非调用频率本身就是固定周期。
- 若 demo 不是 RTOS 任务模型，必须手动给：
  - `DaemonTask()`
  - `DJIMotorControl()`
  - 主控制逻辑
  分别设定明确周期。
- 对“重新上线/重新使能”的路径，要避免无条件把 `hold_ref` 改成当前测量值。

---

## 2. CAN / 电机控制频率坑：mailbox 满、掉帧、部分电机异常

### 现象

调试阶段曾出现：

- `CAN MAILbox full`
- 底盘某个电机不响应、乱转或明显和其他电机不同步
- 某一总线上别的电机也跟着表现异常
- 单独接某个电机时正常，多电机同时挂总线时出问题

典型例子就是：

- 底盘 `id1/id2/id3` 正常，`id4` 异常
- 同插 `can1` 的 yaw 电机也异常

### 根因

`DJIMotorControl()` 每调用一次，会对所有已注册 DJI 电机做一轮 PID 计算，然后按分组发送 CAN 帧。

相关实现：

- `components/devices/motor/dji/dji_motor.c`
- `platform/stm32f4/can/bsp_can.c`

关键机制：

1. DJI 电机按发送组打包
   - 最多 6 个 sender group
2. `DJIMotorControl()` 每轮都会遍历 sender group 并调用 `CANTransmit(..., 1)`
3. `CANTransmit()` 内部会等待邮箱空闲
4. 如果邮箱一直不空，超过超时就报：
   - `CAN MAILbox full!`

所以如果你在裸循环里高频调用 `DJIMotorControl()`，会出现：

- 发送请求堆得比总线实际发得还快
- 3 个 mailbox 很快占满
- 新控制帧排不上去
- 某些电机持续收不到新的控制命令/反馈
- 整条 CAN 上的行为变得不稳定

这不是单个 PID 参数问题，而是任务调度频率和总线吞吐不匹配。

### 本次确认的基线

`firmware/rm_f407/freertos/robot_tasks.c` 中明确写了：

- `MotorControlTask()` 采用 `osDelay(2)`
- 即约 `500Hz`
- 备注直接写明：`降低CAN发送频率避免mailbox满`

这说明框架作者本身也已经把 500Hz 视为一个经验上安全的上限级别。

### 修复/规避规则

- 裸机 demo 中不要让 `DJIMotorControl()` 跑满 while。
- 推荐先按：
  - `DJIMotorControl()` = `2ms` 周期（约 500Hz）
  - `DaemonTask()` = `10ms` 或 `1ms/5ms` 固定周期
  - 主控制逻辑 = `20ms` 周期
- 出现 mailbox 满时，优先排查：
  - `DJIMotorControl()` 是否裸跑
  - 某条 CAN 上挂了多少电机
  - 是否存在没接终端/接线问题
  - 是否某个设备不应答导致邮箱长期不释放
- 不要一上来就怀疑“某个电机 PID 特别怪”，先看总线频率和发送频率是否合理。

---

## 3. 给后续迁移的明确规则

### 3.1 裸机 demo 不等于可以裸 while 狂跑

只要用了下面这些模块，就必须先补调度：

- `DaemonTask()`
- `DJIMotorControl()`
- 各类串口解析/联锁状态机

### 3.2 “时间参数”必须先确认它依赖的时基

例如：

- `reload_count`
- `timeout`
- `hold_ms`

这些名字看起来像“时间”，但实际可能依赖任务调用频率，不是绝对时间。

### 3.3 出现“目标自己变了”，优先查状态机与使能边沿

排查顺序：

1. 是否真的进入了离线/停机/恢复路径
2. 是否在恢复路径里重写了参考值
3. 是否输入噪声导致误切模式
4. 最后才去怀疑 PID

### 3.4 出现“多个电机一起怪”，先查总线，不先查单电机参数

优先查：

1. CAN 发送频率
2. mailbox 是否满
3. 分组发送是否合理
4. 接线/终端电阻/供电
5. 再查单电机方向和 PID

---

## 4. 当前建议保留的调度基线

对于 `infantry_gimbal_et08_demo`，当前建议：

- `DaemonTask()`：`1ms`
- `DJIMotorControl()`：`2ms`
- 主控制逻辑：`20ms`
- 文本日志：`100ms`
- VOFA：`20ms`

这套频率至少满足当前“稳定调云台”的需求，后续如果迁回最小框架，也应优先保留这一调度思想，而不是恢复成裸跑。
