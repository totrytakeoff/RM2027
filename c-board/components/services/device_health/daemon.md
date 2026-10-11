# 设备健康监测服务

`device_health` 用固定容量注册表监测电机、遥控器、裁判系统和视觉链路等
设备。它使用毫秒绝对截止时间，不依赖 `DaemonTask()` 的实际调度频率，也不
使用动态内存。

## 生命周期

系统单线程启动阶段先初始化服务，再注册所有实例：

```c
DaemonServiceInit();

static DaemonInstance *motor_health;

const DaemonConfig config = {
    .timeout_ms = 20U,
    .initial_grace_ms = 1000U,
    .callback = MotorOfflineCallback,
    .owner = motor,
};
motor_health = DaemonRegister(&config);
```

注册和 `DaemonServiceInit()` 只能在调度器启动前的单线程阶段调用。注册表最多
容纳 `DAEMON_MAX_INSTANCES`（当前为 64）个实例；配置无效或容量耗尽时，
`DaemonRegister()` 返回 `NULL`，调用方必须把它视为初始化失败。

`timeout_ms` 和 `initial_grace_ms` 设为零时分别采用 1000 ms 默认值。为了保证
32 位时钟回绕时的截止时间比较仍然无歧义，两个值都必须小于 `2^31` ms。

## 在线语义

只有收到并校验通过的有效数据后，设备才应调用：

```c
DaemonReload(motor_health);
```

新注册的实例在第一次有效喂入前始终不是在线状态。`initial_grace_ms` 只负责
延后启动阶段的第一次离线回调，并不会把尚未通信的设备伪装成在线。

`DaemonIsOnline()` 在设备已喂入且当前时刻尚未到达截止时间时返回 `true`。
`DaemonSetTimeout()` 只修改后续喂入使用的超时值，不延长当前截止时间。

## 任务与回调

健康任务周期调用 `DaemonTask()`。它会检查全部实例，并在每次“在线/等待中到
离线”的事件中只调用一次离线回调；下一次有效喂入会重新武装该事件。超时
精度来自毫秒时钟，任务轮询周期只决定发现离线事件最多会晚多久。

CAN/UART 中断可以安全调用 `DaemonReload()`，服务内部用原子状态协调中断喂入
与任务检查。离线回调由健康任务上下文执行，回调应保持有界、非阻塞，并通过
`owner` 找回所属设备实例。

统一时钟、当前设备超时表和验证结果见
[`docs/migration/platform_runtime_milestone.md`](../../../docs/migration/platform_runtime_milestone.md)。
