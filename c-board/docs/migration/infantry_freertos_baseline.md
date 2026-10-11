# 步兵 FreeRTOS 迁移基线

更新日期：2026-10-02

## 1. 范围

第一阶段只针对单板步兵固件。英雄、轮腿、双板通信、控制参数重调和新控制算法
不在本基线范围内。

步兵行为唯一事实源为 `applications/robot`：

- `app.elf` 通过全静态 FreeRTOS 任务执行该逻辑。
- `test_infantry_minimal` 在原裸机调度外壳中执行同一套应用和电机阶段 API，用于实板行为对照；
  两个入口都在注册端点前选择延后 CAN/UART dispatch，并在电机阶段先派发有界 inbox。
- 废弃 `Src/application` 已删除，仅从 Git 历史追溯。

## 2. 当前控制行为

- ET08、DT7/DR16、VT 由 `ROBOT_REMOTE_BACKEND` 在编译期三选一；当前正式配置只选择 ET08。
- 不进行运行时输入仲裁、热备接管或自动回退，未选中的遥控驱动不会进入最终镜像。
- 设备层只发布统一的摇杆、开关、旋钮、键盘、鼠标和按键状态；应用 Profile 只形成
  `[-1,1]` 无量纲操作意图，执行层独占物理速度、机械限位、运动学和电机饱和。
- 高层控制顺序保持“云台→底盘→发射”。
- 高层控制周期保持 20 ms，电机阶段保持 5 ms。
- 底盘满量程已按 M3508 配置转速、标准减速比和轮径放开；PID、方向、减速比和机械零位
  必须在离地、可断电条件下逐项核验。
- 输入离线、急停、关键任务不健康、INS 未就绪或初始化失败时，底盘、云台和发射一起停止。
- 当前调试配置采用 SA 电平安全门：SA 上位且硬故障清除即进入 `ACTIVE`，SA 下位立即
  关闭全局电机输出；不要求摇杆回中或捕获新的 SA 边沿。
- 单个电机离线只记录 `ERROR` 并由电机层将对应 CAN 输出槽归零，不再阻止其他在线电机运行。
  可通过 `ROBOT_SAFETY_GATE_ON_MOTOR_HEALTH` 恢复整车联锁。

遥控抽象、ET08 映射及重使能规则详见
[遥控适配器与控制分层基线](../architecture/remote_control_adapter_baseline.md)。

## 3. FreeRTOS 任务基线

所有应用任务使用 `xTaskCreateStatic` 和 `vTaskDelayUntil`。

| 任务 | 周期 | 优先级 | 栈 |
| --- | ---: | ---: | ---: |
| `ins` | 1 ms | idle + 4 | 1024 words |
| `motor` | 5 ms | idle + 3 | 384 words |
| `health` | 5 ms | idle + 3 | 256 words |
| `control` | 20 ms | idle + 2 | 768 words |
| `diagnostics` | 10 ms | idle + 1 | 384 words |
| `tuning_uart` | 20 ms | idle + 1 | 256 words |
| `usb_init` | 一次性 | idle + 1 | 128 words |

FreeRTOS 动态分配关闭，启用栈溢出检查和分配失败 hook。当前栈值在取得实板
high-water 记录前保守设置。

关键任务记录心跳、释放间隔、工作执行时间、deadline miss 和历史最小剩余栈。
以下任一条会将安全状态推入 `STOPPED`：

- 100 ms 启动宽限后，关键任务连续四个配置周期无心跳；
- 连续三次释放间隔超过周期的 125%，或工作执行时间达到/超过任务周期；
- 历史剩余栈低于 64 words。

`diagnostics` 和 `tuning_uart` 可观测但不作为安全关键任务。当前视觉联调配置默认开启
低频通用文本日志、关闭 `tuning_uart`，避免两种数据流争用 UART6；需要 VOFA 波形时
再切换为 `MINIMAL_DEBUG_ENABLE=0`、`ROBOT_TUNING_TELEMETRY_ENABLE=1`。
`tuning_uart` 使用 UART6 DMA，串口忙时丢帧而不阻塞控制任务。

## 4. 已建立的运行时约束

- CAN、UART、设备健康、DJI 电机、EKF、诊断格式化和 FreeRTOS 任务使用静态存储。
- 设备超时使用单调毫秒截止，不依赖健康任务轮询频率。
- 正式固件和 `test_infantry_minimal` 在注册设备前选择延后 CAN/UART dispatch；中断只保留
  有界数据，裸机循环与 FreeRTOS 的 5 ms 电机阶段都先解析 inbox。
- 当前选中的统一遥控快照、DJI 反馈和 INS 通过一致快照跨任务读取。
- 20 ms 应用任务发布每个电机的完整命令；5 ms 电机任务独占 PID 运行态和 CAN 输出。
- 正式组合启用 100 ms 命令租约，即使控制任务卡死，电机任务也会拒绝过期命令。
- BMI088/INS 初始化总预算 15 s，失败后保持全局输出门关闭。
- IWDG 只由 5 ms 健康任务且只在所有关键任务健康时喂狗。
- 新建的 HardFault 路径关中断、保留异常记录，然后等待 IWDG 复位或调试器检查。

## 5. 构建基线

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target app.elf test_infantry_minimal --parallel
cmake --build build --parallel

cmake -S tests/host -B build-host -DCMAKE_BUILD_TYPE=Debug
cmake --build build-host --parallel
ctest --test-dir build-host --output-on-failure
```

当前期望门禁：

- 正式 `app.elf` 成功构建；
- 裸机对照 `test_infantry_minimal` 成功构建；
- 30 个嵌入式 demo/回归 target 全部成功构建；
- 16 个主机测试 target 已注册；运行结果以独立 `ctest --test-dir build-host --output-on-failure`
  记录为准；
- 当前默认视觉联调配置（`VISION_ENABLE=1`、文本日志开启、VOFA 关闭）的 Debug
  `app.elf` 构建报告 RAM 55,272 字节、Flash 109,392 字节；资源数随视觉/日志开关变化，
  以上一轮链接器输出为准；
- 8 KiB 主栈预留、0 字节 C 堆预留，并包含一份 `.noinit` HardFault 保留记录；
- 正式镜像不链入 C/FreeRTOS 堆分配、libc 格式化、C++ 动态分配、异常、RTTI 或静态局部对象 guard 运行时符号；
- 嵌入式构建无 libnosys syscall 警告，ELF LOAD 段无 RWX。

## 6. 实板验收清单

在 FreeRTOS 固件取代对照固件并开始调参前，必须记录：

- [ ] 冷启动两秒稳定期内所有输出保持安全。
- [ ] BMI088 断开或不断移动时，15 s 内失败退出且执行器不动作。
- [ ] SA 上位且遥控、初始化、INS 和关键任务正常时直接进入 `ACTIVE`；SA 下位立即停止。
- [ ] SA 下拨、ET08 失联、failsafe 或坏帧都同时禁用底盘、云台、摩擦轮和拨弹电机。
- [ ] ET08 重连且 SA 保持上位时按当前电平门恢复；确认恢复前收到的运动命令始终受
  执行层限幅，SA 下位时绝不恢复。
- [ ] 分别拔掉一个电机反馈，确认日志报错、该电机输出归零且其余在线电机仍可控制。
- [ ] 暂停控制任务后，所有 DJI 命令槽在 105 ms 内归零。
- [ ] SB 下位为云台—底盘跟随，中位为视觉自瞄（离线时安全降级为跟随），上位为小陀螺。
- [ ] yaw/pitch 反馈方向正确；Pitch 正参考必须产生正 GyroX，`hold_target` 与 IMU Pitch
  同向且可直接比较，软/硬限位可阻止越界并允许向安全区退出。
- [ ] SC 下位禁用发射，中位单发，上位连发；SD 上拨触发，单发长按不会重复排队。
- [ ] 5 ms 电机任务下无 CAN 邮箱持续饱和。
- [ ] 所有任务执行时间和释放间隔在至少 10 分钟运行中有界。
- [ ] 所有栈 high-water 留有约定安全余量，不进入栈溢出/分配失败 hook。
- [ ] 分别卡住关键任务和整个调度器时会触发 IWDG 复位，正常极限负载不误复位。
- [ ] USB 初始化完成后一次性任务正常退出。
- [ ] 裁判系统启用后，只读限制与功率/热量保护逻辑符合预期。

## 7. 已知过渡技术债

- 正式组件集仍需继续拆分为更窄的算法、服务和设备 target。
- 未链入 `app.elf` 的多个兼容模块仍提供堆注册 API，不得直接提升到正式固件。
- 部分模块 API/配置宏仍保留 `Minimal*` 命名。
- `DJIMotorInstance` 为历史 demo 保留公开可变字段；正式应用已不依赖该路径。
- INS 在首次发布后尚无独立的传感器合理性/data-ready 运行时截止。
- `g_robot` 上下文仍与诊断共享。
- 应用公开头仍间接暴露部分板级生成头，结构已分层但接口未完全纯化。
- IWDG 实际超时和 fault injection 必须在每块物理控制板上测量。
- CubeMX `.ioc` 尚未追踪。
- 正式主栈暂保守预留 8 KiB，需根据中断嵌套与启动实测收紧。

下一个软件所有权切片是 INS 运行时合理性、更窄的设备/服务 API，以及剩余非活动堆注册路径的移除。
