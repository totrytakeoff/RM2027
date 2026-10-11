# 步兵裸机对照固件

文档更新日期：2026-10-02

该 target 保留原有单循环调度外壳，用于 FreeRTOS 迁移期间的硬件对照，已不再持有步兵控制逻辑。

该对照 target 与正式固件 `app.elf` 都会编译 `applications/robot` 下的模块：

```text
applications/robot
  command -> 编译期单一遥控后端、统一状态与机器人语义映射
  chassis -> 麦轮运动与底盘跟随控制
  gimbal  -> 云台偏航/俯仰控制
  shoot   -> 摩擦轮与拨弹控制
  referee -> 只读裁判系统联锁
```

对照 target 在轮询循环中调度 INS、守护服务、延后 CAN/UART 入站事件和
`RobotApp_MotorStep()`，每 20 ms 执行一次上层应用。正式固件则由静态 FreeRTOS 任务调用
相同 API。两者共享控制参数、命令快照、100 ms 命令租约、安全门控、入站分发模式和应用
调用顺序，因此实测差异应主要来自调度机制。

上机对照前同时构建两个 target：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target app.elf test_infantry_minimal --parallel
```

本 target 不保留旧 VT/ET08 仲裁实现，当前行为与正式应用一致。验收清单统一维护在
`docs/migration/infantry_freertos_baseline.md`。
