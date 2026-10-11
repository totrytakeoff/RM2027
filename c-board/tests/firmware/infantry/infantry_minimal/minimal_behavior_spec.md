# infantry_minimal 裸机对照行为规范

文档更新日期：2026-10-02

本 target 只保留裸机轮询调度外壳，实际机器人行为直接复用
`applications/robot`，不得再建立一套独立行为事实源。启动时先选择延后 CAN/UART
dispatch；每轮在 `RobotApp_MotorStep()` 前调用 `CANDispatchPending(0)` 和
`USARTDispatchPending(0)`，与正式 FreeRTOS 电机任务保持同一入站顺序。

## 1. 输入选择与映射

- ET08、DT7/DR16、VT 由 `ROBOT_REMOTE_BACKEND` 编译期三选一，不做运行时仲裁、接管或回退。
- 当前默认 ET08：SA 为全局安全门，SB 为跟随/自瞄预留/小陀螺，SC 为禁射/单发/连发，SD 为扳机。
- 设备层统一发布摇杆、开关、旋钮、键盘、鼠标和按钮状态，步兵应用层才赋予机器人语义。
- 任一输入离线、failsafe 或数据无效都进入统一安全停止路径。

## 2. 安全与失效处理
- 统一安全停止函数 `RobotApp_ForceSafeStop()` 负责停止底盘、云台、发射。
- 输入离线、急停触发、关键联锁拒绝时，都进入统一安全停止路径。
- 主循环保持 `DaemonTask()` 与 `RobotApp_MotorStep()` 常驻调用；后者统一封装
  `DJIMotorControl()`，避免对照外壳绕过应用层所有权。
- 启动或故障恢复后禁止自动恢复。必须先观测到“SA/SB/SC/SD 全下且四轴回中”，
  再产生新的 SA 上拨沿。

## 3. 闭环与环路策略
- 底盘：运行态速度环（必要时叠加电流环）。
- 云台：
  - Yaw 将速度意图积分为 IMU 总角目标；
  - Pitch 采用“手动速度 → 制动 → 松杆角度保持”，速度与角度目标都经过 IMU 机械限位。
- 发射：
  - 摩擦轮速度环；
  - 拨盘单发采用角度环目标；
  - 拨盘连发采用速度环。
- 所有执行器在 `applications/robot/config/robot_config.h` 用统一宏指定 init/run loop，避免硬编码分散。

## 4. 裁判系统只读联锁
- 仅接入关键字段：`robot_id`、`chassis_power_limit`、`shooter_heat_limit`、`shooter_heat`、`allowance_17mm`、功率输出使能位。
- 不引入 UI 绘制和 referee_task 任务流。
- 联锁策略：
  - 底盘按功率限制缩放输出；
  - 发射按热量和发弹量限制抑制拨盘与摩擦轮。

## 5. 验收场景（比赛前）

- 冷启动稳定性（30 s），SA 上位启动不能直接使能。
- SA 下拨、ET08 关机/failsafe、关键设备或任务故障能统一停机。
- 重连且 SA 保持上位不能自动恢复，完整重使能动作无跳变。
- 四轮方向、坐标旋转、底盘跟随和小陀螺方向正确。
- Yaw/Pitch 方向正确，Pitch 上下限符号与机械范围正确。
- SC/SD 的禁射、单发沿与连发电平行为一致。
- 裁判联锁触发后输出按预期收敛。
- 10 分钟连续运行无异常停机。

详细参数和步骤以
`docs/architecture/remote_control_adapter_baseline.md` 与
`docs/migration/infantry_freertos_baseline.md` 为准。
