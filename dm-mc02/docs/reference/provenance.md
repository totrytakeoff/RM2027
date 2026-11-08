# 文件来源与适配边界

导入时间：2026-10-07，2026-10-11 补充底盘算法和开发脚本。完整逐文件记录见 [import_manifest.json](import_manifest.json)，当前 180 个来源记录（含改写的脚本入口）。`source_sha256` 对应原始来源；`implementation_sha256` 对应本轮适配后快照；`adapted` 标明是否改写。这里的外部路径仅用于追溯，构建不依赖它们。

2026-10-08 审计修复后刷新实现哈希；startup/system 及 PID 的新改动见 [审计修复](../archive/milestones/2026-10-08_audit_fixes.md)。构建清单另记录完整源码与构建配置哈希，以覆盖本工程自有源码。

| 内容 | 来源 | 当前选择及说明 |
| --- | --- | --- |
| HAL | `dm-mc02/damiao_mc02_bsp/dm02_test/Drivers/STM32H7xx_HAL_Driver` | 1.11.5；同源 Inc/Src，按当前外设选择编译单元；保留 LICENSE.txt |
| CMSIS Core | 同 demo 的 `Drivers/CMSIS/Include` | 当前头部标记 Core(M) 5.3；保留 Apache-2.0 文本 |
| CMSIS Device | 同 demo 的 `Drivers/CMSIS/Device/ST/STM32H7xx` | H723 device/system 配套；保留设备组件 LICENSE.txt |
| generated/startup | 同 demo 的 `Core/Src`、`Core/Inc`、`Core/Startup` | 保留 ST 头部，对板级初始化、IRQ、UART/FDCAN 和 HAL 配置作适配 |
| 原始 `.ioc` | 同 demo 的 `dm02_test.ioc` | 保留在 `platform/boards/dm_mc02/reference`，只作参考，未与新源码同步 |
| FreeRTOS kernel / ARM_CM4F | `RM2026/third_party/freertos_kernel/Source` 对应文件 | 10.3.1；只编译 tasks/list/queue 和所选 GCC port；无 heap 实现 |
| FreeRTOS ARM_CM7/r0p1 / LICENSE | FreeRTOS-Kernel 上游固定 tag `V10.3.1-kernel-only` | port.c、portmacro.h、LICENSE；URL 和 SHA-256 已记入清单 |
| CMSIS f32 matrix | 官方 `例程/CtrBoard-H7_ALL/Drivers/CMSIS/DSP/Source/MatrixFunctions` | 六个文件头标记 V1.6.0；改为最小矩阵头，选择 scalar 实现；保留 Apache-2.0 文本 |
| Kalman / QuaternionEKF | RM2026 对应算法目录 | 改为工作区和实例 context，初始 covariance 固定、矩阵失败传播、数值保护；保留 Wang Hongxi 署名 |
| remote state | `RM2026/components/devices/remote/remote_control_state.h` | 复用标准输入数据结构；新 parser 参考 et08/dt7 packing 与映射 |
| PID | `RM2026/components/algorithms/pid/controller.*` | 保留 Wang Hongxi 署名；显式 dt、无平台依赖、字段初始化和数值保护 |
| safety / vision | `RM2026/components/services/safety` 与 `components/devices/vision` | 复用纯状态/协议文件；保留来源头部 |
| chassis kinematics / follow | `RM2026/applications/robot/chassis/chassis_kinematics.*` 与 `chassis_follow.*` | 四个纯 C 文件原样复用；底盘 app 的输入映射、参数和独立 IMU yaw 适配参考原 robot 配置、ET08 profile、chassis/gimbal 控制逻辑，参考哈希见清单 |
| 开发脚本 | `RM2026/scripts` 的 9 个文件 | 保留入口名称；构建/烧录恢复原生 Bash，PowerShell 烧录直接调用 OpenOCD，无 Python 依赖；调试/日志通过 `scripts/devtools.py` 适配；原构建/烧录/调试脚本作者 YZ-Control/myself；VT 监听工具原样复制，RTT 仅供含 RTT 控制块的固件使用 |
| BMI088 寄存器及量程流程 | `RM2026/components/devices/imu/bmi088/BMI088driver.c` | 借鉴流程，未直接复制旧驱动文件；新状态机注明原作者 Wang Hongxi |

本轮新增的 linker、platform API、H7 transport/time/cache、runtime、INS/remote/DJI 服务、output gate、logging、bringup/底盘应用与构建组织均放在本工程内维护。未引入参考工程整套 C++ 应用、定时业务轮询或 CMSIS-DSP 预编译库；INS 噪声及 ET08 通道映射参考旧默认值。底盘 demo 的 CAN1 四电机 ID 已按用户布局配置，机械尺寸仍沿用原工程默认值，车头方向与尺寸尚未核实，见 [底盘说明](../guides/chassis_demo.md)。

上游 FreeRTOS 下载来源：

```text
https://raw.githubusercontent.com/FreeRTOS/FreeRTOS-Kernel/V10.3.1-kernel-only/portable/GCC/ARM_CM7/r0p1/port.c
https://raw.githubusercontent.com/FreeRTOS/FreeRTOS-Kernel/V10.3.1-kernel-only/portable/GCC/ARM_CM7/r0p1/portmacro.h
https://raw.githubusercontent.com/FreeRTOS/FreeRTOS-Kernel/V10.3.1-kernel-only/LICENSE
```

本地来源目录包含未跟踪 demo 文件，因此采用内容哈希定位；不能以参考仓库提交号代替这些文件的版本。RM2026 行为基线为 `4cb0020`。

后续更新 imported 文件时保留来源记录，并刷新实现哈希；新增导入补充清单。许可说明按文件所属组件保留，当前未为全部 RM2026 用户组件另行声明统一许可证。

## 2026-10-17 麦轮 demo 派生

`applications/mecanum_demo` 与对应 config/main 从当前全向轮 demo 复制并独立命名，替换X型麦轮运动学、轮序/极性与坐标约定，新增后PID电流限幅和功率代理遥测。未覆盖原应用配置。复制来源与当前实现 hash 见 [本轮构建记录](../archive/records/2026-10-17_mecanum_build.json)。新增可选 motor-owner hook 默认空，原全向轮不启用麦轮功率限制。

麦轮后续简化：按用户要求移除上述后PID限流与可选hook，仅保留诊断任务内的只读功率估算；限幅版构建记录属于历史版本。


## 2026-11-08 单电机测试扩展

M3508 分支复用本框架 DJI 控制、ET08 映射、安全门与 268/19 减速比配置；DM4310 协议依据用户提供的 DM-J4310-2EC V1.1/V1.2 官方手册。DM 主控位置/速度/力矩串级、MIT torque-only 打包及积分负载保持参考用户本地 `Leg/my-wheelleg` 的 8009P 实现，DM4310 单独设参。新增实现位于 `applications/motor_test/` 与 `components/devices/motor/dm/`，来源和验证边界见 [DM4310 指南](../guides/motor_test_dm4310.md)。
