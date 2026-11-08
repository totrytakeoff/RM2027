# 当前验收状态

更新：2026-10-11。用户确认底盘 demo 基本可用，本轮以“基础框架可用、底盘 demo 完成基本联调”为收尾基线。此结论不等同于完整 RobotApp 已移植完成。

## 已完成

| 范围 | 结论与证据 |
| --- | --- |
| H723 平台与运行时 | 已实测启动、六任务运行、IMU、遥控接收和诊断；[硬件过程记录](../archive/milestones/2026-10-11_hardware_log.md) |
| 故障处理 | 已实测串口启动失败/恢复/隔离、任务停滞、早期看门狗、FPU 异常及停止请求；[确认记录](../archive/hardware/2026-10-11/bringup_verified/) |
| 底盘封装与应用 | ET08、四 M3508、全向轮解算、IMU yaw 和 USART1 诊断接入；用户确认基本可用；[使用指南](../guides/chassis_demo.md) |
| DAP v2 | 已有成功烧录、Flash 校验和复位证据；[前后修订版烧录](../archive/hardware/2026-10-11/chassis/chassis_forward_fix/flash.json) |
| 构建组织 | Debug/Release/观察/DBUS/host 构建记录已保留；[历史快照](../archive/records/build_manifest_2026-10-11.json) |

当前轮序为前左4、前右1、后左3、后右2；遥控平移极性 X=-1、Y=+1。当前源码 yaw Kp40/Ki0/Kd2、角速度上限2.5 rad/s、目标速度90 deg/s，以配置文件为准。用户调参后的基本可用反馈与早期自动采集数据分别记录，不能把旧 hash 的时序/测试结果直接套到所有后续版本。

## 证据边界

- 早期主机测试覆盖遥控映射、yaw 跨圈、模式恢复、非法数值停机、M3508 串级和命令打包；前后极性修订后未重新运行主机测试。
- 底盘动态使用结果来自本次用户反馈；尚无对当前调参版本的完整负载、长时间运行和每项故障场景的量化记录。
- 原生烧录脚本已按 RM2026 形式恢复，默认 BIN/DAP v2、不依赖 Python；本次脚本替换没有再次通过脚本烧录验证。

## 后续工作

- 完整 RobotApp：云台、发射、裁判、视觉和应用协作。
- BMI088 DRDY/DMA 采集与时序对照、IMU 安装轴和机械尺寸标定。
- DT7/DBUS、其他总线与设备功能的实物验收。
- CAN 压力/断连恢复、负载控制和长时间运行的专项记录。

完整目标与接口约束见 [移植方案](../architecture/porting_plan.md)。原始记录与失败尝试均保留在 [归档](../archive/README.md)，本轮整理不操作板子、不改变固件。

## 2026-10-17 麦轮扩展

新增独立 `mecanum_demo`，X型布局前左4/前右3/后左1/后右2；带24V、50W电流代理功率预算和末端总电流限幅。Debug/Release/观察及兼容目标构建完成，未运行测试。2026-10-17 已用DAP v2烧录/校验并复位，串口10秒观测四电机在线、限流器配置正常；遥控离线，未做动态限流验证，见 [烧录记录](../archive/hardware/2026-10-17/mecanum/flash.json)。没有DC总线电流采样，不能将代理功率上限等同实际输入50W硬保证。使用与边界见 [麦轮指南](../guides/mecanum_demo.md)。

### 麦轮简化：移除功率限幅

用户改为手动遥控控制，仅需串口功率观测。已删除功率预算、总电流限幅、反馈回退、积分干预和motor hook，估算仅在诊断任务执行。新串口格式为 `PWR est_mW=... I_mA=... mask=...`。各构建通过，未运行测试或烧录，早期限幅版数据仅作历史记录。

无功率限幅版已完成DAP v2烧录、校验和复位，8秒串口观测四电机在线、遥控SA下档、功率估算格式正常，无LIM行；未做动态动作测试。[记录](../archive/hardware/2026-10-17/mecanum_telemetry_only/flash.json)。

### 麦轮 JustFloat 输出

USART1已改为115200/50Hz的7通道JustFloat，输出估算功率、绝对电流总和、ID1..4有符号电流及在线掩码；原文本不再混入该串口，框架文本转USART10。已通过DAP v2烧录并校验，通道表见 [麦轮指南](../guides/mecanum_demo.md)。

### 麦轮均值显示

按用户要求，显示改为10ms采样、200ms窗口算术平均后发送JustFloat（约5Hz），保持7通道不变；逐样本取绝对值后再平均功率，在线状态变化时重置窗口。已构建并通过DAP v2烧录/校验；串口占用，未再次抓取。[记录](../archive/hardware/2026-10-17/mecanum_justfloat_average/flash.json)。


## 2026-11-08 motor test

- M3508/C620 ID1：独立速度/位置例程，SD 下左摇杆 Y 控速，SD 上右摇杆 Y 映射输出轴 ±π，SA 总使能。268/19 减速箱下速度 PI 已实测整定为 Kp=20、Ki=80；用户确认速度/位置操作效果正常。串口量化证据见 [整定结果](motor_test/results.json)及[对比图](motor_test/tuning.png)。
- DM4310：主控位置→速度→力矩串级，MIT Kp=Kd=0；已构建、烧录校验并确认配置/反馈在线（cfg=31、err=0、o=1），见 [在线记录](motor_test/dm4310/online.json)。本轮尚未开启操作使能进行闭环整定；初始增益不能视作验收参数。
- 主机测试覆盖原底盘、motor test 模式映射、DM 协议及 DM 启用/停机状态机；相关目标完成 ARM 编译。使用说明见 [M3508](../guides/motor_test.md) 与 [DM4310](../guides/motor_test_dm4310.md)。
