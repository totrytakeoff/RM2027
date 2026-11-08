# DM-J4310-2EC 电机测试

DM4310 接入相同 motor test 应用：SA 上档总使能；SD 下档左摇杆 Y 控速；SD 上档右摇杆 Y 控位置（进入该模式或重新使能时捕获零位，输出轴 ±π rad）；SD 中档关闭输出。

## 控制路径

主控位置 PID → 输出轴速度目标 → 主控速度 PID → 输出轴力矩 N·m → MIT torque 字段。位置环和速度环在 MOTOR 任务内以实际 5 ms 周期计算；MIT 帧的 Position/Velocity 字段为编码零值，Kp=Kd=0。电机内置的位置/速度控制增益不参与本测试。参考了 `Leg/my-wheelleg/applications/wheelleg_bringup/joint_control.c` 的 8009P 力矩串级和 `joint_units.c` 的单位转换，并为 DM4310 独立设置参数。

DM4310 的协议位置、速度和力矩均按输出轴单位使用，**不再乘 10:1 机械减速比**。代码中的 `MOTOR_TEST_REDUCTION=1` 表示该接口的单位换算比例，不代表物理减速箱为 1:1。原 M3508 分支仍使用转子反馈与 268/19 比例。

## 配置和构建

在 `config/robots/motor_test/motor_test_config.h` 中选择 `MOTOR_TEST_MODEL_DM4310`，或直接构建单独目标（目标自动定义 `MOTOR_TEST_MODEL=4310`）：

```sh
./scripts/build.sh build --target motor_test_dm4310
./scripts/upload.sh motor_test_dm4310
```

`motor_test` 保留原默认 M3508 分支，产物与 DM4310 独立，均支持现有输出门、命令 epoch/租约、遥控失联和板载急停。

当前 DM 分支接线默认 CAN1，电机 ID=1，Master ID=0，CAN 1 Mbps，电机已配置 MIT 模式（CTRL_MODE=1）。实际接线与参数需核对实物；除下述受限的临时 TIMEOUT 配置外，本应用不写入其他电机参数，不保存零位，也不自动清除故障。

启动后先发送失能帧并只读以下寄存器：PMAX(0x15)、VMAX(0x16)、TMAX(0x17)、CTRL_MODE(0x0A)、TIMEOUT(0x09)。若 TIMEOUT 为 0 或超过 2000 count，且已收到新鲜的 Disabled 反馈、输出门关闭，会临时写入 TIMEOUT=1000 count（50 ms），随后继续只读扫描，读回确认后才允许使能；该值不保存 Flash，电机断电后下次启动会再次检查。收到完整且合法的配置、确认 MIT 模式且硬件超时为 1～2000 count（每 count 50 μs，即不超过 100 ms）以及有效反馈后，才允许使能。PMAX/VMAX/TMAX 是编码量程，按实读值解码，不硬编码为电机机械性能。更改电机参数后应重启主控，重新读取配置。

反馈同时匹配 CAN 总线、Master ID、电机低四位 ID、传输 generation 和 20 ms 新鲜度。使能时先发零力矩，再发 FC 使能，并等待 Enabled 状态才进行闭环。停止时周期发送 FD 失能。失能与限定寄存器读取可以通过输出门；FC 使能、普通力矩、FE 归零、FB 清故障与其他写参数均不能走该旁路。TIMEOUT 写入使用独立接口，仅接受寄存器 0x09、值 1000，不能写成 0 或修改控制模式。即使 MOTOR 任务停滞，已确认的硬件 TIMEOUT 仍负责电机侧超时停机。

## 初始参数和边界

DM4310 速度 PI 初值 Kp=0.2、Ki=0.5（单位 N·m / (rad/s)，积分按秒），位置 P=4 s⁻¹。速度环在零误差处保留积分负载补偿，停机或模式切换时显式清 PID。速度目标限幅为配置速度上限与实读 VMAX 的较小值；速度 PI 输出限幅为配置力矩上限与实读 TMAX 的较小值。

默认配置暂按 V1.1 / 24V：最大速度目标 200 rpm（20.94395 rad/s），力矩上限 7 N·m。目录中的 V1.2 手册给出的峰值为 12.5 N·m，24V/48V 最高空载转速也不同；须确认实物后修改型号分支，不能将初值视为已经实测整定。DM4310 分支已完成主机测试、ARM 构建和烧录校验；首次启动未收到电机回复；重新连接后已连续读到 cfg=31、err=0、o=1，配置完整且反馈在线。当前操作使能关闭，尚未完成闭环实板整定，见 [在线验证记录](../validation/motor_test/dm4310/online.json)。

位置范围是摇杆目标范围，不能代替机械限位。位置反馈丢失连续性后停止，需 SA 关闭或退出位置模式后重新捕获零位。

## USART1

115200 8N1，尝试每 10 ms 输出（忙时可丢样）：

- `DT`：`r` 速度目标、`v` 反馈速度，单位 mrad/s；`q` 反馈力矩、`u` 已提交力矩，单位 mN·m；`e` 使能。
- `MP`：`p`、`a` 是相对零位的目标/实际输出轴位置，单位 mrad；`v` 是输出轴 rpm。
- `DC`：每秒输出配置接收掩码 `cfg`（完整为 31），`err` 配置/反馈故障位、`w` 应用停止原因及 `o` 反馈有效标记。`err` bit0 配置不全、bit1 模式不为 MIT、bit2 硬件 TIMEOUT 无效或超过 100 ms、bit3 编码量程无效、bit4 电机故障状态。

## 来源与验证

协议来源：用户提供的 `dm/DM-J4310-2EC/说明书/DM-J4310-2EC V1.2 Geared Motor User Manual V1.4 2026-09-14.pdf`，p14～17 反馈、MIT、使能/失能/寄存器协议，p20～21 寄存器，p29 超时时基；V1.1 手册用于比较性能规格。

主机测试覆盖 MIT 纯力矩打包（逐帧断言 Kp=Kd=0）、反馈单位、配置读取、失能/只读旁路白名单、启用握手、速度/位置力矩方向、输出门关闭、传输代次与反馈 TTL，以及零误差积分保持。


## 当前实物参数截图

用户截图确认：CAN ID=0x01、Master ID=0x00、CAN 1 Mbps、MIT 模式；PMAX=12.5、VMAX=30、TMAX=10、物理减速比 10。截图中的 CAN Timeout=0，已针对该初始配置增加临时 50 ms watchdog 设置和读回确认。TMAX=10 为通信量程，不能单凭它判定实物版本的机械峰值力矩；默认物理力矩上限仍为 7 N·m，待版本/供电确认后再匹配。
