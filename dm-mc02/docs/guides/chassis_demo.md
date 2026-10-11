# MC-02 四全向轮底盘测试 app

当前状态：用户已确认 demo 基本可用，验收范围见 [当前状态](../validation/README.md)。接线和参数以配置源码为准。

入口：`firmware/dm_mc02/chassis_main.c`。控制代码在 `applications/chassis_demo`，接线和参数在 `config/robots/chassis_demo/chassis_config.h`，四电机绑定在同目录 `binding.c`。

## 1. 运行路径

复用现有 MC-02 的板级初始化、缓存/DMA 分区、BMI088/INS、SBUS 流式接收、CAN 队列、DJI 控制器、安全输出门和静态 FreeRTOS。六个任务周期仍为 INS/I/O 1 ms、motor/health 5 ms、control 20 ms、diagnostics 10 ms。

I/O 发布完整反馈快照，CONTROL 只读取快照并一次提交四个电机命令，MOTOR 独占四个速度/电流 PID。命令带 epoch 与 100 ms 有效期；串口/CAN 异常、反馈过期、控制任务停滞与输出门停止沿用现有封装。新增 `RmBringup_ReadMotor` / `RmBringup_SubmitMotorCommands` 是这个 demo 使用的设备读取/命令提交接口。

## 2. ET08 操作

| 控件 | 行为 |
| --- | --- |
| SA 下 | 关闭输出，四轮零电流命令，重置控制历史 |
| SA 上 | INS 就绪且启动满 2 s、遥控和任务健康时使能 |
| 左摇杆左右 | 平移，沿用原 C 板 `-left_x` 映射 |
| 左摇杆前后 | 前后平移，按本车反馈改为 `+left_y`；方向通过 `CHASSIS_DEMO_REMOTE_Y_SIGN` 配置 |
| 右摇杆左右 | 改变 yaw 目标；松杆保持目标，IMU 闭环跟随 |
| SB 下 | FOLLOW，yaw 跟随 |
| SB 中 | 原 AUTO_AIM 槽位；本独立底盘 app 采用 FOLLOW，无视觉接管 |
| SB 上 | 小陀螺，原理论旋转上限的 60%；优先保留旋转、压缩平移 |
| SC、SD、右摇杆上下、旋钮 | 保留接收与诊断，无底盘动作 |
| 板载按键按下 | 急停；松开后遵循 SA 电平门策略 |

摇杆采用原 100 raw 死区及 660 单边量程，死区外重新映射为满量程。原 SA 电平使能策略保留：`CHASSIS_DEMO_REQUIRE_REARM=false`。默认单轮离线只隔离该轮；`CHASSIS_DEMO_REQUIRE_ALL_MOTORS=true` 可改为四轮联锁。

### 独立底盘的 yaw 适配

原 C 板通过云台 yaw 编码器的相对夹角进行坐标变换和底盘跟随。本 demo 没有云台电机，采用右摇杆生成的“虚拟云台目标方向”，其目标与底盘 IMU yaw 的差代替相对夹角。每次使能捕获当前 yaw，松杆保持目标；小陀螺退出后沿最近等价角恢复跟随，并绕过恢复首拍角速度滤波。

原运动学、平移/旋转死区、0.15 旧值权重、轮速共同限幅和小陀螺旋转优先算法保留。根据实车超调/震荡反馈，当前 yaw 配置采用 `Kp=40, Ki=0, Kd=2`，外环上限 2.5 rad/s；右摇杆目标推进速率为 90 deg/s。控制量为 `Kp*角误差 + Ki*积分 - Kd*gyroZ + 目标角速度`，D 项仅用实测角速度，目标角速度作为独立前馈。这组参数来自当前调参配置；demo 已由用户确认基本可用，量化负载与长时间运行范围见验收状态。IMU 应固定于底盘，轴映射当前 `{1,2,3}`；yaw 极性由 `CHASSIS_DEMO_IMU_YAW_SIGN` 指定。

## 3. 接线与参数

按用户给出的俯视布局 `4 1 / 3 2` 配置，上排作为车头；机械尺寸仍沿用 RM2026 默认值。实际接线必须对应这些明确绑定：

| 轮子 / 诊断索引 | 总线 | 电调 ID | 反馈 ID | 命令 ID |
| --- | --- | --- | --- | --- |
| 前右 / M0 | CAN1 | 1 | 0x201 | 0x200 |
| 前左 / M1 | CAN1 | 4 | 0x204 | 0x200 |
| 后右 / M2 | CAN1 | 2 | 0x202 | 0x200 |
| 后左 / M3 | CAN1 | 3 | 0x203 | 0x200 |

当前轮序 FR1/FL4/BR2/BL3，遥控输入极性 X=-1、Y=+1，在 heading 坐标变换之前应用。四电机 sign 均为 +1。轮序与方向调整过程见 [历史硬件记录](../archive/milestones/2026-10-11_hardware_log.md)。轮半径 0.075 m、轮距参数 0.34 m、减速比 19.20320856，转子上限 523.5987756 rad/s（5000 rpm）；由此得到最大平移约 2.045 m/s、理论纯旋转约 12.029 rad/s。速度 PID 为 `229.18312 / 2.291831 / 0`，电流环 Kp 0.35，限幅 15000 C620 命令单位，反馈超时 20 ms。

ET08 接收机接原 MC-02 SBUS 口（UART5/PD2），100000 8E2。USART1：**PA9 TX → DAP RX，PA10 RX ← DAP TX，GND 共地**，115200 8N1。该目标的 USART1 用作文本诊断；接收侧只统计字节，不执行控制命令。`board_bringup` 的 USART1 仍按其独立目标设置为视觉口。

## 4. 构建与下载

日常构建和下载（默认 h7-debug，下载默认 BIN / DAP v2）：

```sh
./scripts/build.sh build --target chassis_demo
./scripts/upload.sh chassis_demo
# 或 ./scripts/build.sh upload
```

只烧录已有固件不需要 Python；完整参数见 [脚本说明](../../scripts/README.md)。直接 CMake 构建：

```sh
cmake --preset h7-debug
cmake --build --preset h7-debug --target chassis_demo.elf
# 产物 build/h7-debug/chassis_demo.{elf,bin,hex,map}
```

优化构建可用 `h7-release`，同样指定 `chassis_demo.elf`。DBUS preset 不生成 ET08 底盘目标。

观察配置会运行同一套输入、IMU、控制计算和诊断，但电机任务只提交零指令，诊断显示 `tx=0`：

```sh
cmake --preset h7-chassis-observe
cmake --build --preset h7-chassis-observe
```

下载所选配置，例如观察版本：

```sh
openocd -f platform/boards/dm_mc02/openocd/cmsis-dap.cfg \
  -c "cmsis_dap_backend usb_bulk" \
  -c "program build/h7-chassis-observe/chassis_demo.elf verify reset exit"
```

DAP v2 与 HID 均有成功烧录记录，当前默认使用 DAP v2。连接问题与旧版本实测数据归档于历史记录。

底盘运行版本将上面的路径改为 `build/h7-debug/chassis_demo.elf`，其诊断为 `tx=1`。烧录前保持 SA 下、主控静置，供 INS 校准；首次驱动验证时架起四轮，按 FR/FL/BR/BL 验证轮序及闭环 yaw 极性。

## 5. USART1 诊断

每约 200 ms 输出一组行，并共用框架 `RM ... event=...` 日志：

```text
CH t=2500 mode=0 en=0 tx=0 why=64 online=15
RC lx=17 ly=0 rx=0 ry=0 sa=3 sb=3
YAW ref=0 imu=0 err=0 rate=0
BODY vx=0 vy=0 wz=0 scale=0
M0 ref=0 fdb=0 cur=0 temp=30
```

`mode`: FOLLOW=0、AUTO_AIM 槽位=1、SPIN=2。`en` 表示应用使能请求已通过检查，`tx` 表示构建是否允许驱动力；`online` 四位对应 M0..M3。`why` 使用 `RmSafetyReason` 位，bit13 表示计算/发布或提交前复核失败。

`YAW` 为 mrad、mrad/s；`BODY vx/vy` 为 mm/s，`wz` 为 mrad/s，`scale` 为保留平移比例×1000；`M* ref/fdb` 为转子 mrad/s，`cur` 为电调反馈电流原始单位，`temp` 为 °C。所有发送均复制到封装拥有的 DMA 缓冲，控制任务不进行格式化或串口等待。

串口捕获：

```sh
python3 tools/serial_capture.py --duration 60 --output /tmp/chassis_uart.txt
python3 tools/board_probe.py --elf build/h7-chassis-observe/chassis_demo.elf \
  --duration 60 --output /tmp/chassis_swd.json
```

## 6. 已有验证与当前边界

早期 Debug/Release 构建和主机测试覆盖 ET08 映射、死区、yaw 正负反馈/跨圈、SPIN 旋转优先、模式恢复、非法数值停机、M3508 速度/电流串级及 0x200 打包/过期清零。前后极性修订后没有重新执行主机测试。需要自行重跑时使用：

```sh
cmake --preset host -DRM_BUILD_TESTS=ON
cmake --build --preset host
ctest --test-dir build/host --output-on-failure
```

基础固件实测、用户底盘联调反馈与尚未完成的专项验收统一见 [当前验收状态](../validation/README.md)。早期数值与故障过程见 [硬件历史记录](../archive/milestones/2026-10-11_hardware_log.md)。
