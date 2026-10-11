# 视觉上下位机通讯协议

更新日期：2026-10-02

状态：**上下位机协议与链路代码已实现；实板验收待完成**。本文确定 STM32 电控
（下位机）与视觉主机（上位机）之间的串口通讯格式、字段语义、坐标约定与失效策略。

- 下位机：`components/devices/vision/`（已实现，见 §9）
- 上位机：`~/ros_ws/src/rm_serial_driver`（帧层与配置已完成，见 §8.2）

> ⚠️ **2026-09-11 勘误**：本文初版的上下行方向标注反了。上游
> `rm_serial_driver.cpp` 的 `receiveData()` 收 `0x5A`、`sendData()` 发 `0xA5`，
> 即 **`0x5A` 是本机（MCU）发出的遥测，`0xA5` 是上位机下发的目标**。现全文已按
> 上游代码更正；CRC 算法、payload 布局与测试向量本身自始正确，未受影响。

## 1. 总体设计

### 1.1 职责边界

沿用开源 `rm_vision` 栈的算法分工，不重写视觉算法：

| 侧 | 职责 |
| --- | --- |
| 上位机 | 相机取流、装甲板识别、PnP 解算、目标跟踪（EKF）、发布目标状态、弹道解算 |
| 下位机 | 云台角度闭环、限幅与软限位、发射控制、安全门控 |

**下位机不做图像处理，上位机不做云台闭环。** 上位机输出目标的完整运动状态
（位置、速度、自转角速度、多装甲板几何），下位机负责把它变成云台角度目标的
限幅推进。

### 1.2 自瞄模式下的行为边界

自瞄开启后，**只有云台角度目标由视觉接管**，其余全部保持手动遥控不变：

| 子系统 | 自瞄开启后 |
| --- | --- |
| 云台 Yaw / Pitch 角度目标 | **视觉接管**（仅 Yaw 处于角度保持态时） |
| 云台 PID 环路、限幅、软限位、前馈 | 完全复用，不改 |
| 底盘（平移、跟随、小陀螺） | 不变，仍由遥控驱动 |
| 发射（摩擦轮、单发、连发、卡弹恢复） | 不变，仍由 SC/SD 驱动 |
| 安全状态机、任务健康、命令租约 | 完全不变 |
| 摇杆对云台的微调 | **当前不启用**，仅预留字段（见 §3.1 bit3） |

视觉写入的是**云台角度参考**，不是电机电流。执行层限幅、Pitch 软限位
`[-30, +20] deg`、NaN/Inf 失效关闭对视觉来源一视同仁。

**接管的前置条件**（任一不满足即退回手动保持）：

1. `ROBOT_CONTROL_AUTO_AIM`——遥控 SB 拨中位；
2. 链路在线——`250 ms` 内收到过 CRC 有效的目标帧；
3. `TRACKING = 1`——跟踪器已收敛；
4. 全部浮点字段有限——非 NaN / Inf；
5. Yaw 处于角度保持态——速度/制动阶段仍由操作手主导。

### 1.3 坐标系：跟踪器输出必须为云台系

上位机 `armor_tracker` 的 `target_frame` 参数**必须配置为 `gimbal_link`**，不得
使用 `odom`。

理由：

- `gimbal_link → camera_link → camera_optical_frame` 是固定外参，静态 TF 即可，
  由 `robot_state_publisher` 从 URDF 解析，不需要每帧下发 TF。
- 目标状态直接在云台系表达，与下位机的云台控制坐标天然对齐。
- 上游默认在 `odom` 系下输出，需要下位机每帧上报姿态建立 `odom → gimbal_link`
  浮动 TF。那样云台一转参考系就变，跟踪器每帧都要重组，链路耦合重。
- 云台转动由 `gimbal_link` 相对相机的固定外参自动补偿，跟踪器不需要知道云台
  当前指向。

URDF 中现有的 `odom` link 与 `floating` 类型的 `gimbal_joint` 可以删除。

#### 1.3.1 云台系下 `(x, y)` 的正确用法（关键）

`gimbal_link` 随云台转动，因此上位机下发的 `(x, y, z)` 是目标**相对云台当前
指向**的方位，不是世界系绝对坐标。下位机必须按下式使用：

```text
yaw_delta_deg = atan2(y, x) * 180/pi          // 相对当前指向的方位角
yaw_ref      += clamp(yaw_delta_deg, ±单周期限幅)   // 相对增量，不是绝对值
```

**绝不能用 `atan2` 的绝对值直接作为角度参考。** 否则云台转过之后，参考会被拽回
旧方向，形成正反馈导致持续打偏。同理 Pitch：

```text
pitch_delta_deg = -atan2(z, sqrt(x²+y²)) * 180/pi // 当前云台系下的相对增量
```

取负是因为上位机 Z 轴向上为正，而本车 IMU Pitch 抬头为负（见 §5.2）。

### 1.4 串口参数

| 项 | 值 |
| --- | --- |
| 外设 | USART1（**PB7 = RX，PA9 = TX**，AF7） |
| 波特率 | 921600 |
| 数据格式 | 8 位数据 / 无校验 / 1 位停止位（8N1） |
| 流控 | 无 |
| 下位机端点 | `bsp_usart` 端点，延后 dispatch 模式 |

> ⚠️ **接线核对**：本板 USART1 复用了非标准的 PB7/PA9（`platform/boards/rm_f407/
> generated/usart.c`），不是常见的 PA9/PA10。上位机 USB-TTL 的 **TX 必须接 PB7**。
> 该外设与 DMA/中断已在 CubeMX 侧配置完成，无需改动。
>
> 上位机需有串口权限：`sudo usermod -aG dialout $USER`（重新登录生效），
> 或临时 `sudo chmod 666 /dev/ttyUSB0`。

**与裁判系统的互斥**：`REFEREE_UART` 当前也指向 `huart1`。两者由编译期开关
`VISION_ENABLE` 与 `REFEREE_ENABLE` 互斥，同时开启必须 `#error`。见 §6。

带宽占用：遥测 22 B @ 50 Hz ≈ 1.1 kB/s，目标 52 B @ 120 Hz ≈ 6.2 kB/s，
相对 921600 8N1 的约 92 kB/s 容量占用不足 8%，余量充足。

## 2. 帧格式

### 2.1 通用帧

固定长度帧，两侧共用：

```text
偏移  长度  字段      说明
   0     1  header    方向标识与帧起始
   1     1  len       payload 字节数
   2   len  payload   帧体
 2+len    2  crc16     校验，小端（低字节在前）
```

- **`header` 按方向区分**，同时也是两侧各自的帧同步字节（与上游一致）：

  | 方向 | 帧 | header | len | 整帧 |
  | --- | --- | --- | ---: | ---: |
  | MCU → 上位机 | 遥测 telemetry | `0x5A` | 18 | 22 |
  | 上位机 → MCU | 目标 target | `0xA5` | 48 | 52 |

  任一侧收到自己的 header 即可直接丢弃——说明是回声、接错线或帧错位。
  下位机在流式解码中只认 `0xA5`，因此把本机发出的遥测回读进来不会产生误帧。

- **`len`** 为 payload 字节数，用于交叉校验。收到 `len` 与预期不符的帧直接丢弃。
- **`crc16`** 覆盖 `header + len + payload`（即整帧除校验字段外全部字节），
  小端存放。

### 2.2 CRC 算法

**CRC-16/MCRF4XX**：多项式 `0x8408`（反射形式，对应 `0x1021`），初值 `0xFFFF`，
不取反，不异或输出，反射输入与输出。

| 项 | 值 |
| --- | --- |
| 多项式 | `0x8408` 反射 |
| 初值 | `0xFFFF` |
| 输入反射 | 是 |
| 输出反射 | 是 |
| 结果异或 | `0x0000` |
| 校验值 | `CRC("123456789") = 0x6F91` |

**两侧实现本来就是同一算法的两份拷贝，已验证一致，无需修改任何现有 CRC 代码：**

| 侧 | 实现 | 状态 |
| --- | --- | --- |
| 上位机 | `rm_serial_driver/src/crc.cpp`，表首项 `0x1189` | 保留 |
| 下位机 | `components/algorithms/checksum/crc16.c`，`CRC_POLY_16 = 0xA001` | 保留 |
| 下位机（视觉） | `components/devices/vision/vision_protocol.c` 自建表 | 表首项同为 `0x1189` |

下位机 `crc16.c` 的表生成代码对 `0xA001` 做右移查表，等价于多项式 `0x8408`，
生成表首项同为 `0x1189`；`crc_16()` 的驱动式与上位机 `Get_CRC16_Check_Sum()`
逐字相同，初值均为 `0xFFFF`。

> 注意：`crc16.c` 中同时存在 `crc_16()`（初值 `CRC_START_16 = 0xFFFF`）与
> `crc_modbus()`（初值 `CRC_START_MODBUS = 0xFFFF`）两个函数。本协议**只使用
> `crc_16()`**。两者初值巧合相同，当前行为一致，但语义不同，不要混用。

### 2.3 字节序

所有多字节字段（`uint16` / `float`）一律**小端**。STM32F4 为小端架构，
上位机 x86_64 亦为小端，两侧均可直接结构体映射。

### 2.4 校验测试向量

实现后必须用以下向量自检，逐字节一致。

**遥测帧（MCU → 上位机，header `0x5A`，全零 payload）**

```text
5A 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 E5 C8
```

**遥测帧（flags=0x0005, seq=1234, bullet=15, yaw=45.5, pitch=-12.25, roll=1.5）**

```text
5A 12 05 00 D2 04 0F 00 00 00 36 42 00 00 44 C1 00 00 C0 3F 62 C1
```

**目标帧（上位机 → MCU，header `0xA5`，flags=0x47, seq=5678, x=1.0, y=2.0,
z=0.5, yaw=0.3, vx=0.1, vy=-0.2, vz=0.0, v_yaw=1.2, r1=0.25, r2=0.25, dz=0.0）**

```text
A5 30 47 00 2E 16 00 00 80 3F 00 00 00 40 00 00 00 3F 9A 99 99 3E
CD CC CC 3D CD CC 4C BE 00 00 00 00 9A 99 99 3F 00 00 80 3E 00 00
80 3E 00 00 00 00 A3 8B
```

三个向量均已由下位机实现与独立脚本双向验证一致。

## 3. 遥测帧：MCU → 上位机

对应上游 `ReceivePacket`（`header = 0x5A`）。`len = 18`，整帧 22 字节。

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | `u8` | `header` | `0x5A` |
| 1 | `u8` | `len` | `18` |
| 2 | `u16` | `flags` | 见 §3.1 |
| 4 | `u16` | `seq` | 帧序号，每帧 +1，回绕后从 0 继续 |
| 6 | `u8` | `bullet_speed` | 弹速，单位 m/s，整数 |
| 7 | `u8` | `reserved` | 保留，**必须写 0** |
| 8 | `f32` | `yaw` | 云台 IMU 累计 Yaw，deg |
| 12 | `f32` | `pitch` | 云台 IMU Pitch，deg（REP-103，抬头为正） |
| 16 | `f32` | `roll` | 云台 IMU Roll，deg |
| 20 | `u16` | `crc16` | 小端 |

### 3.1 `flags` 位定义

沿用上游 `ReceivePacket` 的位序，使上位机只改结构体布局、不改位定义：

| 位 | 名称 | 说明 |
| ---: | --- | --- |
| 0 | `ENEMY_COLOR` | 0 = 红方，1 = 蓝方。上位机据此选择识别颜色 |
| 1 | `RESET_TRACKER` | 1 = 请求上位机清空跟踪器状态 |
| 2 | `AUTO_AIM_ENABLE` | 1 = 自瞄模式已开启，上位机应更新并下发目标 |
| 3 | `MANUAL_TRIM_ENABLE` | 预留：操作手摇杆微调是否叠加到视觉角度目标 |
| 4 | `FIRE_MODE_AUTO` | 预留：请求上位机允许自动开火判定 |
| 5–15 | — | 保留，**必须写 0** |

> `AUTO_AIM_ENABLE` 是**安全关键位**。下位机在自瞄未开启时置 0，并且**同时
> 在接收侧硬性拒收目标帧**——不等上位机配合，避免上位机异常时干扰手动云台。
> 上位机应尊重该位：收到 0 后停止下发有效目标。

`RESET_TRACKER` 是**一次性脉冲**：只在下一帧成功入队的遥测帧中置位，随后自动清除；
串口忙或姿态非法而未发出的帧不会消费该请求，避免以 50 Hz 反复复位或因一次发送
失败丢失复位请求。

### 3.2 姿态字段的必要性

即使跟踪器工作在云台系，姿态仍然必须每帧上报：

- `yaw` 给出云台绝对指向，用于上位机可视化与调试（RViz 中重建云台朝向）；
- `pitch` 供上位机判断目标是否已超出机械可达范围，可提前放弃；
- `seq` 与姿态时间戳一起，让上位机可以估计链路延迟。

上位机侧 `receiveData()` 现有行为保持不变：用 `roll/pitch/yaw` 发布
`odom → gimbal_link` 的 TF，并据 `detect_color` / `reset_tracker` 调整识别参数。
由于 §1.3 要求跟踪器工作在 `gimbal_link` 系，这些 TF 现在仅用于可视化。

### 3.3 弹速

`bullet_speed` 为整数 m/s，由下位机从 `ROBOT_BULLET_SPEED_MPS` 推导（当前摩擦轮
目标对应约 15 m/s 量级，**需实测标定**）。上位机用它做弹道预补偿的可视化。

### 3.4 敌方颜色来源

调试阶段 `ROBOT_ENEMY_COLOR` 为常量 0（红方）。裁判系统启用后应改为从比赛信息
读取；该改动只涉及 `robot_app.c` 中的一次赋值，不影响协议。

## 4. 目标帧：上位机 → MCU

对应上游 `SendPacket`（`header = 0xA5`）。`len = 48`，整帧 52 字节。

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | `u8` | `header` | `0xA5` |
| 1 | `u8` | `len` | `48` |
| 2 | `u16` | `flags` | 见 §4.1 |
| 4 | `u16` | `seq` | 帧序号 |
| 6 | `f32` | `x` | 目标中心位置 X，m，云台系 |
| 10 | `f32` | `y` | 目标中心位置 Y，m，云台系 |
| 14 | `f32` | `z` | 目标中心位置 Z，m，云台系 |
| 18 | `f32` | `yaw` | 目标朝向，rad，云台系 |
| 22 | `f32` | `vx` | 目标中心速度 X，m/s，云台系 |
| 26 | `f32` | `vy` | 目标中心速度 Y，m/s，云台系 |
| 30 | `f32` | `vz` | 目标中心速度 Z，m/s，云台系 |
| 34 | `f32` | `v_yaw` | 目标自转角速度，rad/s |
| 38 | `f32` | `r1` | 半径 1，m |
| 42 | `f32` | `r2` | 半径 2，m |
| 46 | `f32` | `dz` | 上下装甲板高度差，m |
| 50 | `u16` | `crc16` | 小端 |

字段语义直接对应 `auto_aim_interfaces/msg/Target`，上位机侧只做搬运，不做变换。

> 上游 `SendPacket` 的位域顺序为 `tracking(1) | id(3) | armors_num(3) |
> reserved(1)`，与下表一致。`seq` 是本次新增字段（上游无），插入在 `flags`
> 之后；上位机需同步更新结构体。

### 4.1 `flags` 位定义

| 位 | 名称 | 说明 |
| ---: | --- | --- |
| 0 | `TRACKING` | 1 = 跟踪器已收敛，本帧目标有效 |
| 1–3 | `TARGET_ID` | 目标编号，见 §4.2 |
| 4–6 | `ARMORS_NUM` | 装甲板数量，见 §4.3 |
| 7 | `FIRE_ADVICE` | 预留：上位机判定当前可命中 |
| 8–15 | — | 保留，**必须写 0** |

### 4.2 `TARGET_ID`

沿用开源 tracker 的编号，与 `rm_serial_driver.cpp` 的映射表一致：

| 值 | 含义 |
| ---: | --- |
| 0 | 前哨站 outpost |
| 1–5 | 敌方 1–5 号（英雄 / 工程 / 步兵） |
| 6 | 哨兵 guard |
| 7 | 基地 base |

### 4.3 `ARMORS_NUM`

| 值 | 含义 |
| ---: | --- |
| 2 | 平衡步兵（双腿各一块） |
| 3 | 前哨站（三块） |
| 4 | 常规（四块） |

- `TRACKING = 0` 时，`flags` 的其余位与全部浮点字段**无意义，应写 0**，
  但帧仍需发出，用于维持链路心跳与序号连续性。

### 4.4 双半径与高度差

仅 `ARMORS_NUM = 4` 时有效。常规机器人四块装甲板构成两层，`r1`/`r2` 为两层
各自的半径，`dz` 为两层的高度差。`ARMORS_NUM = 2` 或 `3` 时只有 `r1` 有效，
`r2 = r1`，`dz = 0`。

上位机侧 `tracker.cpp` 已正确处理：`another_r` 在非 4 装甲板时等于 `r`，
`dz` 在非 4 装甲板时保持 0。

### 4.5 发送频率

上位机在 `armor_tracker` 每次输出 `/tracker/target` 时触发发送，**频率跟随相机
帧率**（典型 60–120 Hz），非严格定频。下位机按 §7.1 的超时策略处理抖动，
并在 20 ms 控制周期内对参考做限幅推进，因此高于 50 Hz 的目标帧不会导致过冲。

## 5. 坐标与方向约定

本节为**不可协商项**。任一侧违反都会导致自瞄打反方向。

### 5.1 云台系定义

| 轴 | 方向 |
| --- | --- |
| X | 云台正前方（与 IMU Yaw 零位对齐） |
| Y | 云台左方 |
| Z | 向上 |

遵循 ROS REP-103 右手系。

| 量 | 正方向 | 单位 |
| --- | --- | --- |
| `yaw` | 绕 Z 轴俯视**逆时针** | rad（目标）/ deg（遥测） |
| `pitch` | 绕 Y 轴**抬头** | deg |
| `roll` | 绕 X 轴 | deg |

目标帧的 `(x, y, z)` 处于**随云台转动的** `gimbal_link` 系，使用方式见 §1.3.1。

### 5.2 Pitch 符号（已实板确认）

本车 **IMU Pitch 抬头为负**，与 ROS REP-103（抬头为正）相反。已于 2026-09-11 实板
抬手确认，`robot_config.h` 的 `GIMBAL_PITCH_MIN_DEG = -30.0f`（抬头）注释正确。

内部符号链**自洽**，不需要改动：

- `gimbal.c` 的 `pitch_hold_ref` 与 `attitude.Pitch` 同向，可直接比较；
- `GimbalPitchLimit_ClampAngleReference()` 的 `[-30, +20]` 即"抬头 30 / 低头 20"；
- `gimbal_units.h` 的重力前馈等内部计算一致。

**因此符号转换只允许出现在协议边界**，内部一律保持本车 IMU 符号：

| 方向 | 转换 | 实现 |
| --- | --- | --- |
| 遥测上报姿态 | `wire = -imu` | `RmVision_PitchImuToWire()` |
| 目标仰角解算 | `target_delta_imu = -atan2(z, sqrt(x²+y²))` | `Gimbal_TargetPitchReferenceDeg()` |

`yaw` 与 `roll` 不取反。遥测侧的转换函数集中在
`components/devices/vision/vision_protocol.c`，**禁止在其它位置再次取反**。

### 5.3 有限值防护

固件 Release 构建使用 `-O3`，不启用 `-ffinite-math-only`。
控制安全路径仍不把编译器浮点假设当作安全门：协议数据、云台解算、
Pitch 限位和轴状态均使用 IEEE-754 位模式检查。

| 编译标志 | `isfinite(NaN)` | `isfinite(Inf)` |
| --- | ---: | ---: |
| `-Og`（Debug） | 0（正确拒绝） | 0（正确拒绝） |
| `-O3`（Release） | 0（正确拒绝） | 0（正确拒绝） |

视觉数据是最可能产生 NaN/Inf 的来源，因此视觉路径上**所有**有限值检查都用
IEEE-754 位模式（指数域全 1 即非有限），纯整数运算不受快速数学影响：

| 位置 | 实现 |
| --- | --- |
| 协议层 | `vision_protocol.c` 的 `vision_float_is_finite()` |
| 云台解算 | `gimbal_units.h` 的 `Gimbal_FloatIsFinite()` / `Gimbal_NaNF()` |

历史上的 `-Ofast` 构建曾使 `isfinite()` 防护失效；当前工具链已切换到
`-O3`，云台相关实现也直接使用 `Gimbal_FloatIsFinite()`，避免该问题回归。

## 6. 编译期配置

已在 `applications/robot/config/robot_config.h` 中新增：

```c
#ifndef VISION_ENABLE
#define VISION_ENABLE                   1U      /* [核心-联锁] 视觉通信总开关 */
#endif
#define VISION_UART_PORT                1U      /* [固定] 当前使用 USART1 */
#define VISION_BAUDRATE                 921600U /* [固定] 与上位机一致，8N1 */
#define VISION_ONLINE_TIMEOUT_MS        250U    /* [保护] 无有效目标帧则视觉离线 */
#define VISION_STARTUP_GRACE_MS         3000U   /* [保护] 启动宽限 */
#define VISION_STATUS_LOG_PERIOD_MS     1000U   /* [核心-调试] 自瞄状态打印周期 */
```

互斥与冲突检查（全部为编译期 `#error`）：

```c
#if VISION_ENABLE && REFEREE_ENABLE
#error "Vision link and referee share USART1; enable only one"
#endif
```

另检查与调试串口、调参遥测、遥控接收机的端口冲突。

### 6.1 与裁判系统的共存策略

`REFEREE_UART` 与 `VISION_UART` 当前都指向 `huart1`。处理原则：**完全不动现有
裁判系统接口**。

- `REFEREE_ENABLE` 保持 0，裁判链路现状不变；
- `components/devices/referee/` 下任何文件都不修改；
- 新增的互斥检查在两者同时启用时直接编译失败（已实测触发），避免踩踏；
- 后续若需同时使用，再评估硬件串口分配（USART2 / UART4 / UART5 尚未分配），
  届时把 `VISION_UART_PORT` 改宏即可，协议层不受影响。

### 6.2 关闭视觉时的开销

`VISION_ENABLE = 0` 时整段接管逻辑被预处理掉。已实测：

`arm-none-eabi-size` 实测（2026-09-11 配置快照，bss + data 记作 RAM）：

| 构建 | RAM | Flash |
| --- | ---: | ---: |
| 未接入视觉前（`HEAD`，基线） | 54,744 B | 95,760 B |
| `VISION_ENABLE = 0` | 54,744 B | 95,792 B |
| `VISION_ENABLE = 1` | 55,544 B | 99,164 B |

结论：`VISION_ENABLE = 0` 时 **RAM 与未接入视觉前逐字节相同**（54,744 B），
Flash 只多 32 B（`robot_config.h` 里的联锁 `#error` 与常量不产生代码，
差值来自视觉关闭分支上 `gimbal.c` 中残留的一次比较），可视为零成本。

`VISION_ENABLE = 0` 时 `nm` 中 `RmVision*` 符号数为 **0**，整个视觉模块被
`--gc-sections` 完整回收；`= 1` 时为 8 个。

接入视觉的净成本：**RAM +800 B，Flash +3,404 B**。

该表是关闭调试文本、启用历史配置时的对比快照；当前默认联调配置同时开启文本日志，
因此实际镜像大小以 `app.elf` 的链接器报告为准（2026-10-02 为 RAM 55,272 B、Flash
109,392 B）。

## 7. 失效与降级策略

### 7.1 超时判定

下位机在 `VISION_ONLINE_TIMEOUT_MS = 250 ms` 内未收到任何 CRC 有效的目标帧，
判定视觉离线。复用现有 `daemon` 设备健康服务，与 DJI 电机、遥控的掉线判定
机制一致，并带 `VISION_STARTUP_GRACE_MS = 3000 ms` 启动宽限。

**目标帧率可达 120 Hz 且非定频，因此超时值刻意取得比遥控链路宽松**
（遥控为 100 ms）：宁可晚一点退出自瞄，也不要因相机偶发丢帧导致云台在
视觉与手动之间反复切换。

### 7.2 降级行为

**视觉离线不等于整车停机。** 这是与现有安全策略的关键差异，必须明确：

| 条件 | 云台行为 | 底盘 / 发射 / 安全 |
| --- | --- | --- |
| 自瞄未开启（SB 非中位） | 手动控制 | 不变 |
| 自瞄开启，视觉在线，`TRACKING = 1` | 视觉角度目标 | 不变 |
| 自瞄开启，视觉在线，`TRACKING = 0` | 保持当前角度 | 不变 |
| 自瞄开启，视觉离线（超时） | **退回手动控制** | 不变 |
| 遥控离线 / SA 下位 / 关键任务故障 | 停机 | 停机（现有策略） |

自瞄失效**只影响云台**。底盘、发射、安全门完全按现有逻辑运行。
`RmVisionLink` 的离线回调**不重启串口接收**，与裁判/遥控的离线回调行为不同。

### 7.3 角度目标合法性

下位机在把视觉角度写入云台参考前必须校验（均已实现）：

- 浮点为有限值（非 NaN / Inf），使用位模式检查（§5.3）；
- 自瞄未开启时直接拒收，不依赖上位机配合（§3.1）；
- 视觉角度参考**按相对增量推进**，不是绝对赋值（§1.3.1）；
- 单周期推进量限幅 `VISION_MAX_RATE_DEG_S` / `VISION_MAX_RATE_PITCH_DEG_S`，
  并进一步夹到手动满摇杆速度以内；
- Pitch 经 `GimbalPitchLimit_ClampAngleReference()` 夹到 `[-30, +20] deg`。

### 7.4 帧序号

`seq` 用于诊断丢帧与乱序。下位机在链路统计中暴露。**乱序帧不直接丢弃**，
控制始终取最新到达的有效帧；由于参考按相对增量推进，偶发乱序只会造成一次
小的推进误差，不会产生位置阶跃。

## 8. 兼容性说明

### 8.1 与现有 `serial_frame_protocol` 的关系

`components/devices/vision/serial_frame_protocol.c` 现有实现为裁判系统风格的
变长双 CRC 帧（`0xA5` SOF + 16 位长度 + CRC8 帧头 + cmd_id + 数据 + CRC16 帧尾），
与本协议的定长单 CRC 帧**不兼容**。

处理方式：旧文件保留给历史 demo，并放进 `rm_components_compat`，不进入正式
`rm_components_runtime`。`master_process.c` 只补充了注册失败时的 UART/DMA 回滚；
正式视觉链路使用新增的独立帧层实现，`ins_task` 不再调用旧接口。这样兼容 demo
仍可显式使用旧协议，正式 `app.elf` 不会把两套视觉收发路径都接入运行时。

`rm_components_compat` 为兼容 target 传递链接 `rm_components_runtime`，因此构建兼容
库时可以同时看到新旧源文件；这不改变正式 target 的边界，也不允许一个应用同时
初始化两套视觉链路。

现有 `master_process.c` 的 `VisionInit` / `VisionSend` 目前**无任何调用者**，
`VISION_RECV_SIZE` / `VISION_SEND_SIZE` 与实际结构体尺寸也不一致
（18 / 36 对 12 / 36）。本次实现不依赖这两个宏。

### 8.2 上位机改动范围（✅ 已完成 2026-09-11）

改动限于 `rm_serial_driver` 与配置/URDF，**不动任何视觉算法代码**。
远端仓库为上游 gitee 源码，全部改动为未提交的工作区修改。

| 文件 | 改动 | 状态 |
| --- | --- | --- |
| `rm_serial_driver/include/rm_serial_driver/packet.hpp` | 按 §3/§4 重排两个结构体，加 `length`/`seq`/`bullet_speed`/`reserved`，加位域访问器与 `static_assert` 布局断言 | ✅ |
| `rm_serial_driver/src/rm_serial_driver.cpp` | `receiveData()` 改为按帧头 + `length` 字段分段读帧，CRC 失败逐字节重同步，删除已不存在的 `aim_xyz` 引用；`sendData()` 填充 `seq`，位域改用 `set_*()`，降低日志等级 | ✅ |
| `rm_serial_driver/include/rm_serial_driver/rm_serial_driver.hpp` | 增加 `send_seq_` / `last_receive_seq_` / `have_received_packet_` / `crc_errors_` / `dropped_packets_` / `auto_aim_was_enabled_` | ✅ |
| `rm_serial_driver/config/serial_driver.yaml` | `baud_rate: 921600`，注释 udev 别名建议 | ✅ |
| `rm_gimbal_description/urdf/rm_gimbal.urdf.xacro` | 删除 `odom` link 与 `floating` 的 `gimbal_joint`，`gimbal_link` 成为根 | ✅ |
| `rm_vision_bringup/config/node_params.yaml` | `target_frame: gimbal_link`；`serial_driver` 段同步 921600 | ✅ |
| `rm_vision_bringup/config/launch_params.yaml` | `odom2camera` 改名 `gimbal2camera`（父坐标系不再是 odom） | ✅ |
| `rm_vision_bringup/launch/common.py` | 同步改名引用 | ✅ |
| `rm_serial_driver/include/rm_serial_driver/crc.hpp` / `src/crc.cpp` | **未改**，算法已验证一致 | ✅ |

`armor_detector`、`armor_tracker`、`auto_aim_interfaces`、`ros2_hik_camera`
**完全未改**。

#### 8.2.1 关键实现约束

- **`serial_driver` 的 `receive()` 没有偏移/长度重载**（只有 `receive(std::vector<uint8_t>&)`，
  读取恰好 `buff.size()` 个字节）。因此帧分三段读：帧头(1) → `length`(1) → 帧体(20)。
  不要写成 `receive(data, 1, len)` —— 该重载不存在。
- **不能用 `sizeof(ReceivePacket)` 当帧长**。必须先读 `length` 字段校验，否则两侧结构体
  一旦不同步，会静默错位拼接，比直接报错难查得多。
- **CRC 失败必须逐字节重同步**：只丢帧头这一个字节，让下一个 `0x5A` 重新成为候选起点。
  丢弃整帧会让恰好出现在载荷内的伪帧头连带吞掉紧随的真实帧。这与下位机
  `RmVisionRx_Push()` 的策略一致。
- **`aiming_point_` 保留但无数据源**：本协议遥测帧不带瞄准点（弹道解算在下位机内部）。
  上游按 `aim_x/aim_y/aim_z` 发布 Marker 的代码已删除，Marker 初始化保留供日后扩展。
- 上游源码有两处**重复/无用代码**已顺带清理：`sendData()` 里 id 映射表中
  `{"1", 1}` 重复出现两次（`std::map` 初始化列表静默忽略后者）；
  `ReceivePacket` 的 `aim_x/aim_y/aim_z` 三个字段本协议不存在，相关 Marker 发布
  代码已删除。

#### 8.2.2 设备名

`device_name` 当前为 `/dev/ttyUSB0`。裸设备名在插拔或多设备时可能漂移，**建议用 udev
规则改出稳定别名**：

```bash
# /etc/udev/rules.d/99-rm-vision.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{serial}=="XXXX", SYMLINK+="rm_vision"
```

然后 `device_name: /dev/rm_vision`。上位机用户还需在 `dialout` 组内
（`sudo usermod -aG dialout $USER`，重新登录生效）。

### 8.3 URDF 改动（✅ 已完成）

已按 §8.3 原计划删除 `odom` link 与 `floating` 的 `gimbal_joint`，`gimbal_link`
成为 URDF 根，`camera_joint` 保留并接受 launch 传入的实测外参。

`odom → gimbal_link` 仍由 `rm_serial_driver` 的 `receiveData()` 发布，
但**只用于 RViz 中重建云台朝向**，跟踪器不再依赖它。

> ⚠️ `camera_joint` 的 `xyz` / `rpy` 当前仍是上游默认值
> （`0.08 0 0.065` / `0 0 3.14159265`，见 `launch_params.yaml`）。
> **该外参直接决定自瞄精度，必须实车标定后才能上机。**

## 9. 实现状态

### 9.1 已完成（截至 2026-10-02）

| 项 | 位置 |
| --- | --- |
| 帧编解码与 CRC | `components/devices/vision/vision_protocol.c/.h` |
| 流式解码（分片/粘包/重同步/方向隔离） | 同上，`RmVisionRx_Push()` |
| 串口端点、守护、目标缓存 | `components/devices/vision/vision_link.c/.h` |
| 在线/目标可用性状态机 | `components/devices/vision/vision_link_state.c/.h` |
| 视觉初始化失败回滚 | `platform/stm32f4/usart/bsp_usart.c` 的 `USARTUnregister()`；DMA 无法停止时保留端点并让链路保持静默 |
| 云台自瞄接管与降级 | `applications/robot/gimbal/gimbal.c` |
| 方位/仰角解算（可测） | `applications/robot/gimbal/gimbal_units.h` |
| 编译期配置与互斥检查 | `applications/robot/config/robot_config.h` |
| 应用层接线 | `applications/robot/robot_app.c` |
| 主机测试（协议，114 项断言） | `tests/host/vision_protocol_test.c` |
| 主机测试（自瞄解算，46 项断言） | `tests/host/vision_aim_test.c` |
| 主机测试（在线/目标状态边沿） | `tests/host/vision_link_state_test.c` |
| 构建接入 | `components/CMakeLists.txt`（`rm_components_runtime`） |
| 主机测试注册 | `tests/host/CMakeLists.txt` |

已验证：

- 主机测试已注册，协议、解算和在线状态边沿 target 在本轮主机构建中编译通过；本轮
  未执行 `ctest`，因此新增状态边沿测试的运行结果仍待单独运行确认。历史构建在
  `-O0` / `-Og` / `-O3` / `-Ofast` 变体下曾通过历史测试；
- `app.elf` 构建通过，两项链接审计（堆/stdio/C++ 符号、ELF 段权限）通过；
- `rm_components_runtime`、`rm_components_compat` 和 `test_infantry_minimal` 的入站
  分发边界完成编译检查；FreeRTOS 与裸机对照入口均在注册设备前选择延后 dispatch，
  并在电机阶段先派发 inbox；
- `VISION_ENABLE=0` 时 RAM 回到 54,744 B 基线，`RmVision*` 符号数为 0，零开销；
- `VISION_ENABLE=1 && REFEREE_ENABLE=1` 触发预期 `#error`；
- 三个测试向量与独立脚本逐字节一致；
- 相对方位迭代收敛测试（`test_no_positive_feedback`）证明 §1.3.1 的增量用法
  会收敛到目标且收敛后是稳定不动点。

**尚未验证**：实板收发、真实相机帧率下的链路稳定性、相机外参标定、
云台实际跟随效果。`gimbal.c` 的接管逻辑目前只经过编译、代码审查与
`gimbal_units.h` 层的单元测试，**未在实板闭环验证**。

### 9.2 实现过程中被测试抓到的问题

均已在提交前修复，记录在此备查：

| 问题 | 发现方式 | 修复 |
| --- | --- | --- |
| 上下行方向标注反了（`0x5A`/`0xA5` 互换） | 对照上游 `rm_serial_driver.cpp` | 按上游更正，`0x5A` = MCU 遥测、`0xA5` = 上位机目标 |
| 目标 `z` 是米却被当成角度用 | 写测试时核对字段单位 | 改为 `atan2(z, sqrt(x²+y²))` 换算仰角，并加水平距离下限 |
| 用 `WrapAngleDeg180` 推进 Yaw 参考 | 核对 `attitude.YawTotalAngle` 语义 | 改为多圈累加 `Gimbal_AdvanceAngleRefDeg()`，加回归测试 |
| 新增的 `Gimbal_TargetYawReferenceDeg` 继承了 `isfinite()` 缺陷 | `-Ofast` 下 `Inf y -> hold` 断言失败 | 改用位模式 `Gimbal_FloatIsFinite()` |
| 自瞄开关存在竞态：控制任务刚关闭自瞄，接收回调仍写入可用目标 | 代码审查 | 开关判定与目标写入合入同一临界区 |
| CRC 失败时丢弃整帧会吞掉紧随其后的真帧起始 | `test_stream_garbage_resync` 失败 | 改为丢弃 1 字节逐字节重同步 |
| `IsOnline()` 被目标有效性绑死，手动模式无法反映对端在线 | 状态审计 + 边沿测试 | 拆分在线/目标状态机，合法帧仍喂守护 |
| 视觉 daemon 注册失败会遗留已启动的 UART DMA 端点 | 初始化路径审计 | 增加 `USARTUnregister()` 启动回滚 |
| 新旧视觉协议同时进入正式组件 target | CMake target 审计 | 旧路径移入 `rm_components_compat`，INS 解除旧接口耦合 |
| 遥测发送在 INS 快照异常时可能把 NaN/Inf 发给上位机 | 二次链路审计 | `RmVisionLink_Update()` 采用位模式有限值检查；异常帧丢弃并保留 RESET 脉冲 |

### 9.3 上位机侧已完成（2026-09-11）

| 项 | 状态 |
| --- | --- |
| `packet.hpp` 重排 + 布局断言 | ✅ 22 / 52 字节，偏移与 §3/§4 逐字节一致 |
| `rm_serial_driver.cpp` 帧层重写 | ✅ 分段读帧、length 校验、逐字节重同步 |
| `sendData()` 填充 `seq`、位域访问器 | ✅ |
| `serial_driver.yaml` 921600 | ✅ |
| URDF 删除 `odom` / floating joint | ✅ `gimbal_link` 成为根 |
| `node_params.yaml` `target_frame: gimbal_link` | ✅ |
| `launch_params.yaml` `odom2camera` → `gimbal2camera` | ✅ |
| **两侧测试向量对拍** | ✅ 见下 |

**对拍结果**（下位机 `tests/host/vision_protocol_test.c` ↔ 上位机 `vector_check.cpp`）：

| 向量 | 结果 |
| --- | --- |
| 遥测全零帧 | ✅ 22 字节逐字节一致 |
| 遥测样本帧 | ✅ 22 字节逐字节一致 |
| 目标样本帧 | ✅ 52 字节逐字节一致 |
| 反解字段（flags/seq/角度/xyz 位域） | ✅ 全部一致 |
| 坏 CRC 拒绝 | ✅ 两侧均拒绝 |

上位机 `colcon build` 通过（`-Wall -Werror`），`xacro` 解析通过，
`robot_state_publisher` 发布静态 TF 后
`gimbal_link → camera_optical_frame` 可解析为
平移 `[0.100, 0, 0.050]`、RPY `[-90°, 0, -90°]`，与 URDF 一致。

### 9.4 剩余验收与待实现

- [x] 视觉链路状态已对外暴露（`LogVisionStatus()`，见 §9.5.1）；
- [x] 正式 FreeRTOS 与 `test_infantry_minimal` 均在注册端点前选择延后 CAN/UART dispatch，
      并在电机阶段派发入站事件；
- [ ] 诊断/调参遥测的 64 通道里加一路数值化视觉状态（当前只有文本日志）
- [ ] `camera_joint` 外参实车标定（**上机前必须完成**）
- [ ] 弹速 `ROBOT_BULLET_SPEED_MPS` 实测标定
- [ ] 实板联调：收发、延迟、降级行为、自瞄跟随效果

### 9.5 诊断接口

`RmVisionLink_GetStats()` 返回以下计数，可直接接入现有调参遥测
（`applications/robot/diagnostics/tuning_telemetry.c` 的 64 通道上限）：

| 字段 | 含义 |
| --- | --- |
| `rx_frames` | 通过 CRC 与长度校验的目标帧数 |
| `crc_errors` | CRC 失败次数 |
| `format_errors` | 帧头对但长度不符的次数 |
| `resync_bytes` | 重同步丢弃的字节数 |
| `rejected_not_armed` | 自瞄未开启时丢弃的目标帧数 |
| `rejected_nonfinite` | 含 NaN/Inf 被丢弃的目标帧数 |
| `tx_frames` / `tx_dropped` | 遥测发出 / 串口忙或姿态非有限而丢弃帧数 |
| `target_frames` | 判定为可用的目标帧数 |
| `last_target_id` / `last_armors_num` | 最近目标编号与装甲板数量 |

### 9.5.1 目前的接法：自瞄状态文本日志（2026-09-11）

`RmVisionLink_GetStats()` 的唯一调用者是 `gimbal.c` 里的 `LogVisionStatus()`，
在自瞄模式下按 `VISION_STATUS_LOG_PERIOD_MS`（默认 1 s）打印一行：

```
aim ACTIVE online=1 rx=62 crc=0 fmt=0 tgt=60 rej=0 id=6 n=4
```

**各计数打印的是最近一个统计周期内的增量，不是累计值**——累计值要靠人工做差
才知道当前还在不在收帧，而这恰恰是实板联调时最常看的一条。非自瞄模式下不打印，
但基线仍跟随更新，避免刚切进自瞄时把历史计数当成"这一秒收到的帧"打出来。

排查表：

| 现象 | 结论 |
| --- | --- |
| `rx=0` | 一个字节都没收到：接线（注意 RX 在 PB7）/ 波特率 / 上位机没在发 |
| `crc>0` 或 `fmt>0` | 收到了但帧不同步：两侧协议版本或长度字段不一致 |
| `tgt>0` 但状态是 `idle` | 目标帧正常，是 `TRACKING=0`、链路判离线或云台不在角度保持态 |
| `rej>0` | 目标被判为非有限值（NaN/Inf），算法侧发散了 |

未接入 VOFA 通道的理由：`tuning_telemetry.c` 已有 64 通道上限，而现有
`GetGimbalAxisChannels` 一路就占 21 通道；再加数值通道需要先重构通道分配，
而文本日志在实板联调阶段的信息量更大、改动面更小。若要长期监控再补通道。

> 注意：在接入 `LogVisionStatus()` 之前，`RmVisionLink_GetStats()` **没有调用者**，
> 已被 `-Wl,--gc-sections` 整体回收（`nm` 查不到符号）。这是本次加日志时发现的
> ——即"接口留了但没接"，功能正常，代价是没有任何可观测性。

### 9.6 桌面闭环验证桩（2026-09-11）

`tools/vision_serial_stub/vision_stub.py` 在没有相机、没有装甲板识别的前提下
闭环验证**基本跟随**。它替代上位机视觉栈：收下位机遥测（姿态），在虚拟世界系里
放一个目标，把目标相对云台当前指向的方位按协议发回去。下位机无法区分它与真实视觉。

目的：把问题**分层定位**。若桩能让云台稳定追上并停住，说明协议层 + 解算 + 云台
控制链路全通，剩余问题只在识别/跟踪算法；反之问题一定在下位机侧。

已验证（主机侧）：

- Python 的 CRC 与 C 实现独立对拍，`CRC16/MCRF4XX("123456789") = 0x6F91`，表首项 `0x1189`；
- C 编码 22B 遥测 → Python 解码，6 个字段逐一相符；Python 编码 52B 目标 → C
  `RmVision_DecodeTarget` 解码，flags/位置/编号全部相符；
- 双向篡改 1 字节均被 CRC 拒绝；
- 伪终端（raw 模式）端到端跑通：50 Hz 收发，CRC 零失败，目标帧位置/高度/相对方位正确。

**尚未做**：真实串口 + 实板联调。

### 9.7 Yaw 跟随方向未验证（上机第一件事）

`yaw_hold_ref += atan2f(target_y, target_x)` 的方向取反量**从未被验证过**：

- `docs/remote_control/*.md` 里"Yaw 摇杆方向正确"至今是未勾选的验收项；
- 本车 IMU yaw 的正方向（俯视逆/顺时针）在仓库里没有明确记录；
- `vision_aim_test.c` 的收敛测试是**符号无关**的——它把
  `Gimbal_TargetYawReferenceDeg(yaw_ref, x, y)` 当作"给定世界系方位差 → 新方位参考"
  的映射，无论这个映射是 `ref+θ` 还是 `ref-θ` 都会收敛。它能抓住正反馈，
  但抓不住整体取反。

因此上机时若不接管云台稳定向单一方向转到底，把 `robot_config.h` 的
`VISION_YAW_FOLLOW_ENABLE` 改为 `0U` 即可，**不需要改动解算与主机测试**。
桩的 `--flip-every-s 10` 可用来直接观察云台往哪边转。

### 9.8 二次审计记录（2026-10-02）

- **调度契约**：审计发现 `test_infantry_minimal` 只初始化了应用，没有选择延后
  CAN/UART dispatch，也没有消费 inbox；已补齐为与 FreeRTOS 电机任务相同的
  “注册前配置、消费电机反馈前派发”顺序。
- **失败边界**：视觉 daemon 注册失败时先回滚最后一个 UART 端点；若 HAL 无法安全
  中止 DMA，`USARTUnregister()` 返回 `false` 并保留端点，视觉链路不再发送半初始化
  遥测，也不把该资源伪装成已释放。
- **构建边界**：正式 `app.elf` 只链接 `rm_components_runtime` 的新视觉路径；
  `rm_components_compat` 可为历史 demo 同时编译旧源文件与 runtime 依赖，但应用不得
  同时初始化两套协议。
- **验证边界**：本轮完成 `app.elf`、`test_infantry_minimal`、runtime/compat target
  的增量构建和差异检查；没有执行主机 `ctest`，因此主机测试运行结果仍以单独测试记录为准。

## 10. 实板验收清单

- [x] 两侧 CRC 对 §2.4 测试向量逐字节一致（软件向量已用独立测试程序对拍通过；实板
      连续收发仍待验收）；
- [ ] 921600 8N1 连续收发 10 分钟无校验错误（关注 `crc_errors` / `resync_bytes`）；
- [x] **实板确认 IMU Pitch 抬头符号**（§5.2，2026-09-11 已确认抬头为负）；
- [x] 上位机 `target_frame` 已改为 `gimbal_link`，URDF 已删除 `odom`；
- [ ] `camera_joint` 安装外参实测标定（**上机前必须完成**）；
- [ ] 自瞄未开启（SB 非中位）时，上位机异常下发目标不影响云台手动控制；
- [ ] 自瞄开启、`TRACKING = 1` 时云台跟随视觉目标，底盘与发射不受影响；
- [ ] **云台转动后目标不跑偏**（验证 §1.3.1 的相对增量用法，正反馈问题最易在此暴露）；
- [ ] 拔掉视觉串口后 250 ms 内云台退回手动控制，底盘与发射不中断；
- [ ] 视觉角度目标超出 `[-30, +20] deg` 时被正确夹限，云台不越界；
- [ ] 视觉下发 NaN / Inf 时云台不产生非法输出（Release 构建下同样验证）；
- [ ] 视觉目标单帧跳变时云台不出现猛打；
- [ ] 自瞄开启期间 SA 下位仍能立即停机；
- [ ] 链路延迟（相机曝光 → 云台响应）实测并记录。

## 11. 后续预留

以下项在当前实现中**不启用**，但协议已预留字段，避免后续破坏兼容：

| 项 | 预留位置 | 说明 |
| --- | --- | --- |
| 操作手云台微调 | 遥测 `flags` bit3 | 摇杆偏置叠加到视觉角度目标 |
| 上位机自动开火判定 | 目标 `flags` bit7 / 遥测 bit4 | 下位机保留最终开火决定权 |
| 弹速切换确认 | 遥测 `bullet_speed` | 上位机可据此调整可视化弹道 |
| 能量机关模式 | 遥测 `flags` 保留位 | 需扩展 payload，届时升版本 |
| 链路延迟补偿 | 两侧 `seq` 字段 | 下位机可估计并补偿固定延迟 |
| 弹道解算位置 | `(x,y,z,vx,vy,vz)` | 当前下位机未做提前量，目标帧已携带全部所需状态 |

协议若需扩展字段，优先使用保留位；确实需要增加 payload 时，**必须同步修改
两侧并把 `len` 作为版本区分依据**，不得静默改变长度。
