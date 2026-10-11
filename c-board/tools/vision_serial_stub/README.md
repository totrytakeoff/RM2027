# vision_serial_stub

文档更新日期：2026-09-11

在**没有相机、没有装甲板识别**的前提下闭环验证云台自瞄的**基本跟随**。

它替代上位机视觉栈：按 RM2026 协议收下位机遥测帧（姿态），在**虚拟世界系**里
放一个静止或绕行的目标，把"目标相对云台当前指向的方位"按协议发回去。
下位机无法区分它和真实视觉，因此看到的行为就是真实自瞄跟随的行为。

目的是把问题**分层定位**：如果这个桩能让云台稳定追上并停住，说明协议层 +
解算 + 云台控制链路是通的，剩下的问题就只在识别/跟踪算法；反之问题一定在
下位机侧，与视觉算法无关。

协议基线见 `docs/vision/vision_protocol.md`。

## 依赖

```bash
python3 -m pip install pyserial numpy
```

## 接线

下位机 USART1 的引脚是**非标准**的：

| 方向 | 下位机引脚 | 接到 USB-TTL |
| --- | --- | --- |
| 下位机 -> PC | PA9 (TX) | RXD |
| PC -> 下位机 | **PB7 (RX)** | TXD |
| 共地 | GND | GND |

> 注意 RX 在 **PB7** 而不是常见的 PA10。接错只会单向不通，现象是
> `rx=0`（桩收不到遥测）或下位机侧 `rx=0`（收不到目标）。

## 运行

```bash
# 静止目标，云台系下左偏 20 度、距离 1.5 m、高 0.2 m
python3 tools/vision_serial_stub/vision_stub.py --port /dev/ttyUSB0

# 让目标在虚拟世界里绕圈，验证云台能持续跟随而不是只在一点停住
python3 tools/vision_serial_stub/vision_stub.py --port /dev/ttyUSB0 --orbit-deg-s 20

# 只收不发，先确认下位机在发遥测（排查接线最快的一步）
python3 tools/vision_serial_stub/vision_stub.py --port /dev/ttyUSB0 --listen-only

# 不确定 yaw 正方向时：每 10 秒把目标方位取反，观察云台转哪边
python3 tools/vision_serial_stub/vision_stub.py --port /dev/ttyUSB0 --flip-every-s 10
```

下位机侧把遥控 SB 打到中位（进入 `ROBOT_CONTROL_AUTO_AIM`），
然后看下位机日志里的 `aim ACTIVE ...` 行。

## 判断结果

| 现象 | 结论 |
| --- | --- |
| 云台转向目标并停住，`aim ACTIVE` | 基本跟随成立，链路全通 |
| 云台持续向一个方向转到底 | `VISION_YAW_FOLLOW_ENABLE` 反了（见下） |
| 云台不动，日志 `idle` 且 `tgt>0` | 遥控没打到中位（不是角度保持态） |
| 日志 `rx=0` | 桩收不到遥测：接线 / 波特率 |
| 桩报 CRC 错误 | 协议不同步：两侧版本不一致 |

**Yaw 方向**：本车 IMU yaw 的正方向没有在仓库里被明确验证过
（`docs/remote_control/*.md` 里"Yaw 摇杆方向正确"至今是未勾选项）。
若不接管时云台稳定转反，把 `robot_config.h` 的
`VISION_YAW_FOLLOW_ENABLE` 改成 `0U` 即可，**不需要改动解算与主机测试**。

## 协议要点（违反会"打反方向"）

下位机的跟踪器 `target_frame = gimbal_link`，即 `gimbal_link` **随云台转动**。
所以下发的 `(x, y, z)` 是目标**相对云台当前指向**的方位，不是世界系绝对坐标。

本桩的做法因此是：维护虚拟世界系下目标的绝对方位 `world_yaw`，发送时
`relative = world_yaw - 遥测里的当前 yaw`——即把世界系目标换算到云台系。

这也意味着一个**很容易搞错**的点：如果直接让 `world_yaw` 保持常量而忽略
遥测 yaw，桩发出去的就是世界系绝对方位，下位机会把它当成相对方位用，
形成正反馈持续打偏。这是本桩唯一容易写错的地方，代码里已标注。
