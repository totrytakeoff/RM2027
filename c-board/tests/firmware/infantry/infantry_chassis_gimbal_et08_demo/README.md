# 步兵底盘云台 ET08 融合测试

文档更新日期：2026-07-19

目标：验证一套完整的“云台 IMU 闭环 + 底盘相对角闭环”的 infantry 融合控制链路，而不是只测底盘逆解。

这份 demo 的核心不是炫技，而是把三个容易混淆的姿态量彻底拆开：

- 云台绝对朝向：`gimbal_imu->YawTotalAngle`
- 云台相对底盘夹角：`yaw_motor_encoder - align_offset`
- 底盘跟随输出：由相对夹角闭环得到 `chassis wz`

## 控制定义

- 左摇杆：底盘平移
- 右摇杆左右：云台 Yaw
- 右摇杆上下：云台 Pitch
- `SD` 上：`FOLLOW`
- `SD` 下：`SEPARATE`

## 两种模式

### FOLLOW

- 云台 Yaw 始终使用 IMU `YawTotalAngle` 闭环，保持对世界系的角度目标。
- 底盘读取 Yaw 电机编码器，计算“云台相对底盘夹角” `yaw_offset_deg`。
- 底盘用该相对角做两件事：
  - 把左摇杆平移指令从“云台坐标系”旋转到底盘坐标系
  - 用 `yaw_offset_deg -> wz_cmd` 闭环，让底盘追着云台转，直到 `yaw_offset_deg -> 0`

结果：

- 云台指到哪，底盘跟到哪
- 推左摇杆前，始终沿云台前向平移

### SEPARATE

- 云台 Yaw 仍然是 IMU 闭环，继续对齐世界系目标。
- 底盘不再跟随云台，`wz_cmd=0`
- 但底盘仍然使用 Yaw 编码器得到的 `yaw_offset_deg` 做坐标变换

结果：

- 云台与底盘朝向可以不同
- 推左摇杆前，底盘仍会沿云台前向平移

## 为什么不能只靠 IMU 做底盘变换

如果只有云台上的 IMU，那么 `YawTotalAngle` 代表的是“云台相对世界”的绝对角。

底盘平移坐标变换真正需要的是“云台相对底盘”的夹角：

- `yaw_offset = yaw_gimbal_abs - yaw_chassis_abs`

当前 demo 没有底盘 IMU，所以不能仅靠 `YawTotalAngle` 求得该相对角；这里必须依赖 Yaw 电机编码器。

因此当前实现的职责划分是：

- 云台 IMU：负责云台绝对角闭环
- Yaw 编码器：负责底盘相对角变换与 FOLLOW 闭环

## 当前实现结构

主文件：

- [main.c](main.c)

主要函数：

- `GimbalUpdateFromET08()`
  - 右摇杆更新 `yaw_hold_ref`
  - Yaw 始终走 `OTHER_FEED`，即 IMU 角度/角速度反馈
  - Pitch 采用“速度控制 + 松杆保持”
- `GetYawOffsetDeg()`
  - 从 `motor_yaw->measure.angle_single_round` 计算云台相对底盘夹角
- `BuildChassisCommandFromEt08()`
  - 左摇杆生成云台坐标系下的 `vx_cmd / vy_cmd`
  - 按 `yaw_offset_deg` 旋转成 `vx_body / vy_body`
  - `FOLLOW` 下由 `yaw_offset_deg` 生成 `wz_cmd`
- `ChassisApplyCommand()`
  - 低通滤波后进行全向轮逆解

## 姿态量定义

### 云台绝对角

- 来源：`gimbal_imu->YawTotalAngle`
- 含义：云台当前相对世界系的累计 Yaw
- 用途：云台 Yaw 闭环反馈

### 云台相对底盘夹角

- 来源：`motor_yaw->measure.angle_single_round - YAW_ALIGN_ANGLE_DEG`
- 经过 `WrapAngleDeg180()` 包裹到 `[-180, 180]`
- 用途：
  - 底盘平移坐标变换
  - FOLLOW 模式底盘跟随闭环

## 调参入口

当前最关键的两个参数在 [main.c](main.c)：

- `YAW_CHASSIS_ALIGN_ECD`
  - Yaw 电机机械零位
  - 如果这项错了，分离模式下平移方向会整体偏
- `CHASSIS_FOLLOW_WZ_KP / CHASSIS_FOLLOW_WZ_KD / CHASSIS_FOLLOW_WZ_MAX`
  - FOLLOW 模式底盘跟随增益
  - 太小会跟随拖沓，太大可能左右摆

推荐调参顺序：

1. 先校正 `YAW_CHASSIS_ALIGN_ECD`
2. 在 `SEPARATE` 模式验证“云台朝前时左摇杆前推，底盘是否真向云台前方走”
3. 再切到 `FOLLOW` 模式，调 `KP/KD`
4. 最后再观察轮速和 Pitch 保持

## 已知逻辑边界

这版实现是可运行的融合 demo，但仍有几个明确边界：

1. `SEPARATE` 模式下，底盘没有独立人工旋转输入  
   因为右摇杆已经完全分配给云台，当前实现里 `SEPARATE` 仅表示“不跟随”，不是“允许独立旋转”

2. FOLLOW 的阻尼仍然来自 Yaw 电机编码器速度  
   当前没有底盘 IMU，因此 `KD` 项不是底盘真实角速度，而是云台相对底盘转轴速度的近似项

3. `yaw_offset_deg` 当前取单圈包裹角  
   对正常 infantry 云台左右摆角范围是够用的；如果后续机械允许极大角度偏转，可再改成更稳的多圈相对角管理

4. Pitch 仍然沿用 demo 风格的“速度+保持”  
   它不是这份融合 demo 的重点，但保留是为了现场联调时不影响云台姿态

## 调试输出

### 文本串口

- USART6，100ms 周期
- 输出模式、IMU Yaw、Yaw 编码器、相对角、底盘变换后速度、轮速

### VOFA+

- USART6，JustFloat
- 20ms 周期
- 通道说明见：
  - [vofa_channels.md](vofa_channels.md)
- 示例 CSV：
  - [vofa_sample.csv](vofa_sample.csv)

## 构建

```bash
cmake -S . -B build
cmake --build build --target test_infantry_chassis_gimbal_et08_demo -j8
```

## 烧录

```bash
cmake --build build --target flash-test_infantry_chassis_gimbal_et08_demo
```
