# infantry_omni_demo 详解与排障指南

文档更新日期：2026-07-23

本文面向 `tests/firmware/infantry/infantry_omni_demo/main.c` 的整车 Demo，说明功能、CAN/遥控映射、关键参数与排障思路。

## 1. Demo 目标与组成
- 底盘：全向轮/麦轮底盘，CAN1，M3508，ID1~4
- 云台：Yaw GM6020 在 CAN1 ID1；Pitch GM6020 在 CAN2 ID1
- 发射：摩擦轮 M3508 在 CAN2 ID1/2；拨弹 M2006 在 CAN2 ID6
- 遥控：ET08 SBUS（参照 `tests/firmware/basic/et08_sbus_demo` 的通信逻辑）

## 2. 关键文件
- 入口：`tests/firmware/infantry/infantry_omni_demo/main.c`
- CAN 配置：`platform/boards/rm_f407/generated/can.c`
- DJI 电机驱动：`components/devices/motor/dji/dji_motor.c`
- CAN BSP：`platform/stm32f4/can/bsp_can.c`

## 3. CAN 总线与电机 ID
### 3.1 发送分组说明（DJI 电机驱动）
DJI 电机以 4 个为一组打包发送。
- M2006/M3508：CAN 发送 ID 0x1FF/0x200，接收 ID = 0x200 + motor_id
- GM6020：CAN 发送 ID 0x1FF/0x2FF，接收 ID = 0x204 + motor_id

### 3.2 本 Demo 约定
- CAN1：底盘 M3508 ID1~4，Yaw GM6020 ID1
- CAN2：摩擦 M3508 ID1/2，Pitch GM6020 ID1，拨弹 M2006 ID6

注意：GM6020 的 ID1 与 M3508 的 ID1 不冲突，因为接收 ID 起始地址不同。

## 4. 遥控（ET08 SBUS）逻辑
### 4.1 SBUS 串口
- UART3，100000 9E2（8N1 会导致丢帧）
- 在 `Et08SbusInit()` 中仅注册一次回调，避免重复注册导致卡死

### 4.2 通道映射
可在 `ET08_MAPPING_MODE` 切换左右摇杆映射：
- `0`：左=CH3/CH4，右=CH1/CH2
- `1`：左=CH1/CH2，右=CH3/CH4

### 4.3 默认控制映射
- 左摇杆：底盘平移（x=横移，y=前后）
- 右摇杆：x=底盘旋转，y=云台 pitch
- SA：摩擦轮（上=开，下=关）
- SB：拨弹（上=连发，中=停止，下=单发切换）
- SD：云台跟随（上=跟随，下=不跟随）
- SC：未使用
- LD/RD：未使用

## 5. 关键参数说明
- `PITCH_SPEED_SCALE`：Pitch 速度缩放（当前 0.15）
- `GIMBAL_NO_FOLLOW_WZ_SCALE`：不跟随模式下的底盘反向补偿比例
- `CHASSIS_MAX_VEL / CHASSIS_MAX_ROTATE`：底盘最大线速度/角速度
- `CHASSIS_SPEED_MULTIPLIER`：轮速整体缩放
- `MOTOR_STABILIZE_TIME_MS`：上电后电机稳定等待时间
- `SBUS_ONLINE_TIMEOUT_MS`：SBUS 掉线判定
- `SHOOT_INTERVAL_MS`：单发间隔
- `LOADER_CONTINUOUS_HOLD_MS`：连发保持的去抖延时
- `FRICTION_SPEED_TARGET`：摩擦轮目标转速

## 6. 常见问题与排障
### 6.1 CAN1 只在上电几秒可控，随后不动
可能原因：
- CAN1 进入 bus-off（线缆/终端/电源抖动导致错误累积）
- 过滤器/注册冲突导致 CAN1 接收停止

对应处理：
- 已在 `platform/boards/rm_f407/generated/can.c` 打开 AutoBusOff（CAN1/CAN2），使其自动从 bus-off 恢复
- 如仍出现：建议在主循环中增加 CAN1 错误检测并重启 CAN（可按需添加）

### 6.2 CAN2 正常，CAN1 完全无响应
排查顺序：
1) 确认 CAN1 收发脚：`PD0/PD1` 连接正确，终端电阻存在
2) 单独烧录 `tests/firmware/basic/omni_chassis_demo` 验证 CAN1
3) 在 `tests/firmware/infantry/infantry_omni_demo` 中打开 `CHASSIS_FORCE_TEST` 强制输出
4) 若仍无响应，倾向硬件/总线层问题

### 6.3 摇杆方向不一致
修改位置：
- `ProcessChassisControl()` 中的 `chassis_vx/chassis_vy/chassis_wz` 符号
- `ProcessGimbalControl()` 中 pitch 的符号

### 6.4 Pitch 速度过大
调整 `PITCH_SPEED_SCALE`，推荐 0.1~0.2 区间逐步调小。

## 7. 修改与扩展建议
- 若要增加日志：建议使用板载串口（如 USART6）或蜂鸣器节奏提示
- 若要稳定控制：增加 SBUS 超时降级逻辑、CAN 错误复位机制
- 若只调试底盘：可将 `DEMO_CHASSIS_ONLY` 置为 1 屏蔽云台与发射

## 8. 版本变更记录
- 打开 AutoBusOff：`platform/boards/rm_f407/generated/can.c`，CAN1/CAN2 均设置为 `ENABLE`
- ET08 SBUS 解析：使用 UART3 100k 9E2，回调注册一次
- Pitch 速度缩放：`PITCH_SPEED_SCALE = 0.15`
