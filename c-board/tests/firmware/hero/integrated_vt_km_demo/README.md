# 图传键鼠一体化演示（底盘 + 云台 + 射击）

## 项目简介

本项目将麦轮底盘、双轴云台与 Hero 射击机构整合到同一固件中，输入源采用 VT03/VT13 图传链路键鼠数据，适用于整机联调与功能验证。

## 硬件配置

- CAN1：麦轮底盘 M3508，ID1~4（左前/右前/右后/左后）
- CAN1：拨弹 M3508，ID5
- CAN1：Yaw GM6020，ID2
- CAN2：摩擦轮 M3508，ID1/2（ID2 方向反转）
- CAN2：Pitch GM6020，ID5

## 键鼠映射

- 安全
  - `pause`：急停
  - `gear=S`：允许云台和发射
- 底盘
  - `W/S`：前后平动（vy +/-）
  - `A/D`：左右平移（vx -/+）
  - `Q/E`：车体旋转（wz -/+）
  - `Shift`：加速倍率
  - `Ctrl`：慢速倍率
  - 右摇杆兜底：`ch0 -> vx`，`ch1 -> vy`，`ch3 -> wz`（用于排查键鼠链路）
- 云台（仅 `gear=S`）
  - 鼠标 `X`：yaw 角速度
  - 鼠标 `Y`：pitch 角速度
  - 鼠标无输入时左摇杆兜底：`ch3 -> yaw`，`ch2 -> pitch`
- 射击（仅 `gear=S`）
  - `R`：摩擦轮开关（按下切换）
  - 鼠标左键：单发（边沿触发）
  - 鼠标中键：双发（边沿触发）
  - 鼠标右键：连发（按住生效）

> 注：若摇杆通道与实际不一致，以 `remote_control_demo` 的打印结果为准并在代码中调整。

## 发射逻辑

- 摩擦轮开启后才允许拨弹
- 单发/双发：短间隔触发步进，拨盘每次旋转 60 deg x 19
- 连发：拨弹电机以速度模式持续转动

## 参数配置

- 控制更新周期：20 ms（50 Hz）
- 轮子半径：0.075 m；轮距：0.34 m
- 最大平动速度：20.0 m/s；最大旋转速度：120.0 rad/s
- GM6020 最大速度：3600 deg/s；Pitch 静态补偿：2500
- 摩擦轮目标速度：30000 deg/s；拨弹连续速度：12000 deg/s

## 构建和烧录

### 构建

```bash
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Debug
    ninja -C build test_integrated_vt_km_demo
```

### 烧录

```bash
ninja -C build flash-test_integrated_vt_km_demo

# 或使用脚本（从工程根目录执行）
.\script\upload.ps1 test_integrated_vt_km_demo --format bin
```

## 注意事项

1. 确保 CAN 线束与电机 ID 配置一致
2. 遥控器离线或数值异常时会强制停机
3. 默认图传串口为 `USART6(huart6)`；若你的发送端接在其他串口，请修改 `main.c` 中 `VT_UART_HANDLE`
4. 上电后底盘电机有 2 秒稳定期，期间输出为 0
5. 云台控制要求 `gear=S`，Pitch 默认带重力补偿
6. 调试宏位于 `main.c`：`VT_REQUIRE_CRC_OK`（默认1）、`VT_ENABLE_PAUSE_ESTOP`（默认1）、`VT_REQUIRE_ONLINE`（默认1）
