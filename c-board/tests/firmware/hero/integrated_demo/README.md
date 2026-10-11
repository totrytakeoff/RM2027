# 一体化演示（底盘 + 云台 + 射击）

## 项目简介

本项目将麦轮底盘、双轴云台与 Hero 射击机构整合到同一固件中，统一遥控器映射并避免 CAN ID 冲突，适用于整机联调与功能验证。

## 硬件配置

- CAN1：麦轮底盘 M3508，ID1~4（左前/右前/右后/左后）
- CAN1：拨弹 M3508，ID5
- CAN1：Yaw GM6020，ID2
- CAN2：摩擦轮 M3508，ID1/2（ID2 方向反转）
- CAN2：Pitch GM6020，ID5

## 遥控器映射

- 左拨杆（S1）上：开启摩擦轮（开启后右摇杆交给云台）
- 右拨杆（S2）：下=单发，中=双发，上=连发
- 拨轮：上拨超过阈值触发拨弹，松开停止
- 底盘（按当前通道映射）
  - `rocker_l_`：前后平动（vy，仅在摩擦轮关闭时启用）
  - `rocker_l1`：左右平移（vx）
  - `rocker_r_`：旋转（wz）
- 云台（摩擦轮开启时）
  - `rocker_r_`：yaw
  - `rocker_r1`：pitch

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
ninja -C build test_integrated_demo
```

### 烧录

```bash
ninja -C build flash-test_integrated_demo

# 或使用脚本（从工程根目录执行）
.\script\upload.ps1 test_integrated_demo --format bin
```

## 注意事项

1. 确保 CAN 线束与电机 ID 配置一致
2. 遥控器离线或数值异常时会强制停机
3. 上电后底盘电机有 2 秒稳定期，期间输出为 0
4. 云台仅在摩擦轮开启时启用，Pitch 默认带重力补偿
