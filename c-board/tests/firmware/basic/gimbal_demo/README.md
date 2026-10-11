# 云台遥控控制演示

## 项目简介

本项目整合了遥控接收和电机控制，实现双轴云台的遥控速度控制。使用遥控器右摇杆控制云台：左右控制 yaw 轴，上下控制 pitch 轴。

## 硬件配置

- yaw 轴 GM6020：CAN1，ID 2
- pitch 轴 GM6020：CAN2，ID 1

## 遥控器映射

- 右摇杆左右：yaw 轴转动速度
- 右摇杆上下：pitch 轴转动速度

## 参数配置

- 控制更新频率：50 Hz
- GM6020 最大速度：3600 deg/s

## 构建和烧录

### 构建
```bash
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Debug
ninja -C build gimbal_demo
```

### 烧录
```bash
ninja -C build upload-gimbal_demo
```

## 注意事项

1. 确保电机 ID 和 CAN 接线与代码一致
2. 如转向与手感不符，可在代码中调整方向符号
