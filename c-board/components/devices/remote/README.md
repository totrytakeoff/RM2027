# 遥控设备组件

更新日期：2026-07-23

本目录只保存可复用的遥控设备解析、统一状态和适配器，不定义任何具体机器人行为。

- `remote_control_state.h`：C 兼容的统一输入状态，覆盖摇杆、拨杆、旋钮、辅助轴、键盘、鼠标和按钮。
- `remote_control_adapter.hpp`：C++14 静态适配器，统一生成按下、释放和开关变化边沿。
- `et08/`、`dt7/`、`vt/`：互相独立的协议驱动和 Backend。

机器人模式映射位于 `applications/<robot>/command`，电机机械限位位于对应执行模块。
正式机器人固件的设计、ET08 映射、安全重上锁和 C/C++ ABI 约定见
[遥控适配器与机器人输入基线](../../../docs/architecture/remote_control_adapter_baseline.md)。

项目早期遥控实现参考来源统一记录在 `THIRD_PARTY_NOTICES.md`，组件路径和接口不使用来源战队命名。
