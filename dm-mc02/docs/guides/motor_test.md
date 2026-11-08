# MC-02 单电机测试

`motor_test.elf` 测试 CAN1 上 ID 为 1 的 M3508/C620，SD 下档使用左摇杆 Y 控速，SD 上档使用右摇杆 Y 控位置，SD 中档关闭输出。C620 反馈 ID 为 `0x201`，控制帧 ID 为 `0x200`；该帧中 ID 2～4 的槽位为零，测试时不要将它们用于其他独立控制器。

接线沿用底盘 demo：ET08 接收机 SBUS 接 UART5/PD2，C620 接 CAN1，电机和电调使用合适的独立电源及共地。上电时保持主控静止以完成 IMU 校准。USART1 为 115200 8N1 框架诊断口。

操作与底盘保持一致：SA 上档使能，SA 中/下档关闭；板载按键按下急停。遥控器失联或数据无效、IMU 不就绪、任务不健康、CAN 离线或电机反馈超时均会停止输出。采用底盘的 SA 电平门策略，故障恢复后若 SA 仍在上档会自动恢复，请先将 SA 拨离上档再排查。上电至少等待 2 秒且 IMU 就绪后才能使能。输出门租约和电机命令超时保护沿用框架。

速度模式中，摇杆中位为零；采用底盘配置的 100 raw 死区和 660 单边量程，死区外线性映射。满杆转子目标速度为 ±523.6 rad/s（±5000 rpm）；方向和上限分别由 `config/robots/motor_test/motor_test_config.h` 中的 `MOTOR_TEST_DIRECTION`、`MOTOR_TEST_MAX_RAD_S` 调整。速度环直接输出 C620 电流指令，限幅为 ±16384；速度 PI 使用独立配置；反馈超时和遥控映射继承 `config/robots/chassis_demo/chassis_config.h`，单电机绑定见 `config/robots/motor_test/binding.c`。实际转速和电流取决于供电与负载。

位置模式以进入该模式且满足使能条件时的当前位置作为零位；SA 关闭后再次使能也会重新捕获零位。右摇杆 Y 绝对映射到输出轴 −π～+π rad（±180°），回中回到本次零位；按 268/19 减速比换算为转子目标角度，不是持续累加位置。模式切换会关闭上一租约并重置 PID。若位置反馈连续性丢失，停止输出，须将 SA 拨离上档或退出位置模式后重新捕获零位。

位置外环使用 P=4 s⁻¹（在 DJI 转子角度 deg → 转子速度 rad/s 接口中配置为 `4 × π/180`），输出送入已整定的速度 PI。位置目标范围不是机械限位保护；没有限位开关或绝对编码器，归零来自当前增量反馈。当前外环参数仅完成主机测试，尚待位置模式实板验证。

通过 `motor_test_config.h` 中 `MOTOR_TEST_MODEL` 选择型号，默认 `MOTOR_TEST_MODEL_M3508`。总线和 ID 使用 `MOTOR_TEST_CAN`、`MOTOR_TEST_ID`；减速比与各环参数在对应型号分支中定义。M3508 已实测整定；DM4310 已接入主控力矩串级，独立目标为 `motor_test_dm4310`，配置与验证状态见 [DM4310 测试说明](motor_test_dm4310.md)。其他型号仍需添加对应参数及驱动绑定。

```sh
./scripts/build.sh build --target motor_test
./scripts/upload.sh motor_test
# 或 cmake --build --preset h7-debug --target motor_test.elf
```

输出文件位于 `build/h7-debug/motor_test.{elf,bin,hex}`。首次通电时固定电机并卸载机械负载，SA 置于下档，确认反馈和方向后再缓慢推杆。需要仅观察时，可在独立构建目录配置 `-DRM_MOTOR_TEST_ENABLE_MOTION=OFF`；此时仍执行输入和门控逻辑，但电机任务只发送零指令。


减速箱按 **268/19 = 14.105263:1** 记录，C620 的 `rpm` 与速度闭环均使用转子单位。输出轴转速 = 转子 rpm × 19/268，因此 5000 rpm 转子目标对应输出轴约 354.48 rpm。此处减速比仅用于机械转速换算，不重复乘到 C620 反馈上。

USART1 每 10 ms 尝试输出一行 `MT`（发送忙时可丢样）：`t` 为 ms，`e` 为命令使能，`w` 为停止原因，`o` 为反馈在线，`r` 为转子目标 rpm，`v` 为转子实测 rpm，`i` 为电调反馈电流原始单位，`T` 为温度 °C。位置模式输出 `MP`，其中 `p`、`a` 分别为相对输出轴目标和实测位置，单位 mrad；`v` 仍为转子 rpm。记录命令：

```sh
python3 tools/serial_capture.py --device /dev/ttyACM0 --duration 60 --output /tmp/motor_test_uart.txt
```


## 2026-11-08 实板整定

本次使用当前 268/19 减速箱及用户现场测试装配，经 USART1 采集正反向推杆、回中及高转速测试。最终速度 PI：`Kp=20, Ki=80, Kd=0`（反馈为转子 rad/s，积分使用秒），积分项限幅 5000，输出电流指令限幅 ±16384；反馈滤波与 5 ms 电机任务周期保持框架默认值。已编译、烧录并完成串口复测。

原速度环直出电流时沿用了底盘串级控制的 Kp=229.18312，所测约 36 Hz 振荡。原目标零速窗口实际范围 −1024～1113 rpm、标准差 677 rpm；整定后回中稳态窗口标准差约 3.7 rpm。反向满速目标 −4999 rpm，稳态窗口实际均值约 −5015 rpm，标准差约 27 rpm。所有转速均为转子单位，输出轴需除以 268/19。

证据在 [motor_test 验证目录](../validation/motor_test/)，包含原始串口、统计与 [对比图](../validation/motor_test/tuning.png)。本次结果针对当前装配与现场测试负载，未覆盖其他惯量或长时间满负载运行。
