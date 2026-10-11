# 轮腿关节电机调参说明（DM8009P MIT）

文档更新日期：2026-07-19

适用范围
- 目标：MIT 模式下让关节在负载下稳定定姿、抖动最小
- 适用测试：`tests/firmware/leg/wheelleg_joint_test`、`tests/firmware/leg/wheelleg_balance_test`
- MIT 通用原理参考：`docs/motor/dm8009p_mit_tuning_guide.md`

总线与ID
- CAN1：四个关节电机
- ID：1 左前(FL)，2 左后(RL)，3 右前(FR)，4 右后(RR)
- Master ID：0x00（共享反馈，D[0]低4位区分电机）

串联腿映射（非常关键）
- 髋关节由前电机直接驱动
- 膝关节是串联在髋后的机构，后电机需要跟随“髋 + 膝”的组合角
- 代码体现：
  - 髋目标：`HIP_TARGET_L/R`
  - 膝目标：`HIP_TARGET_L/R + KNEE_TARGET_L/R`

参数修改位置
- 关节目标、每电机增益：`tests/firmware/leg/wheelleg_balance_test/main.c`
  - `HIP_TARGET_*`、`KNEE_TARGET_*`
  - `FL_KP/KD`、`FR_KP/KD`、`RL_KP/KD`、`RR_KP/KD`
  - `HIP_TFF_*`、`KNEE_TFF_*`
- MIT 映射范围：`DM_P_RANGE`、`DM_V_RANGE`、`DM_T_RANGE`
- 驱动限制：KD 在驱动内被限制为 <= 5.0

推荐调参流程（按顺序做）
1) 先关闭平衡和轮毂
   - SA 下拨关闭平衡
   - 只让关节上电并持续发 MIT
2) 设置安全姿态
   - 用你当前稳定的“落地姿态”作为目标
   - 膝关节保持在物理安全范围内
3) 先调重力补偿（TFF）
   - 目标：负载下位置误差尽量小
   - 步进：0.5~1.0 N·m
   - 以 `tq[]` 日志为第一版 TFF 符号与幅值
4) 再加 Kp 提刚度
   - 步进：5~10（N·m/rad）
   - 目标：下压误差 < 0.02~0.05 rad
5) 最后加 Kd 抑制抖动
   - 步进：0.2~0.5（N·m·s/rad）
   - 注意 KD 上限 5.0

常见现象与处理
- 负载下下压明显
  - 先加 TFF，再加 Kp
- 抖动随时间加重
  - 先补 TFF，减少靠误差硬顶
  - 适当下调 Kp 10~20%
  - 小幅上调 Kd（<=5）
  - 检查供电下垂/温升漂移
- 左右侧表现不一致
  - 单独调该电机参数，不要四个一起改
  - 再检查镜像方向/传动一致性
- 停止时嗡嗡震动
  - Kp 过高/齿隙影响明显
  - 降 Kp、微调 TFF

日志检查清单
- 位置：`joint pos[FL RL FR RR]`
- 扭矩：`tq[FL RL FR RR]`
- 错误：`err[FL RL FR RR]`
- 如果扭矩很大但位置仍漂移：优先补 TFF，再降 Kp

安全注意
- MIT 必须持续发帧，否则电机会失能
- 膝关节不要撞限位
- 电机掉线/失能时，先看 error_state 与供电/总线
- 增大增益前先确认镜像方向正确

进入平衡调参前的标准
- 固定姿态能稳定 1~2 分钟
- 轻微扰动后能快速回到目标，无明显抖动
- 扭矩不过载、温升可接受
