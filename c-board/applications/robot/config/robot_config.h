/**
 * @file robot_config.h
 * @brief 机器人应用配置文件 - 所有参数集中于此
 * @date 2026-07-22
 * @note 修改参数后必须重新编译；完整说明见 docs/motor/control_parameter_reference.md
 *
 * 参数标记：
 * - [核心] 直接决定整车速度、射频或机械工作范围，上机前必须确认；
 * - [标定] 与本车几何、安装方向或零位绑定，更换机构后重新测量；
 * - [PID]  闭环参数，只能结合对应遥测逐步调整；
 * - [保护] 超时、限幅、就绪或故障阈值，不用于补偿正常控制性能；
 * - [派生] 由核心/标定参数计算，禁止脱离公式单独填写；
 * - [固定] 协议、硬件连接或控制拓扑，通常不属于调参项。
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "main.h"

/*============================================================================
 * 机器人应用调试系统配置
 *============================================================================*/
#ifndef MINIMAL_DEBUG_ENABLE
/*
 * [核心-调试] 视觉上板联调期临时打开：自瞄状态只有文本日志能看到
 * （aim ACTIVE/idle + rx/crc/tgt/rej 增量计数）。
 * 与调参遥测互斥——文本日志和 VOFA 帧都在 USART6 上，同时开会互相穿插成乱码，
 * 因此打开本开关时必须同时把 ROBOT_TUNING_TELEMETRY_ENABLE 置 0。
 * 联调结束、不再需要看自瞄状态时，两个开关各自恢复原值即可。
 */
#define MINIMAL_DEBUG_ENABLE                        1U /* 通用文本/RTT日志总开关 */
#endif

#define MINIMAL_DEBUG_MODE_TEXT                     (1U << 0) /* [固定] 文本日志位 */
#define MINIMAL_DEBUG_MODE_VOFA                     (1U << 1) /* [固定] 旧 VOFA 日志位 */
#define MINIMAL_DEBUG_MODE                          MINIMAL_DEBUG_MODE_TEXT /* 仅在总开关为 1 时生效 */

#define MINIMAL_DEBUG_UART_PORT                     6U      /* [固定] 通用文本日志串口 */
#define MINIMAL_DEBUG_UART_TIMEOUT_MS               100U    /* [保护] 阻塞文本发送超时 */
#define MINIMAL_DEBUG_UART_BAUDRATE                 115200U /* [固定] 文本日志波特率 */
#define MINIMAL_DEBUG_ALLOW_MIXED_STREAM            0U   /* 0: 单串口禁止Text+VOFA混发 */

#define MINIMAL_DEBUG_MOD_SYSTEM                    1U /* 以下仅在文本日志总开关为 1 时生效 */
#define MINIMAL_DEBUG_MOD_INPUT                     1U
#define MINIMAL_DEBUG_MOD_CHASSIS                   1U
#define MINIMAL_DEBUG_MOD_GIMBAL                    1U
#define MINIMAL_DEBUG_MOD_SHOOT                     0U

#define MINIMAL_DEBUG_TEXT_PERIOD_MS                200U /* 文本周期 5 Hz */
#define MINIMAL_DEBUG_VOFA_PERIOD_MS                20U  /* 旧 VOFA 周期 50 Hz */
#define GIMBAL_DEBUG_DETAIL_PERIOD_MS               100U /* 云台详细文本周期 10 Hz */

/*
 * 独立调参遥测：不复用通用日志任务，不执行字符串格式化。
 * 当前使用 UART6 向 VOFA+ 发送 JustFloat 自定义通道。
 */
#ifndef ROBOT_TUNING_TELEMETRY_ENABLE
/*
 * 视觉联调期关闭：让出 USART6 给通用文本日志（见 MINIMAL_DEBUG_ENABLE）。
 * 需要 VOFA 波形时把它改回 1U，并同时关闭文本日志。
 */
#define ROBOT_TUNING_TELEMETRY_ENABLE            0U /* [核心-调试] 独立二进制遥测总开关 */
#endif
#define ROBOT_TUNING_UART_PORT                    6U      /* [固定] 当前使用 USART6 */
#define ROBOT_TUNING_UART_BAUDRATE                115200U /* [固定] VOFA+ 波特率 */
#define ROBOT_TUNING_TASK_PERIOD_MS               20U     /* [核心-调试] 50 Hz 采样周期 */

#if ROBOT_TUNING_UART_PORT == 1U
#define ROBOT_TUNING_UART_HANDLE huart1
#elif ROBOT_TUNING_UART_PORT == 3U
#define ROBOT_TUNING_UART_HANDLE huart3
#elif ROBOT_TUNING_UART_PORT == 6U
#define ROBOT_TUNING_UART_HANDLE huart6
#else
#error "Unsupported ROBOT_TUNING_UART_PORT, use 1/3/6."
#endif

/* 桥接到bsp_log */
#ifndef BSP_LOG_USE_UART
#define BSP_LOG_USE_UART                            0 /* 通用日志不得占用调参串口 */
#endif
#ifndef BSP_LOG_UART_PORT
#define BSP_LOG_UART_PORT                           MINIMAL_DEBUG_UART_PORT
#endif

#if (!MINIMAL_DEBUG_ENABLE) || ((MINIMAL_DEBUG_MODE & MINIMAL_DEBUG_MODE_TEXT) == 0)
#ifndef DISABLE_LOG_SYSTEM
#define DISABLE_LOG_SYSTEM 1
#endif
#endif

#if MINIMAL_DEBUG_UART_PORT == 1U
#define MINIMAL_DEBUG_UART_HANDLE huart1
#elif MINIMAL_DEBUG_UART_PORT == 3U
#define MINIMAL_DEBUG_UART_HANDLE huart3
#elif MINIMAL_DEBUG_UART_PORT == 6U
#define MINIMAL_DEBUG_UART_HANDLE huart6
#else
#error "Unsupported MINIMAL_DEBUG_UART_PORT, use 1/3/6."
#endif

/*============================================================================
 * 遥控后端：[固定] 编译期只允许选择一个，不进行运行时仲裁或自动回退。
 *============================================================================*/
#define ROBOT_REMOTE_BACKEND_ET08 1U
#define ROBOT_REMOTE_BACKEND_DT7  2U
#define ROBOT_REMOTE_BACKEND_VT   3U

#ifndef ROBOT_REMOTE_BACKEND
#define ROBOT_REMOTE_BACKEND ROBOT_REMOTE_BACKEND_ET08
#endif

#if (ROBOT_REMOTE_BACKEND != ROBOT_REMOTE_BACKEND_ET08) && \
    (ROBOT_REMOTE_BACKEND != ROBOT_REMOTE_BACKEND_DT7) &&  \
    (ROBOT_REMOTE_BACKEND != ROBOT_REMOTE_BACKEND_VT)
#error "ROBOT_REMOTE_BACKEND must select exactly one supported backend"
#endif

#if MINIMAL_DEBUG_ENABLE &&                                            \
    ((((ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_ET08) ||    \
       (ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_DT7)) &&    \
      (MINIMAL_DEBUG_UART_PORT == 3U)) ||                              \
     ((ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_VT) &&       \
      (MINIMAL_DEBUG_UART_PORT == 6U)))
#error "Debug UART conflicts with the selected remote-control backend"
#endif

#if ROBOT_TUNING_TELEMETRY_ENABLE &&                               \
    ((((ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_ET08) ||   \
       (ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_DT7)) &&   \
      (ROBOT_TUNING_UART_PORT == 3U)) ||                           \
     ((ROBOT_REMOTE_BACKEND == ROBOT_REMOTE_BACKEND_VT) &&      \
      (ROBOT_TUNING_UART_PORT == 6U)))
#error "Tuning UART conflicts with the selected remote-control backend"
#endif

#if MINIMAL_DEBUG_ENABLE && ROBOT_TUNING_TELEMETRY_ENABLE &&       \
    (MINIMAL_DEBUG_UART_PORT == ROBOT_TUNING_UART_PORT)
#error "Generic debug and tuning telemetry cannot share one UART"
#endif

/*============================================================================
 * 系统配置
 *============================================================================*/
#define MAIN_LOOP_PERIOD_MS             20U    /* [核心-时序] 应用控制周期，50 Hz */
#define MOTOR_STABILIZE_TIME_MS         2000U  /* [保护] 上电后注册电机前等待 2 s */
#define ROBOT_CPU_FREQUENCY_MHZ      168U   /* [固定] STM32F407 主频 */
#define ROBOT_IMU_INIT_TIMEOUT_MS    15000U /* [保护] INS 初始化最长等待时间 */

/* FreeRTOS迁移阶段任务周期：控制环保持minimal基线，快环独立调度。 */
#define ROBOT_INS_TASK_PERIOD_MS         1U  /* [核心-时序] IMU 更新周期，1 kHz */
#define ROBOT_MOTOR_TASK_PERIOD_MS       5U  /* [核心-时序] DJI 电机闭环周期，200 Hz */
#define ROBOT_HEALTH_TASK_PERIOD_MS      5U  /* [保护] 守护与任务健康检查周期 */
#define ROBOT_DIAGNOSTICS_TASK_PERIOD_MS 10U /* [固定] 文本诊断任务调度周期 */

/* 20 ms 控制任务失活时，5 ms 电机任务独立执行的命令租约。 */
#define ROBOT_MOTOR_COMMAND_TIMEOUT_MS    100U /* [保护] 应用 100 ms 未续命则电机清零 */

#if ROBOT_MOTOR_COMMAND_TIMEOUT_MS <= MAIN_LOOP_PERIOD_MS
#error "ROBOT_MOTOR_COMMAND_TIMEOUT_MS must exceed one control period"
#endif
#if ROBOT_MOTOR_COMMAND_TIMEOUT_MS >= 0x80000000UL
#error "ROBOT_MOTOR_COMMAND_TIMEOUT_MS exceeds the wrap-safe range"
#endif

/* 任务健康判定：连续超期、心跳丢失或栈余量不足均触发安全停机。 */
#define ROBOT_TASK_STARTUP_GRACE_MS           100U /* [保护] 调度启动宽限 */
#define ROBOT_TASK_HEARTBEAT_PERIODS            4U /* [保护] 允许丢失的任务周期数 */
#define ROBOT_TASK_DEADLINE_TOLERANCE_PERCENT  25U /* [保护] 单周期超期容差 */
#define ROBOT_TASK_MAX_CONSECUTIVE_OVERRUNS     3U /* [保护] 连续超期判故障次数 */
#define ROBOT_TASK_MIN_STACK_FREE_WORDS        64U /* [保护] 最低剩余栈，单位 word */
#define ROBOT_TASK_STACK_SAMPLE_INTERVAL      100U /* [固定] 栈余量抽样间隔 */

/* Only the health task refreshes IWDG; a stalled scheduler or critical task resets. */
#define ROBOT_HARDWARE_WATCHDOG_TIMEOUT_MS   1000U /* [保护] IWDG 复位时间 */

/*
 * 调试阶段安全策略：SA 使用直接电平门；单个电机掉线只隔离该电机并报警，
 * 不连带关闭其余在线执行器。遥控失联、初始化失败、INS/任务故障仍会停机。
 */
#define ROBOT_SAFETY_REQUIRE_EXPLICIT_REARM  0U /* [核心-安全] 0=SA 电平直接放行 */
#define ROBOT_SAFETY_GATE_ON_MOTOR_HEALTH    0U /* [核心-安全] 0=掉线只报警/隔离 */

#if (ROBOT_SAFETY_REQUIRE_EXPLICIT_REARM > 1U) || \
    (ROBOT_SAFETY_GATE_ON_MOTOR_HEALTH > 1U)
#error "ROBOT_SAFETY_* options must be 0 or 1"
#endif

#if ROBOT_HARDWARE_WATCHDOG_TIMEOUT_MS <= ROBOT_HEALTH_TASK_PERIOD_MS
#error "ROBOT_HARDWARE_WATCHDOG_TIMEOUT_MS must exceed one health period"
#endif

/*============================================================================
 * CAN 总线电机配置：[固定] 改接线或电调 ID 后同步修改，不能用于调速度。
 *============================================================================*/
/* 底盘电机 - CAN1, M3508 */
#define CHASSIS_CAN             hcan1
#define CHASSIS_MOTOR_FR_ID     3U /* 前右 */
#define CHASSIS_MOTOR_FL_ID     2U /* 前左 */
#define CHASSIS_MOTOR_BR_ID     4U /* 后右 */
#define CHASSIS_MOTOR_BL_ID     1U /* 后左 */

/* 云台电机 - GM6020 */
#define YAW_CAN                 hcan1
#define YAW_MOTOR_ID            1U
#define PITCH_CAN               hcan2
#define PITCH_MOTOR_ID          1U

/* 发射电机 - CAN2 */
#define FRICTION_CAN            hcan2
#define FRICTION_LEFT_ID        1U
#define FRICTION_RIGHT_ID       2U
#define LOADER_CAN              hcan2
#define LOADER_MOTOR_ID         6U

/*============================================================================
 * 遥控设备配置 - ET08/DT7/VT。这里只允许设备量程、死区和掉线时间，
 * 禁止加入 m/s、rad/s、deg/s 或机械限位。
 *============================================================================*/
#define RC_UART                   huart3 /* [固定] ET08/DT7 接收串口 */
#define ET08_ONLINE_TIMEOUT_MS      100U /* [保护] 连续无有效 SBUS 帧则掉线 */
#define DT7_ONLINE_TIMEOUT_MS       100U /* [保护] 连续无有效 DBUS 帧则掉线 */
#define ET08_STICK_DEADZONE_RAW      100 /* [标定] 中位实测最大偏移约 74 */
#define ET08_STICK_FULL_SCALE_RAW  660.0f /* [标定] 摇杆单边有效量程 */
#define DT7_STICK_DEADZONE_RAW        10 /* [标定] DT7 中位死区 */
#define DT7_STICK_FULL_SCALE_RAW   660.0f /* [标定] DT7 单边量程 */
#define VT_STICK_DEADZONE_RAW         10 /* [标定] VT 虚拟摇杆死区 */
#define VT_STICK_FULL_SCALE_RAW    660.0f /* [标定] VT 虚拟摇杆单边量程 */

/*============================================================================
 * 图传链路配置 - VT03/VT13 (USART6, 921600bps)
 *============================================================================*/
#define VT_UART                 huart6
#define VT_UART_PORT            6U
#define VT_BAUDRATE             921600U
#define VT_PROTOCOL_FRAME_SIZE  21U
#define VT_ONLINE_TIMEOUT_MS    200U    /* [保护] 图传链路掉线时间 */

/* 键盘按键位定义 */
#define VT_KEY_W                (1U << 0)
#define VT_KEY_S                (1U << 1)
#define VT_KEY_A                (1U << 2)
#define VT_KEY_D                (1U << 3)
#define VT_KEY_SHIFT            (1U << 4)
#define VT_KEY_CTRL             (1U << 5)
#define VT_KEY_Q                (1U << 6)
#define VT_KEY_E                (1U << 7)
#define VT_KEY_R                (1U << 8)
#define VT_KEY_F                (1U << 9)
#define VT_KEY_G                (1U << 10)
#define VT_KEY_Z                (1U << 11)
#define VT_KEY_X                (1U << 12)
#define VT_KEY_C                (1U << 13)
#define VT_KEY_V                (1U << 14)
#define VT_KEY_B                (1U << 15)

/* VT 键盘只生成无量纲意图；真实底盘速度仍由执行层核心参数决定。 */
#define VT_CHASSIS_BASE_INTENT  0.60f /* 普通按键为满量程的 60% */
#define VT_CHASSIS_FAST_MULT    (1.0f / VT_CHASSIS_BASE_INTENT) /* Shift 恢复 100% */
#define VT_CHASSIS_SLOW_MULT    0.50f /* Ctrl 降到普通速度的 50% */

/* VT云台灵敏度 */
#define VT_MOUSE_YAW_INTENT_PER_COUNT   0.010f /* 每个鼠标计数对应的无量纲 Yaw 意图 */
#define VT_MOUSE_PITCH_INTENT_PER_COUNT 0.010f /* 每个鼠标计数对应的无量纲 Pitch 意图 */

/*============================================================================
 * 裁判系统(只读联锁)配置
 *============================================================================*/
#define REFEREE_ENABLE                   0U    /* [核心-联锁] 0=调试阶段不依赖裁判链路 */
#define REFEREE_UART                     huart1 /* [固定] 裁判系统串口 */
#define REFEREE_CHASSIS_POWER_NOMINAL    80.0f /* [核心] 底盘功率缩放基准，W */
#define REFEREE_CHASSIS_SCALE_MIN        0.30f /* [保护] 功率限制时最小速度比例 */
#define REFEREE_HEAT_STOP_RATIO          0.95f /* [保护] 达到热量上限 95% 后禁止拨弹 */

#if REFEREE_ENABLE && MINIMAL_DEBUG_ENABLE && (MINIMAL_DEBUG_UART_PORT == 1U)
#error "Debug UART conflicts with the enabled referee link"
#endif

/*============================================================================
 * 视觉链路配置
 *
 * [核心-联锁] VISION_ENABLE 与 REFEREE_ENABLE 共用 USART1，编译期互斥。
 * 裁判系统接口本身不做任何修改；视觉启用时裁判保持关闭即可。
 *
 * 调试阶段上位机（Ubuntu）需要串口权限：
 *   sudo usermod -aG dialout $USER     # 重新登录后生效
 * 或临时：sudo chmod 666 /dev/ttyACM0 /dev/ttyUSB0
 *
 * 协议基线见 docs/vision/vision_protocol.md。
 *============================================================================*/
#ifndef VISION_ENABLE
#define VISION_ENABLE                   1U      /* [核心-联锁] 视觉通信总开关 */
#endif
#define VISION_UART_PORT                1U      /* [固定] 当前使用 USART1 */
#define VISION_BAUDRATE                 921600U /* [固定] 与上位机一致，8N1 */

/* 上位机 60~120 Hz 且非严格定频，超时需容忍抖动；选 250 ms 约 15 个周期。 */
#define VISION_ONLINE_TIMEOUT_MS        250U    /* [保护] 无有效目标帧则视觉离线 */
/* 视觉栈与机器人分开上电，需给上位机启动与相机取流留出时间。 */
#define VISION_STARTUP_GRACE_MS         3000U   /* [保护] 启动宽限，期间不判离线 */
/* 自瞄模式下按此周期打印一次链路增量计数（rx/crc/tgt/rej）。 */
#define VISION_STATUS_LOG_PERIOD_MS     1000U   /* [核心-调试] 自瞄状态打印周期 */

/* [核心-调试] Yaw 跟随方向。目标在云台系下偏左（y 为正）时云台应向左转。
 * 本车 IMU yaw 为多圈累计角、正方向待实板确认：若不接管时云台稳定向错误
 * 方向转到底，把这里改为 0U 即可，无需改动解算与测试。 */
#ifndef VISION_YAW_FOLLOW_ENABLE
#define VISION_YAW_FOLLOW_ENABLE        1U
#endif

#if (VISION_ENABLE > 1U) || (VISION_ONLINE_TIMEOUT_MS == 0U) || \
    (VISION_STARTUP_GRACE_MS == 0U) || (VISION_STATUS_LOG_PERIOD_MS == 0U) || \
    (VISION_YAW_FOLLOW_ENABLE > 1U)
#error "VISION_* options must be 1/0 and timeouts nonzero"
#endif

#if VISION_UART_PORT == 1U
#define VISION_UART_HANDLE huart1
#elif VISION_UART_PORT == 6U
#define VISION_UART_HANDLE huart6
#else
#error "Unsupported VISION_UART_PORT, use 1/6."
#endif

#if VISION_ENABLE && REFEREE_ENABLE
#error "Vision link and referee share USART1; enable only one"
#endif

#if VISION_ENABLE && (VISION_UART_PORT == MINIMAL_DEBUG_UART_PORT) && \
    MINIMAL_DEBUG_ENABLE
#error "Vision UART conflicts with the debug console"
#endif

#if VISION_ENABLE && (VISION_UART_PORT == ROBOT_TUNING_UART_PORT) && \
    ROBOT_TUNING_TELEMETRY_ENABLE
#error "Vision UART conflicts with tuning telemetry"
#endif

/* 遥控后端 ET08/DT7 固定占用 USART3（同上文 3U 的既有写法）。 */
#if VISION_ENABLE && (VISION_UART_PORT == 3U)
#error "Vision UART conflicts with the remote-control receiver"
#endif

/*============================================================================
 * 底盘参数
 *============================================================================*/
/*
 * [核心-底盘最高轮速] M3508 转子参考硬上限，单位 rad/s。
 * 当前 523.599 rad/s = 5000 rpm；四轮解算与最终等比例限速都使用该值。
 * 要整体提高/降低底盘极速，优先调整这里，并同步检查温升、电源和机械强度。
 */
#define M3508_ROTOR_SPEED_LIMIT_RAD_S 523.5987756f

/* [核心-云台速度基准] 仅用于推导手动 Yaw/Pitch 速度，不是最终电流限幅。 */
#define GM6020_SPEED_LIMIT_DEG_S      3600.0f
/* [保护] GM6020 最终 CAN 命令绝对值校验上限；不是角速度。 */
#define GM6020_COMMAND_LIMIT          30000.0f

/* [标定] 全向轮有效滚动半径与左右轮接地点间距，单位 m。 */
#define CHASSIS_WHEEL_RADIUS          0.075f
#define CHASSIS_WHEEL_BASE            0.34f

/* [标定] 标准 M3508 减速箱电机转子/轮轴减速比；更换传动后必须实测。 */
#define CHASSIS_MOTOR_REDUCTION_RATIO 19.20320856f

/*
 * [派生-禁止单独填写]
 * 满意图直接映射到 M3508 配置转速上限，不再设置一层“遥控速度”。
 * 当前最大平移约 2.045 m/s，理论纯旋转约 12.029 rad/s；修改轮径、轮距、
 * 减速比或电机上限后自动变化。平移+旋转超限时四轮在执行层统一缩放。
 */
#define CHASSIS_MAX_TRANSLATION_SPEED \
    (M3508_ROTOR_SPEED_LIMIT_RAD_S * CHASSIS_WHEEL_RADIUS / \
     CHASSIS_MOTOR_REDUCTION_RATIO)
#define CHASSIS_MAX_ROTATION_SPEED_RAD_S \
    (CHASSIS_MAX_TRANSLATION_SPEED / (CHASSIS_WHEEL_BASE * 0.5f))

#define CHASSIS_MOTOR_SPEED_DEADZONE_RAD_S 2.09439510f /* [保护] 非跟随模式 20 rpm 以下清零 */
#define CHASSIS_DEADZONE_VX      0.08f /* [核心-手感] 车体横移命令死区，m/s */
#define CHASSIS_DEADZONE_VY      0.08f /* [核心-手感] 车体前进命令死区，m/s */
#define CHASSIS_DEADZONE_WZ      0.10f /* [核心-手感] 车体旋转命令死区，rad/s */
/* [核心-手感] 一阶低通旧值权重：0=无滤波，越接近 1 越平滑也越迟钝。 */
#define CHASSIS_SPEED_FILTER_COEF 0.15f

/*
 * [PID-底盘轮速] 四个 M3508 转子速度串级电流环。
 * 速度参考/反馈/误差单位均为 rad/s，输出为 C620 电流命令；MAX_OUT 越小越软。
 */
#define CHASSIS_SPEED_KP        229.18312f /* 速度误差的即时电流增益 */
#define CHASSIS_SPEED_KI        2.291831f  /* 消除带载静差，过大将低频振荡 */
#define CHASSIS_SPEED_KD        0.0f       /* 当前关闭，速度反馈噪声大时慎用 */
#define CHASSIS_SPEED_MAX_OUT   15000.0f   /* [保护] 速度 PID 输出限幅，C620 命令 */

/* [固定] 底盘正式路径始终使用速度环；不要通过遥控模式切换闭环拓扑。 */
#define CHASSIS_INIT_LOOP              SPEED_LOOP
#define CHASSIS_RUN_LOOP_STOP          SPEED_LOOP
#define CHASSIS_RUN_LOOP_NORMAL        SPEED_LOOP

/* [标定] 云台机械正前方对应的 Yaw 原始编码器值；本车 2026-07-20 实测为 7821。 */
#define YAW_CHASSIS_ALIGN_ECD          7821U
#define YAW_ALIGN_ANGLE_DEG            (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) /* [派生] */
/* [标定] IMU 累计 Yaw 的逻辑零位；仅在对应标定逻辑中使用。 */
#define IMU_YAW_LOGIC_ZERO_TOTAL_DEG   364.30f
/* [标定] 编码器正前方后的亚刻度逻辑修正，通常保持 0。 */
#define YAW_OFFSET_LOGIC_ZERO_DEG      0.0f

/*
 * [PID-底盘跟随] Yaw 相对底盘角误差(rad) -> 底盘角速度(rad/s)。
 * Kp 决定追赶强度；Kd 抑制超调；Ki 当前关闭；MAX 只限制跟随外环，不限制小陀螺。
 */
#define CHASSIS_FOLLOW_WZ_KP           30.0f
#define CHASSIS_FOLLOW_WZ_KI           0.0f
#define CHASSIS_FOLLOW_WZ_KD           0.80f
#define CHASSIS_FOLLOW_WZ_MAX          10.0f       /* [核心] 跟随追赶最大角速度，rad/s */
#define CHASSIS_FOLLOW_WZ_I_MAX_RAD_S  0.26179939f /* [保护] I 项最大贡献，rad/s */
/* [保护] 跟随接近正姿态时仅清除小于约 3.33 rpm 的轮电机参考，避免死区过大。 */
#define CHASSIS_FOLLOW_MOTOR_SPEED_DEADZONE_RAD_S 0.34906585f

/* [核心-小陀螺转速] 占理论纯旋转能力的比例；当前 0.60 -> 约 7.218 rad/s。 */
#define CHASSIS_SPIN_SPEED_RATIO       0.60f
/* [派生] 实际小陀螺角速度；提高比例会减少留给平移的轮速余量。 */
#define CHASSIS_SPIN_SPEED_RAD_S       \
    (CHASSIS_MAX_ROTATION_SPEED_RAD_S * CHASSIS_SPIN_SPEED_RATIO)

/*============================================================================
 * 云台参数
 *============================================================================*/
/*
 * [核心-云台手动速度] 满摇杆对应的执行层速度/目标推进速率。
 * 当前 Yaw=828 deg/s，Pitch=540 deg/s；遥控层只提供 [-1,1] 意图，不再二次限速。
 */
#define GIMBAL_YAW_MAX_SPEED_DEG_S      (GM6020_SPEED_LIMIT_DEG_S * 0.23f)
#define GIMBAL_PITCH_MAX_SPEED_DEG_S    (GM6020_SPEED_LIMIT_DEG_S * 0.15f)

/* [核心-机械安全] IMU Pitch 软限位：负角抬头、正角低头，对所有控制来源统一生效。 */
#define GIMBAL_PITCH_MIN_DEG             -30.0f /* 最大抬头 30 deg */
#define GIMBAL_PITCH_MAX_DEG              20.0f /* 最大低头 20 deg */

/* [标定] 本车正控制轴输出需经驱动反向后才产生正 IMU Pitch；不是遥控方向参数。 */
#define PITCH_MOTOR_OUTPUT_REVERSED         1U

/* [核心-Pitch 归中] 上电/安全门重新放行后的 IMU 角目标，当前回到 0 deg。 */
#define PITCH_STARTUP_CENTER_DEG             0.0f

/* [核心-Yaw 状态切换] 松杆后先用速度环刹停，再锁定当前 IMU 累计角。 */
#define YAW_BRAKE_SPEED_EPS     10.0f /* 进入稳定计数的角速度阈值，deg/s */
#define YAW_BRAKE_STABLE_COUNT  10U   /* 连续满足次数；20 ms 主循环下约 200 ms */
#define YAW_BRAKE_TIMEOUT_MS    220U  /* [保护] 最迟切回角度保持的时间 */

/* [保护] 控制任务异常延迟时，Pitch 单次目标积分最多按 40 ms 计算。 */
#define PITCH_TARGET_INTEGRATION_MAX_DT_MS 40U

/*============================================================================
 * 云台自瞄接管参数
 *
 * 视觉下线角度目标是"角速度命令"而非绝对角，因此这里只限制视觉角度参考
 * 相对当前姿态的单周期最大推进量。取值为手动满摇杆同量级再放宽约 3 倍：
 * 既能跟上目标阶跃，又不会因目标跳变或跟踪器重收敛把云台猛打。
 *============================================================================*/
#define VISION_MAX_RATE_DEG_S          2800.0f /* [核心-调试] 角度参考最大推进速率 */
#define VISION_MAX_RATE_PITCH_DEG_S    1200.0f /* [核心-调试] Pitch 推进速率上限 */

/* [保护] 目标水平距离下限：低于该值方位角与仰角无定义，放弃该帧。 */
#define VISION_MIN_HORIZONTAL_DIST_M   0.10f

/*
 * [核心-调试] 自瞄所需的上位机侧参数。裁判系统启用后应改为从比赛信息读取，
 * 当前调试阶段先用常量，保证视觉在无裁判链路时也能独立跑通。
 */
#define ROBOT_ENEMY_COLOR               0U      /* 0=红方 1=蓝方 */
#define ROBOT_BULLET_SPEED_MPS          15U     /* 弹速 m/s，需实测标定 */

/*
 * [PID-Yaw 角度外环] IMU 角误差(deg) -> 速度参考(deg/s)。
 * 只在松杆后的 ANGLE 保持态运行，不决定手动摇杆转速。
 */
#define YAW_ANGLE_KP            15.0f
#define YAW_ANGLE_KI            0.00f
#define YAW_ANGLE_KD            2.00f
#define YAW_ANGLE_MAX_OUT       5000.0f /* [保护] 外环速度参考绝对限幅，deg/s */

/* [PID-Yaw 速度内环] IMU GyroZ 误差(deg/s) -> GM6020 电流命令。 */
#define YAW_SPEED_KP            18.0f
#define YAW_SPEED_KI            10.0f
#define YAW_SPEED_KD            0.0f
#define YAW_SPEED_MAX_OUT       15000.0f /* [保护] 速度 PID 总输出限幅 */
#define YAW_SPEED_I_MAX         4000.0f  /* [保护] 速度积分项限幅 */

/* [核心-小陀螺 Yaw 前馈] 底盘基座角速度(rad/s) -> GM6020 电流补偿。 */
#define YAW_BASE_RATE_CURRENT_FF_K          900.0f   /* 前馈增益；过大将顺逆向不对称 */
#define YAW_BASE_RATE_CURRENT_FF_MAX        10000.0f /* [保护] 前馈绝对限幅 */
#define YAW_BASE_RATE_FF_LPF                0.85f    /* 旧值权重，越大越平滑/滞后 */
#define YAW_BASE_RATE_FF_DEADBAND_RAD_S     0.20f    /* 小于该基座角速度不补偿 */

/* [PID-Pitch 角度外环] IMU Pitch 误差(deg) -> 速度参考(deg/s)，始终运行。 */
#define PITCH_ANGLE_KP          100.0f
#define PITCH_ANGLE_KI          0.0f
#define PITCH_ANGLE_KD          0.50f
#define PITCH_ANGLE_MAX_OUT     5000.0f /* [保护] 外环速度参考绝对限幅，deg/s */

/* [PID-Pitch 速度内环] IMU GyroX 误差(deg/s) -> GM6020 电流命令。 */
#define PITCH_SPEED_KP          10.0f
#define PITCH_SPEED_KI          15.0f
#define PITCH_SPEED_KD          0.0f
#define PITCH_SPEED_MAX_OUT     15000.0f /* [保护] 速度 PID 总输出限幅 */
#define PITCH_SPEED_I_MAX       5000.0f  /* [保护] 速度积分项限幅 */

/* Pitch重力补偿
 * 约定:
 * - 控制坐标统一为 IMU Pitch 正方向；
 * - 本车最终电机执行输出反向，因此保持本车水平姿态需要负的控制轴前馈；
 * - 当前 K=0 是去掉重力补偿后的临时采样配置，不代表最终标定值。
 */
#define PITCH_GRAVITY_FF_K             (-0.0f) /* [核心-待标定] cos 模型峰值电流系数 */
#define PITCH_GRAVITY_FF_MAX           4000.0f /* [保护] 重力前馈绝对限幅 */
#define PITCH_GRAVITY_HORIZONTAL_DEG   0.0f    /* [标定] 炮管机械水平时 IMU Pitch */
#define PITCH_FF_LPF                   0.8f    /* 旧值权重，越大越平滑/滞后 */

/* [预留] 旧分离模式位置目标范围；正式 FreeRTOS 云台路径当前未使用。 */
#define GIMBAL_SEPARATE_YAW_MAX_ANGLE    1080.0f

/*============================================================================
 * 发射参数
 *============================================================================*/
/*
 * [核心-摩擦轮转速] M3508 直驱轮目标，单位转子 rad/s。
 * 当前 -523.599 rad/s = -5000 rpm；绝对值决定弹速，负号只决定本车安装方向。
 * 左右轮共用同一逻辑目标，右轮由电机反向配置得到相反物理转向。
 */
#define FRICTION_TARGET_SPEED_RAD_S      (-523.5987756f)
/* [核心-启动手感] 每 ms 最大目标变化量；当前约 698 ms 从 0 升到 5000 rpm。 */
#define FRICTION_SPEED_SLEW_RAD_S_PER_MS 0.75f

/* [保护-摩擦轮就绪] 命令达到 98%，且双轮速度误差不超过 8% 持续 150 ms 才允许拨弹。 */
#define FRICTION_READY_COMMAND_RATIO     0.98f
#define FRICTION_READY_ERROR_RATIO       0.08f
#define FRICTION_READY_HOLD_MS           150U
/* [保护] 已就绪后误差超过 25% 持续 60 ms 才撤销，避免单帧扰动误停。 */
#define FRICTION_READY_DROP_ERROR_RATIO  0.25f
#define FRICTION_READY_DROP_HOLD_MS      60U

/* [标定] M2006 P36；编码器在电机侧，八槽拨盘每发输出轴转过 45 deg。 */
#define LOADER_GEAR_RATIO                 36.0f /* 电机转子/拨盘输出轴减速比 */
#define LOADER_OUTPUT_STEP_DEG            45.0f /* 八槽拨盘每发的输出轴角度 */
#define LOADER_DIRECTION_SIGN             1.0f /* [标定] +1/-1 决定正向拨弹方向 */
/* [派生] 每发电机侧角度：1 * 45 * 36 = 1620 deg，禁止直接写魔数。 */
#define LOADER_ANGLE_STEP                 \
    (LOADER_DIRECTION_SIGN * LOADER_OUTPUT_STEP_DEG * LOADER_GEAR_RATIO)

/* [核心-连发射频] 当前 5 发/s；这是调整连发快慢的首选参数。 */
#define LOADER_CONTINUOUS_SHOTS_PER_S      5.0f
/* [派生] 电机侧 8100 deg/s = 1350 rpm；输出轴 37.5 rpm = 5 发/s。 */
#define LOADER_CONTINUOUS_SPEED_DEG_S      \
    (LOADER_ANGLE_STEP * LOADER_CONTINUOUS_SHOTS_PER_S)
/* [核心-连发启动] 每 ms 最大目标变化 10 deg/s，约 810 ms 升到 5 发/s。 */
#define LOADER_CONTINUOUS_SLEW_PER_MS      10.0f

/* [核心-单发速度] 位置目标按 5000 deg/s 推进，理论 324 ms 完成 1620 deg。 */
#define LOADER_SINGLE_REF_SLEW_DEG_PER_MS  5.0f
/* [保护-单发完成判据] 目标到位、角误差和转速同时满足后持续 60 ms 才算完成。 */
#define LOADER_SINGLE_SETTLE_EPS_DEG       15.0f  /* 电机侧角误差，deg */
#define LOADER_SINGLE_SETTLE_SPEED_DEG_S   180.0f /* 电机侧速度绝对值，deg/s=30 rpm */
#define LOADER_SINGLE_SETTLE_MS            60U
/* [保护] 完成后继续位置保持 150 ms；总动作超过 1 s 判超时并停机。 */
#define LOADER_SINGLE_HOLD_MS              150U
#define LOADER_SINGLE_TIMEOUT_MS           1000U
/* [核心-语义] 忙时最多缓存一发；SD 保持上位由执行层锁存，不会重复入队。 */
#define LOADER_SINGLE_QUEUE_LIMIT          1U

/*
 * [保护-连发卡弹] 命令超过目标 60%、实际低于 60 rpm、输出超过限幅 80%，
 * 连续 120 ms 才确认卡弹；随后按正常连发速度反转 120 ms，最多重试两次。
 */
#define LOADER_JAM_DETECT_COMMAND_RATIO     0.60f
#define LOADER_JAM_SPEED_THRESHOLD_DEG_S   360.0f
#define LOADER_JAM_OUTPUT_RATIO             0.80f
#define LOADER_JAM_DETECT_MS                120U
#define LOADER_JAM_REVERSE_SPEED_DEG_S      (-LOADER_CONTINUOUS_SPEED_DEG_S) /* [派生] */
#define LOADER_JAM_REVERSE_MS               120U
#define LOADER_JAM_MAX_RETRIES              2U

#if FRICTION_READY_HOLD_MS <= MAIN_LOOP_PERIOD_MS
#error "FRICTION_READY_HOLD_MS must span multiple control updates"
#endif
#if LOADER_JAM_DETECT_MS <= MAIN_LOOP_PERIOD_MS
#error "LOADER_JAM_DETECT_MS must span multiple control updates"
#endif
#if LOADER_JAM_REVERSE_MS <= MAIN_LOOP_PERIOD_MS
#error "LOADER_JAM_REVERSE_MS must span multiple control updates"
#endif
#if LOADER_JAM_MAX_RETRIES == 0U
#error "LOADER_JAM_MAX_RETRIES must allow at least one recovery"
#endif

/* [PID-摩擦轮速度] 误差单位 rad/s，输出为 C620 电流命令；当前纯 P 已实机稳定。 */
#define FRICTION_SPEED_KP       47.74648f /* 等价 5 command/rpm */
#define FRICTION_SPEED_KI       0.0f     /* 当前关闭；只在确认稳定静差后小步增加 */
#define FRICTION_SPEED_KD       0.0f     /* 当前关闭，避免放大测速噪声 */
#define FRICTION_SPEED_MAX_OUT  8000.0f  /* [保护] 约为 C620 合法命令范围的 49% */

/* [PID-拨弹速度] 误差单位电机侧 deg/s，输出为 C610 电流命令。 */
#define LOADER_SPEED_KP         1.666667f /* 等价 10 command/rpm */
#define LOADER_SPEED_KI         0.0f      /* 当前关闭；连发存在约 2% 带载静差 */
#define LOADER_SPEED_KD         0.0f      /* 当前关闭 */
#define LOADER_SPEED_MAX_OUT    4000.0f   /* [保护] C610 最大命令的 40% */
#define LOADER_SPEED_I_MAX      2000.0f   /* [保护] 启用 Ki 后的积分项限幅 */

/* [PID-拨弹位置外环] 电机侧角误差(deg) -> 速度参考(deg/s)，只用于单发。 */
#define LOADER_ANGLE_KP         3.5f
#define LOADER_ANGLE_KI         0.0f
#define LOADER_ANGLE_KD         0.0f
#define LOADER_ANGLE_MAX_OUT    6000.0f /* [保护] 单发内环速度参考限幅，deg/s */

/* [固定] 摩擦轮只跑速度环；拨弹单发跑角度->速度串级，连发只跑速度环。 */
#define FRICTION_INIT_LOOP               SPEED_LOOP
#define FRICTION_RUN_LOOP_STOP           SPEED_LOOP
#define FRICTION_RUN_LOOP_ON             SPEED_LOOP
#define LOADER_INIT_LOOP                 SPEED_LOOP
#define LOADER_RUN_LOOP_STOP             SPEED_LOOP
#define LOADER_RUN_LOOP_SINGLE           ANGLE_LOOP
#define LOADER_RUN_LOOP_CONTINUOUS       SPEED_LOOP

#endif /* ROBOT_CONFIG_H */
