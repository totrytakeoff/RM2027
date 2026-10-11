/**
 * @file main.c
 * @brief 电机测试固件（tests/firmware/basic/motor_test）
 *
 * 你可以把它当成“电机调试专用程序”：
 * - 主循环里持续跑 `DJIMotorControl()` / `DaemonTask()` / `HAL_Delay()`，保证底层任务在运行；
 * - 具体要让电机怎么动，由你选择调用哪个 `MotorTest_*()` 函数；
 * - 为了方便“在线改参 + 观察效果”，这里额外提供了一组 `volatile` 全局变量，
 *   你可以在 GDB 里 `set variable` 动态切换 demo 和调整 MIT 参数。
 *
 * ---------------------------- 如何在 main 中调用 ----------------------------
 * 1) 选择 DM MIT demo（推荐）：
 *    - 将 `g_dm_demo_select` 设为 1（定速）或 2（定时 60° 阶跃），主循环会自动持续发送 MIT 帧。
 * 2) 也可以直接在 while(1) 里调用：
 *    - 例如：`MotorTest_DM_MIT_PeriodicAngleStep(60.0f, 1000);`
 *
 * ---------------------------- 如何在 GDB 中调用/调参 ----------------------------
 * 注意：DM 电机 MIT 模式需要“持续发送”（通常 1kHz，至少几百 Hz）才能维持控制。
 * 所以不建议在 GDB 里只 `call MotorTest_*()` 一次，因为那只会发一帧，电机很快超时失能。
 *
 * 推荐做法（在 GDB 里“改变量”，让主循环持续调用）：
 *   (gdb) set variable g_dm_demo_select = 1
 *   (gdb) set variable g_dm_target_speed_rad_s = 10
 *   (gdb) set variable g_dm_kd_speed = 0.6
 *
 * 切换到“每隔 1s 转 60°”：
 *   (gdb) set variable g_dm_demo_select = 2
 *   (gdb) set variable g_dm_step_deg = 60
 *   (gdb) set variable g_dm_step_interval_ms = 1000
 *   (gdb) set variable g_dm_kp_step = 1
 *   (gdb) set variable g_dm_kd_step = 0.2
 *
 * 关闭输出/停转：
 *   (gdb) set variable g_dm_demo_select = 0
 *   (gdb) call MotorTest_StopAll()
 */
#include "main.h"

#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include "bsp_init.h"
#include "bsp_log.h"
#include "daemon.h"
#include "dji_motor.h"
#include "dmmotor.h"

#define M3508_MOTOR_COUNT 4U
static const uint8_t M3508_CAN_IDS[M3508_MOTOR_COUNT] = {1U, 2U, 3U, 4U};
#define GM6020_CAN_ID 5U




#define M3508_SPEED_MAX 360.0f // 360 deg/s , 1 turn per second
#define M3508_SPEED_MIN (-M3508_SPEED_MAX)
#define M3508_ANGLE_MAX 36000.0f  // deg, allow ~100 turns for demos
#define M3508_ANGLE_MIN (-M3508_ANGLE_MAX)

#define GM6020_SPEED_MAX 3600.0f  // deg/s, conservative limit
#define GM6020_SPEED_MIN (-GM6020_SPEED_MAX)
#define GM6020_ANGLE_MAX 1440.0f  // deg
#define GM6020_ANGLE_MIN (-GM6020_ANGLE_MAX)

/* ---------------- DM8009P (MIT) demo config ----------------
 * 上位机读到的通信参数：
 * - Motor CAN ID = 0x01
 * - Master ID = 0x00
 *
 * 说明：框架 CAN 底层是“严格过滤器”，实际接收过滤的 StdId 为：
 *   feedback_std_id = (master_id<<4) | (motor_id & 0x0F)
 * 所以本配置下反馈 StdId=0x01（与 dm_demo_pio 的结论一致）。
 */
#define DM_MOTOR_ID 0x01U
#define DM_MASTER_ID 0x00U

/* 与上位机截图一致的映射范围（MIT 打包/解包要一致） */
#define DM_P_RANGE 12.5f
#define DM_V_RANGE 45.0f
#define DM_T_RANGE 54.0f

/* Demo 1：定速旋转（MIT：kp=0，仅用 kd 让 v_des 生效） */
#define DM_DEMO_SPEED_RAD_S 6.0f
#define DM_DEMO_SPEED_KP 0.0f
#define DM_DEMO_SPEED_KD 0.5f
#define DM_DEMO_SPEED_TFF 0.0f

/* Demo 2：定时转动指定角度（MIT：位置阶跃） */
#define DM_DEMO_STEP_DEG 30.0f
#define DM_DEMO_STEP_INTERVAL_MS 2000U
#define DM_DEMO_STEP_KP 1.0f
#define DM_DEMO_STEP_KD 0.2f
#define DM_DEMO_STEP_TFF 0.0f

static DJIMotorInstance* m3508_motors[M3508_MOTOR_COUNT] = {
        NULL};                                 // Lazily created M3508 instances
static DJIMotorInstance* loader_motor = NULL;  // M3508 loader motor on a different CAN group
static DJIMotorInstance* gm6020_motor = NULL;  // Lazily created GM6020 instance
static DMMotor_Handle* dm_motor = NULL;
static uint8_t dm_mit_enabled = 0;
static uint32_t dm_last_enable_tick = 0;

/* ---------------- DM demo 运行时参数（GDB 可直接改） ----------------
 * g_dm_demo_select:
 *   0 = 不输出（默认）
 *   1 = MIT 定速（p=0, v=target_speed, kp=0, kd 可调）
 *   2 = MIT 位置阶跃（每隔 interval 加 step_deg，kp/kd 可调）
 */
volatile uint8_t g_dm_demo_select = 2;
volatile float g_dm_target_speed_rad_s = DM_DEMO_SPEED_RAD_S;
volatile float g_dm_kp_speed = DM_DEMO_SPEED_KP;
volatile float g_dm_kd_speed = DM_DEMO_SPEED_KD;
volatile float g_dm_tff_speed = DM_DEMO_SPEED_TFF;

volatile float g_dm_step_deg = DM_DEMO_STEP_DEG;
volatile uint32_t g_dm_step_interval_ms = DM_DEMO_STEP_INTERVAL_MS;
volatile float g_dm_kp_step = DM_DEMO_STEP_KP;
volatile float g_dm_kd_step = DM_DEMO_STEP_KD;
volatile float g_dm_tff_step = DM_DEMO_STEP_TFF;

void SystemClock_Config(void);
static void EnsureM3508MotorReady(void);
static void EnsureLoaderMotorReady(void);
static void EnsureGM6020MotorReady(void);
static void EnsureDM8009PMotorReady(void);
static float ClampFloat(float value, float min, float max);

void MotorTest_StopAll(void);
void MotorTest_M3508_SpeedLoop(float target_speed_deg_s);
void MotorTest_M3508_PositionLoop(float target_angle_deg);
void MotorTest_M3508_PeriodicAngleStep(float step_deg, uint32_t interval_ms);
void MotorTest_Loader_SpeedLoop(float target_speed_deg_s);
void MotorTest_Loader_PositionLoop(float target_angle_deg);
void MotorTest_GM6020_SpeedLoop(float target_speed_deg_s);
void MotorTest_GM6020_PositionLoop(float target_angle_deg);
void MotorTest_GM6020_PeriodicAngleStep(float step_deg, uint32_t interval_ms);
void MotorTest_DM_MIT_ConstantSpeed(float target_speed_rad_s);
void MotorTest_DM_MIT_PeriodicAngleStep(float step_deg, uint32_t interval_ms);

/**
 * @brief Bare-metal entry: initialise HAL/BSP, bring up CAN1 and continuously
 *        pump the motor + daemon tasks so the test helpers can be invoked from
 *        a debugger or GDB.
 */
int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_USART6_UART_Init();

    BSPInit();
    LOGINFO("[motor_test] core init finished");
    HAL_UART_Transmit(&huart6, (uint8_t*)"---\n", 3, 1000);

      MotorTest_M3508_SpeedLoop(7200.0f);
    //   MotorTest_Loader_SpeedLoop(7200.0f);

    while (1) {
        /* 通过 g_dm_demo_select 选择 demo（推荐 GDB 改变量方式） */
        // if (g_dm_demo_select == 1) {
        //     MotorTest_DM_MIT_ConstantSpeed(g_dm_target_speed_rad_s);
        // } else if (g_dm_demo_select == 2) {
        //     MotorTest_DM_MIT_PeriodicAngleStep(g_dm_step_deg, g_dm_step_interval_ms);
        // }

        // MotorTest_GM6020_SpeedLoop(7200.0f);
        // MotorTest_M3508_PeriodicAngleStep(60.0f, 1000U);
        // MotorTest_GM6020_PeriodicAngleStep(30.0f, 1000U);

        DJIMotorControl();
        DaemonTask();
        /* 2ms ≈ 500Hz：足够让 MIT “持续发送”维持使能；如需更顺滑可改小（注意 CPU 占用） */
        HAL_Delay(2);
    }
}

/**
 * @brief Force every registered motor into stop mode. Useful when finishing a
 *        test or recovering from an unexpected state.
 */
void MotorTest_StopAll(void) {
    for (uint8_t i = 0; i < M3508_MOTOR_COUNT; ++i) {
        if (m3508_motors[i] != NULL) {
            DJIMotorStop(m3508_motors[i]);
        }
    }
    if (loader_motor != NULL) {
        DJIMotorStop(loader_motor);
    }
    if (gm6020_motor != NULL) {
        DJIMotorStop(gm6020_motor);
    }
    if (dm_motor != NULL) {
        DMMotor_Disable(dm_motor, DM_MODE_MIT);
        dm_mit_enabled = 0;
    }
    LOGINFO("[motor_test] all motors stopped");
}

/**
 * @brief Run the M3508 in a speed-loop (speed+current) on CAN1.
 *
 * @param target_speed_deg_s Desired speed in deg/s. The helper clamps it to a
 *                           safe range before forwarding to the motor module.
 */
void MotorTest_M3508_SpeedLoop(float target_speed_deg_s) {
    EnsureM3508MotorReady();
    target_speed_deg_s = ClampFloat(target_speed_deg_s, M3508_SPEED_MIN, M3508_SPEED_MAX);
    for (uint8_t i = 0; i < M3508_MOTOR_COUNT; ++i) {
        if (m3508_motors[i] == NULL) {
            continue;
        }
        DJIMotorOuterLoop(m3508_motors[i], SPEED_LOOP);
        DJIMotorEnable(m3508_motors[i]);
        DJIMotorSetRef(m3508_motors[i], target_speed_deg_s);
    }
    LOGINFO("[motor_test] %u x M3508 speed ref %d deg/s", (unsigned)M3508_MOTOR_COUNT,
            (int)target_speed_deg_s);
}

/**
 * @brief Run the M3508 in a cascaded position loop (angle -> speed -> current).
 *
 * @param target_angle_deg Desired total angle. Uses the module’s total-angle
 *                         accumulator so you can step multiple turns.
 */
void MotorTest_M3508_PositionLoop(float target_angle_deg) {
    EnsureM3508MotorReady();
    target_angle_deg = ClampFloat(target_angle_deg, M3508_ANGLE_MIN, M3508_ANGLE_MAX);
    for (uint8_t i = 0; i < M3508_MOTOR_COUNT; ++i) {
        if (m3508_motors[i] == NULL) {
            continue;
        }
        DJIMotorOuterLoop(m3508_motors[i], ANGLE_LOOP);
        DJIMotorEnable(m3508_motors[i]);
        DJIMotorSetRef(m3508_motors[i], target_angle_deg);
    }
    LOGINFO("[motor_test] %u x M3508 angle ref %d deg", (unsigned)M3508_MOTOR_COUNT,
            (int)target_angle_deg);
}

/**
 * @brief Run the loader M3508 (id 7, different CAN group) in speed loop.
 */
void MotorTest_Loader_SpeedLoop(float target_speed_deg_s) {
    EnsureLoaderMotorReady();
    target_speed_deg_s = ClampFloat(target_speed_deg_s, M3508_SPEED_MIN, M3508_SPEED_MAX);
    DJIMotorOuterLoop(loader_motor, SPEED_LOOP);
    DJIMotorEnable(loader_motor);
    DJIMotorSetRef(loader_motor, target_speed_deg_s);
    LOGINFO("[motor_test] Loader M3508 speed ref %d deg/s", (int)target_speed_deg_s);
}

/**
 * @brief Run the loader M3508 (id 7, different CAN group) in angle loop.
 */
void MotorTest_Loader_PositionLoop(float target_angle_deg) {
    EnsureLoaderMotorReady();
    target_angle_deg = ClampFloat(target_angle_deg, M3508_ANGLE_MIN, M3508_ANGLE_MAX);
    DJIMotorOuterLoop(loader_motor, ANGLE_LOOP);
    DJIMotorEnable(loader_motor);
    DJIMotorSetRef(loader_motor, target_angle_deg);
    LOGINFO("[motor_test] Loader M3508 angle ref %d deg", (int)target_angle_deg);
}

/**
 * @brief Demo helper: periodically increase the M3508角度参考值，每隔 interval_ms 毫秒累加
 * step_deg。 用于验证减速箱电机按恒定角速度(例如1s 60°)运行时的表现。
 *
 * @param step_deg    Angle increment (deg) applied every period. Positive values rotate forward.
 * @param interval_ms Interval between increments in milliseconds.
 */
void MotorTest_M3508_PeriodicAngleStep(float step_deg, uint32_t interval_ms) {
    EnsureM3508MotorReady();

    static uint8_t step_state_initialized[M3508_MOTOR_COUNT] = {0};
    static float current_target[M3508_MOTOR_COUNT] = {0.0f};
    static uint32_t last_step_tick[M3508_MOTOR_COUNT] = {0};

    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < M3508_MOTOR_COUNT; ++i) {
        DJIMotorInstance* motor = m3508_motors[i];
        if (motor == NULL) {
            continue;
        }

        if (!step_state_initialized[i]) {
            current_target[i] =
                    ClampFloat(motor->measure.total_angle, M3508_ANGLE_MIN, M3508_ANGLE_MAX);
            last_step_tick[i] = now;
            step_state_initialized[i] = 1;
            LOGINFO("[motor_test] M3508[%d] step demo start angle %d deg", (int)M3508_CAN_IDS[i],
                    (int)current_target[i]);
        }

        uint32_t elapsed = now - last_step_tick[i];
        if (interval_ms != 0U && elapsed >= interval_ms) {
            uint32_t steps = elapsed / interval_ms;
            last_step_tick[i] += steps * interval_ms;
            current_target[i] += step_deg * (float)steps;
            current_target[i] = ClampFloat(current_target[i], M3508_ANGLE_MIN, M3508_ANGLE_MAX);
            LOGINFO("[motor_test] M3508[%d] step target -> %d deg", (int)M3508_CAN_IDS[i],
                    (int)current_target[i]);
        }

        DJIMotorOuterLoop(motor, ANGLE_LOOP);
        DJIMotorEnable(motor);
        DJIMotorSetRef(motor, current_target[i]);
    }
}

/**
 * @brief GM6020 角度阶跃：类似于 3508 的 demo，每隔 interval_ms ms 将位置参考累加 step_deg。
 *
 * @param step_deg    每次递增角度（deg）。
 * @param interval_ms 每次递增之间的间隔（ms）。
 */
void MotorTest_GM6020_PeriodicAngleStep(float step_deg, uint32_t interval_ms) {
    EnsureGM6020MotorReady();

    static uint8_t step_state_initialized = 0;
    static float current_target = 0.0f;
    static uint32_t last_step_tick = 0;

    uint32_t now = HAL_GetTick();
    if (!step_state_initialized) {
        current_target = gm6020_motor->measure.total_angle;
        last_step_tick = now;
        step_state_initialized = 1;
        LOGINFO("[motor_test] GM6020 step demo start angle %d deg", (int)current_target);
    }

    uint32_t elapsed = now - last_step_tick;
    if (interval_ms != 0U && elapsed >= interval_ms) {
        uint32_t steps = elapsed / interval_ms;
        last_step_tick += steps * interval_ms;
        current_target += step_deg * (float)steps;
        current_target = ClampFloat(current_target, GM6020_ANGLE_MIN, GM6020_ANGLE_MAX);
        LOGINFO("[motor_test] GM6020 target -> %d deg", (int)current_target);
    }

    MotorTest_GM6020_PositionLoop(current_target);
}

/**
 * @brief DM8009P MIT 定速旋转 Demo（不做任何外部环路）
 *
 * 做法：
 * - kp=0：不做位置刚度
 * - v_des=目标速度
 * - kd>0：提供速度误差“阻尼”，使 v_des 生效（可理解为简单速度 P 控制）
 *
 * 你只需要调宏：
 * - `DM_DEMO_SPEED_KD`、`DM_DEMO_SPEED_TFF`
 */
void MotorTest_DM_MIT_ConstantSpeed(float target_speed_rad_s) {
    EnsureDM8009PMotorReady();
    const DMMotor_Feedback* fb = DMMotor_GetFeedback(dm_motor);
    uint32_t now = HAL_GetTick();

    if (!dm_mit_enabled) {
        DMMotor_ClearError(dm_motor, DM_MODE_MIT);
        DMMotor_Enable(dm_motor, DM_MODE_MIT);
        dm_mit_enabled = 1;
        dm_last_enable_tick = now;
    }

    if (fb && fb->error_state != 0) {
        LOGWARNING("[motor_test] DM error state=0x%02x, try clear+enable", fb->error_state);
        DMMotor_ClearError(dm_motor, DM_MODE_MIT);
        DMMotor_Enable(dm_motor, DM_MODE_MIT);
        dm_last_enable_tick = now;
    }

    if (now - dm_last_enable_tick > 100) {
        DMMotor_Enable(dm_motor, DM_MODE_MIT);
        dm_last_enable_tick = now;
    }

    const float p_des = 0.0f; /* kp=0 时 p_des 不参与 */
    DMMotor_SendMIT(dm_motor, p_des, target_speed_rad_s, g_dm_kp_speed, g_dm_kd_speed,
                    g_dm_tff_speed);
}

/**
 * @brief DM8009P MIT 定时转动指定角度 Demo（每隔 interval_ms 将目标位置累加 step_deg）
 *
 * 说明：
 * - MIT 的 p_des 单位是 rad，且受 `DM_P_RANGE` 限制；这里采用回绕避免饱和。
 * - 参数调节：主要动 `DM_DEMO_STEP_KP/DM_DEMO_STEP_KD/DM_DEMO_STEP_TFF`
 */
void MotorTest_DM_MIT_PeriodicAngleStep(float step_deg, uint32_t interval_ms) {
    EnsureDM8009PMotorReady();
    const DMMotor_Feedback* fb = DMMotor_GetFeedback(dm_motor);
    uint32_t now = HAL_GetTick();

    if (!dm_mit_enabled) {
        DMMotor_ClearError(dm_motor, DM_MODE_MIT);
        DMMotor_Enable(dm_motor, DM_MODE_MIT);
        dm_mit_enabled = 1;
        dm_last_enable_tick = now;
    }
    if (now - dm_last_enable_tick > 100) {
        DMMotor_Enable(dm_motor, DM_MODE_MIT);
        dm_last_enable_tick = now;
    }

    static uint8_t inited = 0;
    static float target_p = 0.0f;
    static uint32_t last_step_tick = 0;

    if (!inited) {
        if (fb) {
            target_p = fb->position_rad;
        }
        last_step_tick = now;
        inited = 1;
    }

    if (interval_ms != 0U) {
        uint32_t elapsed = now - last_step_tick;
        if (elapsed >= interval_ms) {
            uint32_t steps = elapsed / interval_ms;
            last_step_tick += steps * interval_ms;

            target_p += DM_DegToRad(step_deg) * (float)steps;
            while (target_p > DM_P_RANGE) target_p -= 2.0f * DM_P_RANGE;
            while (target_p < -DM_P_RANGE) target_p += 2.0f * DM_P_RANGE;
        }
    }

    DMMotor_SendMIT(dm_motor, target_p, 0.0f, g_dm_kp_step, g_dm_kd_step, g_dm_tff_step);
}

/**
 * @brief Execute a GM6020 speed test (ANGLE->SPEED control disabled, only
 *        speed loop) using CAN1.
 *
 * @param target_speed_deg_s Reference in deg/s, clamped internally.
 */
void MotorTest_GM6020_SpeedLoop(float target_speed_deg_s) {
    EnsureGM6020MotorReady();
    target_speed_deg_s = ClampFloat(target_speed_deg_s, GM6020_SPEED_MIN, GM6020_SPEED_MAX);
    DJIMotorOuterLoop(gm6020_motor, SPEED_LOOP);
    DJIMotorEnable(gm6020_motor);
    DJIMotorSetRef(gm6020_motor, target_speed_deg_s);
    LOGINFO("[motor_test] GM6020 speed ref %d deg/s", (int)target_speed_deg_s);
}

/**
 * @brief Execute a GM6020 position test (ANGLE->SPEED cascade).
 *
 * @param target_angle_deg Reference angle in deg; helper clamps to a reasonable
 *                         range for safety.
 */
void MotorTest_GM6020_PositionLoop(float target_angle_deg) {
    EnsureGM6020MotorReady();
    target_angle_deg = ClampFloat(target_angle_deg, GM6020_ANGLE_MIN, GM6020_ANGLE_MAX);
    DJIMotorOuterLoop(gm6020_motor, ANGLE_LOOP);
    DJIMotorEnable(gm6020_motor);
    DJIMotorSetRef(gm6020_motor, target_angle_deg);
    LOGINFO("[motor_test] GM6020 angle ref %d deg", (int)target_angle_deg);
}

/**
 * @brief Lazy-register the M3508 motor with CAN1 and copy the PID configuration
 *        from the production chassis module for realistic behaviour.
 */
static void EnsureM3508MotorReady(void) {
    for (uint8_t i = 0; i < M3508_MOTOR_COUNT; ++i) {
        if (m3508_motors[i] != NULL) {
            continue;
        }

        Motor_Init_Config_s config = {
                .can_init_config =
                        {
                                .can_handle = &hcan1,
                                .tx_id = M3508_CAN_IDS[i],
                        },
                .controller_param_init_config =
                        {
                                .angle_PID =
                                        {
                                                .Kp = 5.0f,
                                                .Ki = 0.0f,
                                                .Kd = 0.0f,
                                                .MaxOut = M3508_SPEED_MAX,
                                                .IntegralLimit = 500.0f,
                                                .Improve = PID_Trapezoid_Intergral |
                                                           PID_Integral_Limit,
                                        },
                                .speed_PID =
                                        {
                                                .Kp = 10.0f,
                                                .Ki = 0.0f,
                                                .Kd = 0.0f,
                                                .IntegralLimit = 3000.0f,

                                                .Improve = PID_Trapezoid_Intergral |
                                                           PID_Integral_Limit |
                                                           PID_Derivative_On_Measurement,
                                                .MaxOut = 12000.0f,
                                        },
                                .current_PID =
                                        {
                                                .Kp = 0.5f,
                                                .Ki = 0.0f,
                                                .Kd = 0.0f,
                                                .IntegralLimit = 3000.0f,
                                                .Improve = PID_Trapezoid_Intergral |
                                                           PID_Integral_Limit |
                                                           PID_Derivative_On_Measurement,
                                                .MaxOut = 15000.0f,
                                        },
                        },
                .controller_setting_init_config =
                        {
                                .angle_feedback_source = MOTOR_FEED,
                                .speed_feedback_source = MOTOR_FEED,
                                .outer_loop_type = SPEED_LOOP,
                                .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
                                .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                        },
                .motor_type = M3508,
        };

        m3508_motors[i] = DJIMotorInit(&config);
        LOGINFO("[motor_test] M3508 index %u registered on CAN1 id %d", (unsigned)i,
                (int)M3508_CAN_IDS[i]);
    }
}

/**
 * @brief Register the loader motor (M3508, CAN1 id 7). Its CAN sender group is
 *        different from ids 1-4, so it is handled separately.
 */
static void EnsureLoaderMotorReady(void) {
    if (loader_motor != NULL) {
        return;
    }

    Motor_Init_Config_s config = {
            .can_init_config =
                    {
                            .can_handle = &hcan1,
                            .tx_id = 7U,  // uses sender group 0x1ff on CAN1
                    },
            .controller_param_init_config =
                    {
                            .angle_PID =
                                    {
                                            .Kp = 5.0f,
                                            .Ki = 0.0f,
                                            .Kd = 0.0f,
                                            .MaxOut = M3508_SPEED_MAX,
                                            .IntegralLimit = 500.0f,
                                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                                    },
                            .speed_PID =
                                    {
                                            .Kp = 10.0f,
                                            .Ki = 0.0f,
                                            .Kd = 0.0f,
                                            .IntegralLimit = 3000.0f,

                                            .Improve = PID_Trapezoid_Intergral |
                                                       PID_Integral_Limit |
                                                       PID_Derivative_On_Measurement,
                                            .MaxOut = 12000.0f,
                                    },
                            .current_PID =
                                    {
                                            .Kp = 0.5f,
                                            .Ki = 0.0f,
                                            .Kd = 0.0f,
                                            .IntegralLimit = 3000.0f,
                                            .Improve = PID_Trapezoid_Intergral |
                                                       PID_Integral_Limit |
                                                       PID_Derivative_On_Measurement,
                                            .MaxOut = 15000.0f,
                                    },
                    },
            .controller_setting_init_config =
                    {
                            .angle_feedback_source = MOTOR_FEED,
                            .speed_feedback_source = MOTOR_FEED,
                            .outer_loop_type = SPEED_LOOP,
                            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
                            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    },
            .motor_type = M3508,
    };

    loader_motor = DJIMotorInit(&config);
    LOGINFO("[motor_test] Loader M3508 registered on CAN1 id 7");
}

/**
 * @brief Lazy-register the GM6020 motor with CAN1. PID gains are borrowed from
 *        the gimbal module so the test mimics the real control stack.
 */
static void EnsureGM6020MotorReady(void) {
    if (gm6020_motor != NULL) {
        return;
    }

    Motor_Init_Config_s config = {
            .can_init_config =
                    {
                            .can_handle = &hcan1,
                            .tx_id = GM6020_CAN_ID,
                    },
            .controller_param_init_config =
                    {
                            .angle_PID =
                                    {
                                            .Kp = 8.0f,
                                            .Ki = 0.0f,
                                            .Kd = 0.0f,
                                            .DeadBand = 0.1f,
                                            .IntegralLimit = 100.0f,
                                            .Improve = PID_Trapezoid_Intergral |
                                                       PID_Integral_Limit |
                                                       PID_Derivative_On_Measurement,
                                            .MaxOut = 500.0f,
                                    },
                            .speed_PID =
                                    {
                                            .Kp = 10.0f,
                                            .Ki = 40.0f,
                                            .Kd = 0.0f,
                                            .IntegralLimit = 3000.0f,
                                            .Improve = PID_Trapezoid_Intergral |
                                                       PID_Integral_Limit |
                                                       PID_Derivative_On_Measurement,
                                            .MaxOut = 20000.0f,
                                    },
                    },
            .controller_setting_init_config =
                    {
                            .angle_feedback_source = MOTOR_FEED,
                            .speed_feedback_source = MOTOR_FEED,
                            .outer_loop_type = ANGLE_LOOP,
                            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    },
            .motor_type = GM6020,
    };

    gm6020_motor = DJIMotorInit(&config);
    LOGINFO("[motor_test] GM6020 registered on CAN1 id %d", (int)GM6020_CAN_ID);
}

static void EnsureDM8009PMotorReady(void) {
    if (dm_motor != NULL) return;

    DMMotor_InitConfig config = {
            .can_handle = &hcan1,
            .motor_id = DM_MOTOR_ID,
            .master_id = DM_MASTER_ID,
            .auto_clear_error = true,
            .auto_enable_mit = false,
            /* Demo 建议不要每次上电都“保存零位”（可能会影响标定）；需要时手动调用
               DMMotor_SaveZero() */
            .auto_zero_position = false,
            .use_shared_feedback_id = true,
            .position_range = DM_P_RANGE,
            .velocity_range = DM_V_RANGE,
            .torque_range = DM_T_RANGE,
    };

    dm_motor = DMMotor_Init(&config);
    LOGINFO("[motor_test] DM init done, motor_id=%d master_id=%d", (int)DM_MOTOR_ID,
            (int)DM_MASTER_ID);
}

/**
 * @brief Simple helper to keep references within the specified limit.
 */
static float ClampFloat(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM14) {
        HAL_IncTick();
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
            RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
    (void)file;
    (void)line;
}
#endif
