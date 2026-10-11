/**
 * @file shoot.c
 * @brief 机器人摩擦轮与拨弹电机控制
 */

#include "shoot.h"
#include "shoot_trigger.h"
#include "robot_config.h"
#include "robot_types.h"
#include "referee.h"
#include "debug.h"
#include "dji_motor.h"
#include "rm_critical.h"
#include "rm_time.h"
#include "user_lib.h"
#include "can.h"
#include <math.h>

/*============================================================================
 * 私有变量
 *============================================================================*/
static DJIMotorInstance *motor_friction_l = NULL;
static DJIMotorInstance *motor_friction_r = NULL;
static DJIMotorInstance *motor_loader = NULL;

static ShootState_e shoot_state = SHOOT_OFF;
static float loader_target_angle = 0.0f;
static float loader_position_cmd = 0.0f;
static uint8_t single_shot_active = 0;
static uint8_t pending_shots = 0;
static float loader_speed_cmd = 0.0f;
static uint32_t loader_speed_tick = 0U;
static uint32_t loader_position_tick = 0U;
static uint32_t single_shot_start_ms = 0U;
static uint32_t single_settle_start_ms = 0U;
static uint32_t single_hold_start_ms = 0U;
static uint32_t single_timeout_count = 0U;
static uint32_t single_start_count = 0U;
static uint8_t single_hold_active = 0U;
static SingleShotTrigger single_trigger;
static RobotFireMode_e input_fire_mode = ROBOT_FIRE_DISABLED;
static uint8_t input_fire_trigger_down = 0U;
static uint8_t input_fire_trigger_pressed = 0U;
static float loader_ref_last = 0.0f;
static uint8_t last_continuous_mode = 0U;
static float friction_speed_cmd = 0.0f;
static uint32_t friction_speed_tick = 0U;
static uint32_t friction_ready_start_ms = 0U;
static uint32_t friction_drop_start_ms = 0U;
static uint8_t friction_ready = 0U;
static ShootLoaderJamState_e loader_jam_state = SHOOT_LOADER_JAM_IDLE;
static uint32_t loader_jam_stall_start_ms = 0U;
static uint32_t loader_jam_reverse_start_ms = 0U;
static uint8_t loader_jam_retry_count = 0U;
static uint32_t loader_jam_fault_count = 0U;

static bool PublishMotorCommand(DJIMotorInstance *motor,
                                Closeloop_Type_e outer_loop,
                                float reference,
                                Motor_Working_Type_e working_state,
                                bool update_working_state)
{
    DJIMotorCommand command;

    if (!DJIMotorGetCommand(motor, &command)) {
        return false;
    }
    command.settings.outer_loop_type = outer_loop;
    command.reference = reference;
    if (update_working_state) {
        command.working_state = working_state;
    }
    return DJIMotorPublishCommand(motor, &command);
}

static bool PublishMotorCommandWithReset(DJIMotorInstance *motor,
                                         Closeloop_Type_e outer_loop,
                                         float reference,
                                         Motor_Working_Type_e working_state,
                                         uint8_t pid_reset_mask)
{
    DJIMotorCommand command;

    if (!DJIMotorGetCommand(motor, &command)) {
        return false;
    }
    command.settings.outer_loop_type = outer_loop;
    command.reference = reference;
    command.working_state = working_state;
    command.pid_reset_mask = pid_reset_mask;
    return DJIMotorPublishCommand(motor, &command);
}

static float MoveToward(float current, float target, float max_delta)
{
    if (max_delta <= 0.0f) {
        return current;
    }
    if (target > current + max_delta) {
        return current + max_delta;
    }
    if (target < current - max_delta) {
        return current - max_delta;
    }
    return target;
}

static float LoaderTotalAngle(void)
{
    DJI_Motor_Measure_s measure;

    return (motor_loader != NULL &&
            DJIMotorGetMeasure(motor_loader, &measure))
               ? measure.total_angle
               : 0.0f;
}

static float LoaderSpeed(void)
{
    DJI_Motor_Measure_s measure;

    return (motor_loader != NULL &&
            DJIMotorGetMeasure(motor_loader, &measure))
               ? measure.speed_aps
               : 0.0f;
}

static bool MotorMeasure(DJIMotorInstance *motor,
                         DJI_Motor_Measure_s *measure)
{
    return motor != NULL && measure != NULL &&
           DJIMotorGetMeasure(motor, measure);
}

static void UpdateFrictionReady(float final_target, uint32_t now)
{
    DJI_Motor_Measure_s left;
    DJI_Motor_Measure_s right;
    float target_abs = fabsf(final_target);
    bool feedback_valid = DJIMotorIsOnline(motor_friction_l) &&
                          DJIMotorIsOnline(motor_friction_r) &&
                          MotorMeasure(motor_friction_l, &left) &&
                          MotorMeasure(motor_friction_r, &right);
    bool command_ready;
    bool speed_ready;
    bool speed_within_drop_band;
    float left_error;
    float right_error;

    if (!feedback_valid || target_abs < 1.0f) {
        friction_ready = 0U;
        friction_ready_start_ms = 0U;
        friction_drop_start_ms = 0U;
        return;
    }

    command_ready = fabsf(friction_speed_cmd) >=
                    target_abs * FRICTION_READY_COMMAND_RATIO;
    left_error = fabsf(fabsf(left.speed_rad_s) - target_abs);
    right_error = fabsf(fabsf(right.speed_rad_s) - target_abs);
    speed_ready = left_error <= target_abs * FRICTION_READY_ERROR_RATIO &&
                  right_error <= target_abs * FRICTION_READY_ERROR_RATIO;
    speed_within_drop_band =
        left_error <= target_abs * FRICTION_READY_DROP_ERROR_RATIO &&
        right_error <= target_abs * FRICTION_READY_DROP_ERROR_RATIO;

    if (friction_ready != 0U) {
        if (command_ready && speed_within_drop_band) {
            friction_drop_start_ms = 0U;
        } else if (friction_drop_start_ms == 0U) {
            friction_drop_start_ms = now;
        } else if ((now - friction_drop_start_ms) >=
                   FRICTION_READY_DROP_HOLD_MS) {
            friction_ready = 0U;
            friction_ready_start_ms = 0U;
            friction_drop_start_ms = 0U;
        }
        return;
    }

    if (!command_ready || !speed_ready) {
        friction_ready_start_ms = 0U;
        return;
    }
    if (friction_ready_start_ms == 0U) {
        friction_ready_start_ms = now;
    } else if ((now - friction_ready_start_ms) >= FRICTION_READY_HOLD_MS) {
        friction_ready = 1U;
        friction_drop_start_ms = 0U;
    }
}

static void UpdateFrictionMotors(float target_speed, uint32_t now)
{
    float dt_ms = 0.0f;

    if (friction_speed_tick == 0U) {
        friction_speed_tick = now;
    } else {
        dt_ms = (float)(now - friction_speed_tick);
        friction_speed_tick = now;
    }
    friction_speed_cmd = MoveToward(
        friction_speed_cmd, target_speed,
        FRICTION_SPEED_SLEW_RAD_S_PER_MS * dt_ms);

    if (motor_friction_l) {
        (void)PublishMotorCommand(motor_friction_l, FRICTION_RUN_LOOP_ON,
                                  friction_speed_cmd, MOTOR_ENALBED, true);
    }
    if (motor_friction_r) {
        (void)PublishMotorCommand(motor_friction_r, FRICTION_RUN_LOOP_ON,
                                  friction_speed_cmd, MOTOR_ENALBED, true);
    }
    UpdateFrictionReady(target_speed, now);
}

static void StopLoader(uint32_t now)
{
    if (motor_loader) {
        (void)PublishMotorCommand(motor_loader, LOADER_RUN_LOOP_STOP, 0.0f,
                                  MOTOR_STOP, true);
    }
    loader_speed_cmd = 0.0f;
    loader_speed_tick = now;
    loader_position_tick = now;
    loader_ref_last = 0.0f;
}

static void ResetLoaderJamRecovery(bool clear_retry_count)
{
    loader_jam_state = SHOOT_LOADER_JAM_IDLE;
    loader_jam_stall_start_ms = 0U;
    loader_jam_reverse_start_ms = 0U;
    if (clear_retry_count) {
        loader_jam_retry_count = 0U;
    }
}

static float LoaderFinalOutput(void)
{
    DJIMotorControlSnapshot control;

    return motor_loader != NULL &&
                   DJIMotorGetControlSnapshot(motor_loader, &control)
               ? control.final_output
               : 0.0f;
}

static bool LoaderJamCondition(void)
{
    return fabsf(loader_speed_cmd) >=
               fabsf(LOADER_CONTINUOUS_SPEED_DEG_S) *
                   LOADER_JAM_DETECT_COMMAND_RATIO &&
           fabsf(LoaderSpeed()) <= LOADER_JAM_SPEED_THRESHOLD_DEG_S &&
           fabsf(LoaderFinalOutput()) >=
               LOADER_SPEED_MAX_OUT * LOADER_JAM_OUTPUT_RATIO;
}

static bool UpdateLoaderJamRecovery(uint32_t now)
{
    if (loader_jam_state == SHOOT_LOADER_JAM_LOCKED) {
        StopLoader(now);
        return true;
    }

    if (loader_jam_state == SHOOT_LOADER_JAM_REVERSING) {
        if ((now - loader_jam_reverse_start_ms) < LOADER_JAM_REVERSE_MS) {
            (void)PublishMotorCommand(motor_loader,
                                      LOADER_RUN_LOOP_CONTINUOUS,
                                      LOADER_JAM_REVERSE_SPEED_DEG_S,
                                      MOTOR_ENALBED, true);
            loader_ref_last = LOADER_JAM_REVERSE_SPEED_DEG_S;
            return true;
        }

        loader_jam_state = SHOOT_LOADER_JAM_IDLE;
        loader_jam_reverse_start_ms = 0U;
        loader_speed_cmd = 0.0f;
        loader_speed_tick = now;
        (void)PublishMotorCommandWithReset(
            motor_loader, LOADER_RUN_LOOP_CONTINUOUS, 0.0f,
            MOTOR_ENALBED, DJI_MOTOR_PID_RESET_ALL);
        loader_ref_last = 0.0f;
        return true;
    }

    if (!LoaderJamCondition()) {
        loader_jam_stall_start_ms = 0U;
        return false;
    }
    if (loader_jam_stall_start_ms == 0U) {
        loader_jam_stall_start_ms = now;
        return false;
    }
    if ((now - loader_jam_stall_start_ms) < LOADER_JAM_DETECT_MS) {
        return false;
    }

    loader_jam_stall_start_ms = 0U;
    if (loader_jam_retry_count >= LOADER_JAM_MAX_RETRIES) {
        loader_jam_state = SHOOT_LOADER_JAM_LOCKED;
        loader_jam_fault_count++;
        StopLoader(now);
        MDBG_SHT("loader jam locked retries=%u faults=%lu",
                 (unsigned)loader_jam_retry_count,
                 (unsigned long)loader_jam_fault_count);
        return true;
    }

    loader_jam_retry_count++;
    loader_jam_state = SHOOT_LOADER_JAM_REVERSING;
    loader_jam_reverse_start_ms = now;
    (void)PublishMotorCommandWithReset(
        motor_loader, LOADER_RUN_LOOP_CONTINUOUS,
        LOADER_JAM_REVERSE_SPEED_DEG_S, MOTOR_ENALBED,
        DJI_MOTOR_PID_RESET_ALL);
    loader_ref_last = LOADER_JAM_REVERSE_SPEED_DEG_S;
    MDBG_SHT("loader jam reverse retry=%u speed=%.1f output=%.1f",
             (unsigned)loader_jam_retry_count,
             (double)LoaderSpeed(), (double)LoaderFinalOutput());
    return true;
}

/*============================================================================
 * 公共函数
 *============================================================================*/
bool Shoot_Init(void)
{
    SingleShotTrigger_Init(&single_trigger);
    input_fire_mode = ROBOT_FIRE_DISABLED;
    input_fire_trigger_down = 0U;
    input_fire_trigger_pressed = 0U;
    single_start_count = 0U;

    // 摩擦轮电机配置 - M3508速度环
    Motor_Init_Config_s friction_config = {
        .motor_type = M3508,
        .can_init_config = {
            .can_handle = &FRICTION_CAN,
            .tx_id = FRICTION_LEFT_ID,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = FRICTION_SPEED_KP,
                .Ki = FRICTION_SPEED_KI,
                .Kd = FRICTION_SPEED_KD,
                .MaxOut = FRICTION_SPEED_MAX_OUT,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .speed_unit = MOTOR_SPEED_RAD_PER_SEC,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };
    motor_friction_l = DJIMotorInit(&friction_config);
    if (motor_friction_l) {
        (void)PublishMotorCommand(motor_friction_l, FRICTION_INIT_LOOP, 0.0f,
                                  MOTOR_STOP, true);
    }
    
    // 右摩擦轮(反向)
    friction_config.can_init_config.tx_id = FRICTION_RIGHT_ID;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_friction_r = DJIMotorInit(&friction_config);
    if (motor_friction_r) {
        (void)PublishMotorCommand(motor_friction_r, FRICTION_INIT_LOOP, 0.0f,
                                  MOTOR_STOP, true);
    }
    
    // 拨弹电机配置 - M2006位置环
    Motor_Init_Config_s loader_config = {
        .motor_type = M2006,
        .can_init_config = {
            .can_handle = &LOADER_CAN,
            .tx_id = LOADER_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = LOADER_ANGLE_KP,
                .Ki = LOADER_ANGLE_KI,
                .Kd = LOADER_ANGLE_KD,
                .MaxOut = LOADER_ANGLE_MAX_OUT,
                .IntegralLimit = 500.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
            },
            .speed_PID = {
                .Kp = LOADER_SPEED_KP,
                .Ki = LOADER_SPEED_KI,
                .Kd = LOADER_SPEED_KD,
                .IntegralLimit = LOADER_SPEED_I_MAX,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = LOADER_SPEED_MAX_OUT,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .speed_unit = MOTOR_SPEED_DEG_PER_SEC,
            .outer_loop_type = LOADER_INIT_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };
    motor_loader = DJIMotorInit(&loader_config);
    if (motor_loader) {
        (void)PublishMotorCommand(motor_loader, LOADER_INIT_LOOP, 0.0f,
                                  MOTOR_STOP, true);
    }

    return (motor_friction_l != NULL) && (motor_friction_r != NULL) &&
           (motor_loader != NULL);
}

void Shoot_Update(Input_Data_t *input)
{
    uint32_t now = RmTime_NowMs();
    Shoot_Cmd_t cmd = {0};
    uint8_t continuous_requested;
    bool single_trigger_activated;
    float friction_target_speed;

    if (input == NULL) {
        input_fire_mode = ROBOT_FIRE_DISABLED;
        input_fire_trigger_down = 0U;
        input_fire_trigger_pressed = 0U;
        MDBG_SHT("stop by null input");
        Shoot_Stop();
        return;
    }

    input_fire_mode = input->fire_mode;
    input_fire_trigger_down = input->fire_trigger_down;
    input_fire_trigger_pressed = input->fire_trigger_pressed;
    if (!input->online || input->emergency_stop) {
        MDBG_SHT("stop by input offline/estop");
        Shoot_Stop();
        return;
    }

    /*
     * 单发的一次性语义由执行层根据稳定电平维护。遥控边沿只用于诊断，
     * 即使某一帧 changed_mask 丢失，也不会漏掉已经稳定上拨的 SD。
     */
    single_trigger_activated = SingleShotTrigger_Update(
        &single_trigger, input->fire_mode == ROBOT_FIRE_SINGLE,
        input->fire_trigger_down != 0U);

    if (!MinimalReferee_AllowShoot()) {
        MDBG_SHT("stop by referee lock");
        Shoot_Stop();
        return;
    }

    /* SC 下位或应用层禁用发射时，摩擦轮与拨弹电机都必须立即停机。 */
    if (input->fire_mode == ROBOT_FIRE_DISABLED) {
        Shoot_Stop();
        return;
    }

    if (input->fire_mode == ROBOT_FIRE_SINGLE) {
        /* SD 每次上拨只消费一次；忙时最多保留一个待发请求。 */
        if (single_trigger_activated) {
            if (pending_shots < LOADER_SINGLE_QUEUE_LIMIT) {
                pending_shots++;
                MDBG_SHT("single trigger queued level=1 edge=%u count=%lu",
                         (unsigned)input->fire_trigger_pressed,
                         (unsigned long)single_trigger.activation_count);
            } else {
                MDBG_SHT("single trigger dropped queue full");
            }
        }
    } else {
        /* 模式切换时不允许遗留的单发目标与连发速度环并存。 */
        single_shot_active = 0U;
        pending_shots = 0U;
        single_hold_active = 0U;
        single_settle_start_ms = 0U;
    }

    continuous_requested =
        (input->fire_mode == ROBOT_FIRE_CONTINUOUS &&
         input->fire_trigger_down != 0U)
            ? 1U
            : 0U;

    if (continuous_requested != 0U) {
        shoot_state = SHOOT_CONTINUOUS;
    } else if (single_shot_active != 0U || pending_shots != 0U ||
               single_hold_active != 0U) {
        shoot_state = SHOOT_SINGLE;
    } else {
        shoot_state = SHOOT_FRICTION_ON;
    }

    cmd.friction = FRICTION_ON;
    cmd.loader = shoot_state == SHOOT_CONTINUOUS
                     ? LOADER_CONTINUOUS
                     : (shoot_state == SHOOT_SINGLE ? LOADER_SINGLE
                                                    : LOADER_STOP);
    cmd.control_mode = CTRL_ENABLE;
    cmd.ref_type = (shoot_state == SHOOT_SINGLE) ? REF_ANGLE : REF_SPEED;
    g_robot.shoot = cmd;
    
    friction_target_speed = FRICTION_TARGET_SPEED_RAD_S *
                            MinimalReferee_FrictionSpeedScale();
    UpdateFrictionMotors(friction_target_speed, now);
    
    if (motor_loader == NULL || !MinimalReferee_AllowLoader()) {
        if (motor_loader) {
            (void)PublishMotorCommand(motor_loader, LOADER_RUN_LOOP_STOP,
                                      0.0f, MOTOR_STOP, true);
        }
        single_shot_active = 0;
        pending_shots = 0;
        single_hold_active = 0U;
        single_settle_start_ms = 0U;
        last_continuous_mode = 0U;
        ResetLoaderJamRecovery(true);
        StopLoader(now);
        shoot_state = SHOOT_FRICTION_ON;
        g_robot.shoot.loader = LOADER_STOP;
        g_robot.shoot.ref_type = REF_SPEED;
        return;
    }

    /* 摩擦轮完成升速并稳定前，不允许拨弹；严重掉速会中止当前动作。 */
    if (friction_ready == 0U) {
        if (single_shot_active != 0U || single_hold_active != 0U) {
            single_shot_active = 0U;
            single_hold_active = 0U;
            single_settle_start_ms = 0U;
            pending_shots = 0U;
            MDBG_SHT("single aborted by friction speed drop");
        }
        last_continuous_mode = 0U;
        ResetLoaderJamRecovery(true);
        StopLoader(now);
        return;
    }
    
    if (shoot_state == SHOOT_CONTINUOUS) {
        float speed_target = LOADER_CONTINUOUS_SPEED_DEG_S;
        float dt_ms;
        float max_delta;
        bool jam_handled;

        if (!last_continuous_mode) {
            loader_speed_cmd = 0.0f;
            loader_speed_tick = now;
            ResetLoaderJamRecovery(true);
        }
        if (loader_jam_state == SHOOT_LOADER_JAM_IDLE &&
            loader_speed_tick != 0U) {
            dt_ms = (float)(now - loader_speed_tick);
            loader_speed_tick = now;
            max_delta = LOADER_CONTINUOUS_SLEW_PER_MS * dt_ms;
            loader_speed_cmd = MoveToward(loader_speed_cmd, speed_target,
                                          max_delta);
        }
        jam_handled = UpdateLoaderJamRecovery(now);
        if (!jam_handled && !last_continuous_mode) {
            (void)PublishMotorCommandWithReset(
                motor_loader, LOADER_RUN_LOOP_CONTINUOUS, loader_speed_cmd,
                MOTOR_ENALBED, DJI_MOTOR_PID_RESET_ALL);
        } else if (!jam_handled) {
            (void)PublishMotorCommand(motor_loader,
                                      LOADER_RUN_LOOP_CONTINUOUS,
                                      loader_speed_cmd, MOTOR_ENALBED, true);
        }
        if (!jam_handled) {
            loader_ref_last = loader_speed_cmd;
        }
        if (!last_continuous_mode) {
            MDBG_SHT("enter continuous");
        }
        last_continuous_mode = 1U;
        single_shot_active = 0;
        pending_shots = 0;
    } else if (shoot_state == SHOOT_SINGLE) {
        bool shot_started = false;

        if (last_continuous_mode) {
            MDBG_SHT("exit continuous");
        }
        last_continuous_mode = 0U;
        ResetLoaderJamRecovery(true);

        if (!single_shot_active && !single_hold_active &&
            pending_shots > 0U) {
            loader_position_cmd = LoaderTotalAngle();
            loader_target_angle = loader_position_cmd + LOADER_ANGLE_STEP;
            pending_shots--;
            single_shot_active = 1U;
            if (single_start_count < UINT32_MAX) {
                single_start_count++;
            }
            single_shot_start_ms = now;
            loader_position_tick = now;
            single_settle_start_ms = 0U;
            shot_started = true;
            MDBG_SHT("single start target=%.1f pending=%u",
                     (double)loader_target_angle, (unsigned)pending_shots);
        }

        if (single_shot_active) {
            float dt_ms = (float)(now - loader_position_tick);
            float angle_error;
            float speed_abs;

            loader_position_tick = now;
            loader_position_cmd = MoveToward(
                loader_position_cmd, loader_target_angle,
                LOADER_SINGLE_REF_SLEW_DEG_PER_MS * dt_ms);
            if (shot_started) {
                (void)PublishMotorCommandWithReset(
                    motor_loader, LOADER_RUN_LOOP_SINGLE,
                    loader_position_cmd, MOTOR_ENALBED,
                    DJI_MOTOR_PID_RESET_ALL);
            } else {
                (void)PublishMotorCommand(motor_loader,
                                          LOADER_RUN_LOOP_SINGLE,
                                          loader_position_cmd,
                                          MOTOR_ENALBED, true);
            }
            loader_ref_last = loader_position_cmd;

            angle_error = fabsf(LoaderTotalAngle() - loader_target_angle);
            speed_abs = fabsf(LoaderSpeed());
            if (fabsf(loader_position_cmd - loader_target_angle) < 0.01f &&
                angle_error <= LOADER_SINGLE_SETTLE_EPS_DEG &&
                speed_abs <= LOADER_SINGLE_SETTLE_SPEED_DEG_S) {
                if (single_settle_start_ms == 0U) {
                    single_settle_start_ms = now;
                } else if ((now - single_settle_start_ms) >=
                           LOADER_SINGLE_SETTLE_MS) {
                    single_shot_active = 0U;
                    single_hold_active = 1U;
                    single_hold_start_ms = now;
                    MDBG_SHT("single done");
                }
            } else {
                single_settle_start_ms = 0U;
            }

            if (single_shot_active != 0U &&
                (now - single_shot_start_ms) >=
                    LOADER_SINGLE_TIMEOUT_MS) {
                single_timeout_count++;
                single_shot_active = 0U;
                single_hold_active = 0U;
                single_settle_start_ms = 0U;
                pending_shots = 0U;
                MDBG_SHT("single timeout err=%.1f speed=%.1f",
                         (double)angle_error, (double)speed_abs);
                StopLoader(now);
                return;
            }
        }

        if (single_hold_active) {
            (void)PublishMotorCommand(motor_loader, LOADER_RUN_LOOP_SINGLE,
                                      loader_target_angle, MOTOR_ENALBED, true);
            loader_ref_last = loader_target_angle;
            if ((now - single_hold_start_ms) >= LOADER_SINGLE_HOLD_MS) {
                single_hold_active = 0U;
                if (pending_shots == 0U) {
                    StopLoader(now);
                } else {
                    loader_position_tick = now;
                }
            }
        } else if (!single_shot_active) {
            StopLoader(now);
        }
    } else {
        if (last_continuous_mode) {
            MDBG_SHT("exit continuous");
        }
        last_continuous_mode = 0U;
        ResetLoaderJamRecovery(true);
        single_shot_active = 0;
        pending_shots = 0;
        single_hold_active = 0U;
        single_settle_start_ms = 0U;
        StopLoader(now);
    }
}

void Shoot_Stop(void)
{
    /* 扳机消费锁存由输入电平重装；停机本身不能制造新的单发请求。 */
    shoot_state = SHOOT_OFF;
    single_shot_active = 0;
    pending_shots = 0;
    single_hold_active = 0U;
    single_settle_start_ms = 0U;
    single_hold_start_ms = 0U;
    loader_target_angle = 0.0f;
    loader_position_cmd = 0.0f;
    loader_speed_cmd = 0.0f;
    loader_speed_tick = 0U;
    loader_position_tick = 0U;
    single_shot_start_ms = 0U;
    loader_ref_last = 0.0f;
    last_continuous_mode = 0U;
    friction_speed_cmd = 0.0f;
    friction_speed_tick = 0U;
    friction_ready_start_ms = 0U;
    friction_drop_start_ms = 0U;
    friction_ready = 0U;
    ResetLoaderJamRecovery(true);
    g_robot.shoot = (Shoot_Cmd_t){
        .friction = FRICTION_OFF,
        .loader = LOADER_STOP,
        .control_mode = CTRL_ZERO_FORCE,
        .ref_type = REF_SPEED,
    };
    
    if (motor_friction_l) {
        (void)PublishMotorCommand(motor_friction_l,
                                  FRICTION_RUN_LOOP_STOP, 0.0f,
                                  MOTOR_STOP, true);
    }
    if (motor_friction_r) {
        (void)PublishMotorCommand(motor_friction_r,
                                  FRICTION_RUN_LOOP_STOP, 0.0f,
                                  MOTOR_STOP, true);
    }
    if (motor_loader) {
        (void)PublishMotorCommand(motor_loader, LOADER_RUN_LOOP_STOP, 0.0f,
                                  MOTOR_STOP, true);
    }
}

bool Shoot_IsHealthy(void)
{
    return DJIMotorIsOnline(motor_friction_l) &&
           DJIMotorIsOnline(motor_friction_r) &&
           DJIMotorIsOnline(motor_loader);
}

ShootState_e Shoot_GetState(void)
{
    return shoot_state;
}

float Shoot_GetLoaderRef(void)
{
    return loader_ref_last;
}

float Shoot_GetLoaderFeedback(void)
{
    if (motor_loader == NULL) {
        return 0.0f;
    }
    if (g_robot.shoot.ref_type == REF_ANGLE) {
        return LoaderTotalAngle();
    }
    return LoaderSpeed();
}

uint8_t Shoot_IsSingleActive(void)
{
    return single_shot_active;
}

uint8_t Shoot_GetPendingShots(void)
{
    return pending_shots;
}

bool Shoot_GetTuningSnapshot(ShootTuningSnapshot *snapshot)
{
    RmCriticalState state;

    if (snapshot == NULL) {
        return false;
    }

    state = RmCritical_Enter();
    snapshot->input_fire_mode = input_fire_mode;
    snapshot->fire_trigger_down = input_fire_trigger_down;
    snapshot->fire_trigger_pressed = input_fire_trigger_pressed;
    snapshot->single_trigger_consumed = single_trigger.consumed;
    snapshot->single_trigger_activation_count =
        single_trigger.activation_count;
    snapshot->shoot_state = shoot_state;
    snapshot->friction_ready = friction_ready;
    snapshot->single_active = single_shot_active;
    snapshot->pending_shots = pending_shots;
    snapshot->single_start_count = single_start_count;
    snapshot->single_timeout_count = single_timeout_count;
    snapshot->loader_jam_state = loader_jam_state;
    snapshot->loader_jam_retry_count = loader_jam_retry_count;
    snapshot->loader_jam_fault_count = loader_jam_fault_count;
    RmCritical_Exit(state);
    return true;
}

bool Shoot_GetMotorTuningSnapshot(ShootMotor_e selection,
                                  DJIMotorTuningSnapshot *snapshot)
{
    DJIMotorInstance *motor;

    if (snapshot == NULL) {
        return false;
    }
    switch (selection) {
    case SHOOT_MOTOR_LOADER:
        motor = motor_loader;
        break;
    case SHOOT_MOTOR_FRICTION_LEFT:
        motor = motor_friction_l;
        break;
    case SHOOT_MOTOR_FRICTION_RIGHT:
        motor = motor_friction_r;
        break;
    default:
        return false;
    }
    return DJIMotorGetTuningSnapshot(motor, snapshot);
}
