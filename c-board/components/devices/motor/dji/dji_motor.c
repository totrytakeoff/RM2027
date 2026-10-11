#include "dji_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "rm_critical.h"
#include "rm_time.h"
#include <math.h>
#include <stdatomic.h>
#include <string.h>

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* Fixed-capacity storage for registered DJI motor instances. */
static DJIMotorInstance dji_motor_storage[DJI_MOTOR_CNT];
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算

/**
 * DJI ESC commands share one frame per group of four motors. These six
 * transmit-only instances cover three command groups on each CAN bus.
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

/** Enable only command groups containing a successfully registered motor. */
static uint8_t sender_enable_flag[6] = {0};
static atomic_bool global_output_enabled = ATOMIC_VAR_INIT(true);
/* Disabled by default so existing single-loop demos retain their behavior. */
static _Atomic uint32_t command_timeout_ms = ATOMIC_VAR_INIT(0U);

static float DJIMotorClampOutput(Motor_Type_e motor_type, float output)
{
    float limit;

    if (!isfinite(output)) {
        return 0.0f;
    }

    switch (motor_type) {
    case M2006:
        limit = 10000.0f; /* C610: -10 A..10 A maps to -10000..10000. */
        break;
    case M3508:
        limit = 16384.0f; /* C620 command range. */
        break;
    case GM6020:
        limit = 30000.0f;
        break;
    default:
        limit = 32767.0f;
        break;
    }

    if (output > limit) {
        return limit;
    }
    if (output < -limit) {
        return -limit;
    }
    return output;
}

static void DJIMotorRecordCommandPublication(DJIMotorInstance *motor,
                                             uint32_t now_ms)
{
    motor->command_last_publish_ms = now_ms;
    motor->command_generation++;
    motor->command_published = 1U;
}

static void DJIMotorExpireCommand(DJIMotorInstance *motor,
                                  uint32_t generation)
{
    RmCriticalState state = RmCritical_Enter();

    /* A newer publication wins if it arrived after the command snapshot. */
    if (motor->command_generation == generation) {
        motor->command_published = 0U;
    }
    RmCritical_Exit(state);
}

static void DJIMotorResetPidRuntime(DJIMotorInstance *motor, uint8_t mask)
{
    if ((mask & DJI_MOTOR_PID_RESET_CURRENT) != 0U) {
        PIDReset(&motor->motor_controller.current_PID);
    }
    if ((mask & DJI_MOTOR_PID_RESET_SPEED) != 0U) {
        PIDReset(&motor->motor_controller.speed_PID);
    }
    if ((mask & DJI_MOTOR_PID_RESET_ANGLE) != 0U) {
        PIDReset(&motor->motor_controller.angle_PID);
    }
}

static void DJIMotorConsumeCommand(DJIMotorInstance *motor,
                                   DJIMotorCommand *command,
                                   uint32_t *last_publish_ms,
                                   uint32_t *generation,
                                   bool *published)
{
    RmCriticalState state = RmCritical_Enter();

    memcpy(command, &motor->command_mailbox, sizeof(*command));
    command->pid_reset_mask = motor->pending_pid_reset_mask;
    *last_publish_ms = motor->command_last_publish_ms;
    *generation = motor->command_generation;
    *published = motor->command_published != 0U;
    motor->pending_pid_reset_mask = DJI_MOTOR_PID_RESET_NONE;
    RmCritical_Exit(state);

    motor->motor_settings = command->settings;
    motor->motor_controller.pid_ref = command->reference;
    motor->stop_flag = command->working_state;
    DJIMotorResetPidRuntime(motor, command->pid_reset_mask);
}

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令。
 */
static bool MotorSenderGrouping(DJIMotorInstance *motor,
                                CAN_Init_Config_s *config)
{
    uint8_t motor_id;
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    if ((motor == NULL) || (config == NULL) ||
        ((config->can_handle != &hcan1) &&
         (config->can_handle != &hcan2)) ||
        (config->tx_id < 1U) || (config->tx_id > 8U)) {
        return false;
    }
    motor_id = (uint8_t)(config->tx_id - 1U);

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 1 : 4;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }

        // 计算接收id并设置分组发送id
        config->rx_id = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        // 检查是否发生id冲突
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                LOGERROR("[dji_motor] id [%lu], can_bus [%u]",
                         (unsigned long)config->rx_id,
                         (unsigned)can_bus);
                return false;
            }
        }
        break;

    case GM6020:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 2 : 5;
        }

        config->rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                LOGERROR("[dji_motor] id [%lu], can_bus [%u]",
                         (unsigned long)config->rx_id,
                         (unsigned)can_bus);
                return false;
            }
        }
        break;

    default:
        LOGERROR("[dji_motor] unsupported motor type");
        return false;
    }

    return true;
}

/**
 * @todo 评估是否可以简化多圈角度计算。
 * @brief 解码 CAN 反馈并发布一致性测量快照。
 *
 * @param can_instance 收到反馈的 CAN 端点。
 */
bool DJIMotorGetMeasure(const DJIMotorInstance *motor,
                        DJI_Motor_Measure_s *measure)
{
    RmCriticalState state;

    if ((motor == NULL) || (measure == NULL)) {
        return false;
    }

    state = RmCritical_Enter();
    memcpy(measure, &motor->measure, sizeof(*measure));
    RmCritical_Exit(state);
    return true;
}

bool DJIMotorGetCommand(const DJIMotorInstance *motor,
                        DJIMotorCommand *command)
{
    RmCriticalState state;

    if ((motor == NULL) || (command == NULL)) {
        return false;
    }

    state = RmCritical_Enter();
    memcpy(command, &motor->command_mailbox, sizeof(*command));
    RmCritical_Exit(state);
    command->pid_reset_mask = DJI_MOTOR_PID_RESET_NONE;
    return true;
}

static void DJIMotorCopyPidSnapshot(const PIDInstance *pid,
                                    DJIMotorPidSnapshot *snapshot)
{
    snapshot->reference = pid->Ref;
    snapshot->measure = pid->Measure;
    snapshot->error = pid->Err;
    snapshot->p_output = pid->Pout;
    snapshot->i_output = pid->Iout;
    snapshot->d_output = pid->Dout;
    snapshot->output = pid->Output;
    snapshot->max_output = pid->MaxOut;
}

static void DJIMotorPublishControlSnapshot(DJIMotorInstance *motor)
{
    DJIMotorControlSnapshot next = {0};
    RmCriticalState state;

    if (motor == NULL) {
        return;
    }

    next.settings = motor->motor_settings;
    next.command_reference = motor->motor_controller.pid_ref;
    DJIMotorCopyPidSnapshot(&motor->motor_controller.angle_PID,
                            &next.angle);
    DJIMotorCopyPidSnapshot(&motor->motor_controller.speed_PID,
                            &next.speed);
    DJIMotorCopyPidSnapshot(&motor->motor_controller.current_PID,
                            &next.current);
    next.final_output = motor->final_output;
    next.output_active = motor->output_active;

    state = RmCritical_Enter();
    memcpy(&motor->control_snapshot, &next, sizeof(next));
    motor->control_snapshot_valid = 1U;
    RmCritical_Exit(state);
}

bool DJIMotorGetControlSnapshot(const DJIMotorInstance *motor,
                                DJIMotorControlSnapshot *snapshot)
{
    RmCriticalState state;
    bool valid;

    if ((motor == NULL) || (snapshot == NULL)) {
        return false;
    }

    state = RmCritical_Enter();
    memcpy(snapshot, &motor->control_snapshot, sizeof(*snapshot));
    valid = motor->control_snapshot_valid != 0U;
    RmCritical_Exit(state);
    return valid;
}

bool DJIMotorGetTuningSnapshot(const DJIMotorInstance *motor,
                               DJIMotorTuningSnapshot *snapshot)
{
    RmCriticalState state;
    bool valid;

    if ((motor == NULL) || (snapshot == NULL)) {
        return false;
    }

    state = RmCritical_Enter();
    memcpy(&snapshot->measure, &motor->measure, sizeof(snapshot->measure));
    memcpy(&snapshot->control, &motor->control_snapshot,
           sizeof(snapshot->control));
    valid = motor->feedback_initialized != 0U &&
            motor->control_snapshot_valid != 0U;
    RmCritical_Exit(state);
    return valid;
}

bool DJIMotorPublishCommand(DJIMotorInstance *motor,
                            const DJIMotorCommand *command)
{
    RmCriticalState state;
    uint32_t now_ms;
    uint8_t reset_mask;

    if ((motor == NULL) || (command == NULL)) {
        return false;
    }

    reset_mask = command->pid_reset_mask & DJI_MOTOR_PID_RESET_ALL;
    now_ms = RmTime_NowMs();
    state = RmCritical_Enter();
    memcpy(&motor->command_mailbox, command, sizeof(*command));
    motor->command_mailbox.pid_reset_mask = DJI_MOTOR_PID_RESET_NONE;
    motor->pending_pid_reset_mask |= reset_mask;
    DJIMotorRecordCommandPublication(motor, now_ms);
    RmCritical_Exit(state);
    return true;
}

bool DJIMotorSetCommandTimeout(uint32_t timeout_ms)
{
    if (timeout_ms >= UINT32_C(0x80000000)) {
        return false;
    }
    atomic_store_explicit(&command_timeout_ms, timeout_ms,
                          memory_order_release);
    return true;
}

bool DJIMotorIsOnline(const DJIMotorInstance *motor)
{
    RmCriticalState state;
    bool feedback_initialized;

    if (motor == NULL) {
        return false;
    }
    state = RmCritical_Enter();
    feedback_initialized = motor->feedback_initialized != 0U;
    RmCritical_Exit(state);
    return feedback_initialized && DaemonIsOnline(motor->daemon);
}

bool DJIMotorAllOnline(void)
{
    if (idx == 0U) {
        return false;
    }
    for (size_t i = 0U; i < idx; ++i) {
        if (!DJIMotorIsOnline(dji_motor_instance[i])) {
            return false;
        }
    }
    return true;
}

void DJIMotorSetGlobalOutputEnabled(bool enabled)
{
    atomic_store_explicit(&global_output_enabled, enabled, memory_order_release);
}

bool DJIMotorGlobalOutputEnabled(void)
{
    return atomic_load_explicit(&global_output_enabled, memory_order_acquire);
}

static void DecodeDJIMotor(CANInstance *can_instance)
{
    const uint8_t *rx_buffer;
    DJIMotorInstance *motor;
    DJI_Motor_Measure_s next;
    bool first_feedback;
    RmCriticalState state;

    if ((can_instance == NULL) || (can_instance->id == NULL) ||
        (can_instance->rx_len < 7U)) {
        return;
    }

    rx_buffer = can_instance->rx_buff;
    motor = (DJIMotorInstance *)can_instance->id;
    state = RmCritical_Enter();
    memcpy(&next, &motor->measure, sizeof(next));
    first_feedback = (motor->feedback_initialized == 0U);
    RmCritical_Exit(state);

    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    next.last_ecd = next.ecd;
    next.ecd = ((uint16_t)rx_buffer[0] << 8U) | rx_buffer[1];
    next.angle_single_round = ECD_ANGLE_COEF_DJI * (float)next.ecd;
    next.speed_aps =
        (1.0f - SPEED_SMOOTH_COEF) * next.speed_aps +
        RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF *
            (float)((int16_t)((uint16_t)rx_buffer[2] << 8U |
                              rx_buffer[3]));
    next.speed_rad_s =
        (1.0f - SPEED_SMOOTH_COEF) * next.speed_rad_s +
        RPM_2_RAD_PER_SEC * SPEED_SMOOTH_COEF *
            (float)((int16_t)((uint16_t)rx_buffer[2] << 8U |
                              rx_buffer[3]));
    next.real_current =
        (int16_t)((1.0f - CURRENT_SMOOTH_COEF) *
                      (float)next.real_current +
                  CURRENT_SMOOTH_COEF *
                      (float)((int16_t)((uint16_t)rx_buffer[4] << 8U |
                                        rx_buffer[5])));
    next.temperature = rx_buffer[6];

    if (first_feedback) {
        next.last_ecd = next.ecd;
    } else {
        if ((int32_t)next.ecd - (int32_t)next.last_ecd > 4096) {
            next.total_round--;
        } else if ((int32_t)next.ecd - (int32_t)next.last_ecd < -4096) {
            next.total_round++;
        }
    }
    next.total_angle = (float)next.total_round * 360.0f +
                       next.angle_single_round;

    state = RmCritical_Enter();
    memcpy(&motor->measure, &next, sizeof(next));
    motor->feedback_initialized = 1U;
    RmCritical_Exit(state);
    DaemonReload(motor->daemon);
}

static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;

    if ((motor == NULL) || (motor->motor_can_instance == NULL)) {
        return;
    }
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    LOGERROR("[dji_motor] motor offline, can bus [%u], id [%lu]",
             (unsigned)can_bus,
             (unsigned long)motor->motor_can_instance->tx_id);
}

// 初始化并返回一个固定存储的电机实例。
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    if ((config == NULL) || (idx >= DJI_MOTOR_CNT))
    {
        LOGERROR("[dji_motor] motor instance capacity exhausted");
        return NULL;
    }

    DJIMotorInstance *instance = &dji_motor_storage[idx];
    memset(instance, 0, sizeof(DJIMotorInstance));

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config;
    instance->command_mailbox.settings = config->controller_setting_init_config;
    instance->command_mailbox.working_state = MOTOR_STOP;

    // motor controller init 电机控制器初始化
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    // 前馈源由应用配置，控制任务只读取对应指针。
    DJIMotorPublishControlSnapshot(instance);

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    if (!MotorSenderGrouping(instance, &config->can_init_config)) {
        LOGERROR("[dji_motor] invalid motor CAN grouping");
        return NULL;
    }

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = DecodeDJIMotor; // set callback
    config->can_init_config.id = instance;                        // set id,eq to address(it is identity)
    instance->motor_can_instance = CANRegister(&config->can_init_config);
    if (instance->motor_can_instance == NULL)
    {
        LOGERROR("[dji_motor] CAN endpoint registration failed");
        return NULL;
    }

    // 注册守护线程
    DaemonConfig daemon_config = {
        .callback = DJIMotorLostCallback,
        .owner = instance,
        .timeout_ms = 20U, // 20ms未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);
    if (instance->daemon == NULL)
    {
        LOGERROR("[dji_motor] health endpoint registration failed");
        /* Keep the CAN callback owner reserved, but never enable output. */
        DJIMotorStop(instance);
        dji_motor_instance[idx++] = instance;
        return NULL;
    }

    instance->last_total_angle = 0.0f;
    instance->angle_feedback_sign = 1;
    instance->angle_feedback_locked = 0;

    DJIMotorEnable(instance);
    sender_enable_flag[instance->sender_group] = 1U;
    dji_motor_instance[idx++] = instance;
    return instance;
}

/* 电流反馈来自电调；外部力矩传感器应作为独立反馈源接入。 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    RmCriticalState state;
    uint32_t now_ms;

    if (motor == NULL) {
        return;
    }
    if ((loop != ANGLE_LOOP) && (loop != SPEED_LOOP)) {
        LOGERROR("[dji_motor] loop type error, check memory access and func param");
        return;
    }
    now_ms = RmTime_NowMs();
    state = RmCritical_Enter();
    if (loop == ANGLE_LOOP)
        motor->command_mailbox.settings.angle_feedback_source = type;
    else
        motor->command_mailbox.settings.speed_feedback_source = type;
    DJIMotorRecordCommandPublication(motor, now_ms);
    RmCritical_Exit(state);
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    if (motor != NULL) {
        uint32_t now_ms = RmTime_NowMs();
        RmCriticalState state = RmCritical_Enter();
        motor->command_mailbox.working_state = MOTOR_STOP;
        DJIMotorRecordCommandPublication(motor, now_ms);
        RmCritical_Exit(state);
    }
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    if (motor != NULL) {
        uint32_t now_ms = RmTime_NowMs();
        RmCriticalState state = RmCritical_Enter();
        motor->command_mailbox.working_state = MOTOR_ENALBED;
        DJIMotorRecordCommandPublication(motor, now_ms);
        RmCritical_Exit(state);
    }
}

/* 修改电机的最外层闭环。 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    if (motor != NULL) {
        uint32_t now_ms = RmTime_NowMs();
        RmCriticalState state = RmCritical_Enter();
        motor->command_mailbox.settings.outer_loop_type = outer_loop;
        DJIMotorRecordCommandPublication(motor, now_ms);
        RmCritical_Exit(state);
    }
}

// 设置控制参考值。
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    if (motor != NULL) {
        uint32_t now_ms = RmTime_NowMs();
        RmCriticalState state = RmCritical_Enter();
        motor->command_mailbox.reference = ref;
        DJIMotorRecordCommandPublication(motor, now_ms);
        RmCritical_Exit(state);
    }
}

// 为所有电机实例计算串级 PID，并发送分组控制报文。
void DJIMotorControl()
{
    // 保存局部引用，避免在控制计算中重复索引实例表。
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // CAN 电流控制值
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DJI_Motor_Measure_s measure_snapshot;
    const DJI_Motor_Measure_s *measure;
    DJIMotorCommand command;
    uint32_t last_publish_ms;
    uint32_t command_generation;
    uint32_t now_ms = RmTime_NowMs();
    uint32_t timeout_ms = atomic_load_explicit(
        &command_timeout_ms, memory_order_acquire);
    bool command_published;
    bool command_fresh;
    float pid_measure, pid_ref;             // PID 测量值和参考值

    // 遍历所有电机实例，计算串级 PID 并填充发送报文。
    for (size_t i = 0; i < idx; ++i)
    {
        motor = dji_motor_instance[i];
        DJIMotorConsumeCommand(motor, &command, &last_publish_ms,
                               &command_generation, &command_published);
        command_fresh = (timeout_ms == 0U) ||
                        (command_published &&
                         (RmTime_ElapsedMs(now_ms, last_publish_ms) <
                          timeout_ms));
        if ((timeout_ms != 0U) && command_published && !command_fresh) {
            DJIMotorExpireCommand(motor, command_generation);
        }
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        group = motor->sender_group;
        num = motor->message_num;
        if (!DJIMotorGetMeasure(motor, &measure_snapshot)) {
            memset(sender_assignment[group].tx_buff + 2U * num, 0, 2U);
            motor->final_output = 0.0f;
            DJIMotorPublishControlSnapshot(motor);
            continue;
        }
        measure = &measure_snapshot;

        if (!command_fresh ||
            (command.working_state != MOTOR_ENALBED) ||
            !DJIMotorGlobalOutputEnabled() || !DJIMotorIsOnline(motor)) {
            if (motor->output_active != 0U) {
                DJIMotorResetPidRuntime(motor, DJI_MOTOR_PID_RESET_ALL);
            }
            motor->output_active = 0U;
            motor->final_output = 0.0f;
            memset(sender_assignment[group].tx_buff + 2U * num, 0, 2U);
            motor->last_total_angle = measure->total_angle;
            DJIMotorPublishControlSnapshot(motor);
            continue;
        }
        if (motor->output_active == 0U) {
            DJIMotorResetPidRuntime(motor, DJI_MOTOR_PID_RESET_ALL);
            motor->output_active = 1U;
        }

        pid_ref = motor_controller->pid_ref;
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        float delta_angle = measure->total_angle - motor->last_total_angle;
        if (!motor->angle_feedback_locked)
        {
            if (fabsf(measure->speed_aps) > 5.0f && fabsf(delta_angle) > 0.1f)
            {
                motor->angle_feedback_sign = (measure->speed_aps * delta_angle >= 0.0f) ? 1 : -1;
                motor->angle_feedback_locked = 1;
            }
        }
        // pid_ref 依次流过已启用的外环。
        // 仅在位置环为最外环时计算位置环输出。
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = ((command.external_input_mask &
                                DJI_MOTOR_EXTERNAL_ANGLE) != 0U)
                                  ? command.other_angle_feedback
                                  : ((motor_controller->other_angle_feedback_ptr != NULL)
                                         ? *motor_controller->other_angle_feedback_ptr
                                         : measure->total_angle);
            else
                pid_measure = measure->total_angle * motor->angle_feedback_sign; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 当速度环处于有效控制链中时计算速度环。
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += ((command.external_input_mask &
                             DJI_MOTOR_EXTERNAL_SPEED_FF) != 0U)
                               ? command.speed_feedforward
                               : ((motor_controller->speed_feedforward_ptr != NULL)
                                      ? *motor_controller->speed_feedforward_ptr
                                      : 0.0f);

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = ((command.external_input_mask &
                                DJI_MOTOR_EXTERNAL_SPEED) != 0U)
                                  ? command.other_speed_feedback
                                  : ((motor_controller->other_speed_feedback_ptr != NULL)
                                         ? *motor_controller->other_speed_feedback_ptr
                                         : measure->speed_aps);
            else // MOTOR_FEED
                pid_measure =
                    (motor_setting->speed_unit ==
                     MOTOR_SPEED_RAD_PER_SEC)
                        ? measure->speed_rad_s
                        : measure->speed_aps;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 电流环使用电调反馈，位于控制链最内层。
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += ((command.external_input_mask &
                         DJI_MOTOR_EXTERNAL_CURRENT_FF) != 0U)
                           ? command.current_feedforward
                           : ((motor_controller->current_feedforward_ptr != NULL)
                                  ? *motor_controller->current_feedforward_ptr
                                  : 0.0f);
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 电机型号决定电调命令的物理范围，应用层参数不得绕过最终保护。
        motor->final_output = DJIMotorClampOutput(motor->motor_type, pid_ref);
        set = (int16_t)motor->final_output;

        // 按电机组填入发送数据。
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff);

        motor->last_total_angle = measure->total_angle;
        DJIMotorPublishControlSnapshot(motor);
    }

    // 仅发送包含已注册电机的命令组。
    for (size_t i = 0; i < 6; ++i)
    {
        if (sender_enable_flag[i])
        {
            CANTransmit(&sender_assignment[i], 1000U);
        }
    }
}
