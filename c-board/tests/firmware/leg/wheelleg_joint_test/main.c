/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Wheel-leg joint MIT control test (DM8009P + ET08)
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"

#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_init.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "dmmotor.h"
#include "et08_remote.h"
#include "utils.h"

#define UPDATE_INTERVAL_MS 20U
#define CONTROL_INTERVAL_MS 2U
#define ENABLE_INTERVAL_MS 100U
#define TELEMETRY_INTERVAL_MS 100U
#define CMD_BUFFER_LEN 64U
#define VOFA_MAX_FLOATS 24U
#define TELEMETRY_VOFA_DEFAULT 1U

#define FRONT_LEFT_ID 1U
#define REAR_LEFT_ID 2U
#define FRONT_RIGHT_ID 3U
#define REAR_RIGHT_ID 4U
#define DM_MASTER_ID 0x00U

#define DM_P_RANGE 12.5f
#define DM_V_RANGE 45.0f
#define DM_T_RANGE 54.0f

#define RC_STICK_MAX 660.0f
#define RC_DEADZONE 30

// Mirror direction signs (adjust if needed).
#define HIP_LEFT_SIGN -1.0f
#define HIP_RIGHT_SIGN +1.0f
#define KNEE_LEFT_SIGN 1.0f
#define KNEE_RIGHT_SIGN -1.0f

// Stick rate (deg/s) to target position increment.
#define HIP_RATE_DEG_S 30.0f
#define KNEE_RATE_DEG_S 30.0f

// Knee joint limit (deg). Zero is at front limit block.
// If your direction is reversed, swap MIN/MAX signs.
#define KNEE_MIN_DEG 0.0f
#define KNEE_MAX_DEG 74.6f

// Knee software offsets (deg). Positive = more "back" if KNEE sign is positive.
#define HIP_OFFSET_DEG 0.0f
#define KNEE_LEFT_OFFSET_DEG 0.0f
#define KNEE_RIGHT_OFFSET_DEG 0.0f

// MIT gains for position hold (tune on real hardware).
// Higher Kp gives stronger holding torque (stiffer), Kd adds damping.
#define HIP_KP 60.0f
#define HIP_KD 2.5f
#define KNEE_KP 70.0f
#define KNEE_KD 3.0f
// Constant torque feed-forward (tune if still weak).
#define HIP_TFF 0.0f
#define KNEE_TFF 0.0f

typedef struct
{
    DMMotor_Handle *handle;
    float target_p;
    float kp;
    float kd;
    float t_ff;
    uint8_t enabled;
    uint32_t last_enable_ms;
    uint8_t target_inited;
} JointState;

static ET08_Ctrl_t *et08_ctrl = NULL;
static JointState front_left = {0};
static JointState rear_left = {0};
static JointState front_right = {0};
static JointState rear_right = {0};
static USARTInstance *telemetry_usart = NULL;
static uint8_t rc_online_last = 0;
static uint8_t control_ready = 0;
static uint8_t auto_zero_active = 0;
static float hip_left_target = 0.0f;
static float hip_right_target = 0.0f;
static float knee_left_target = 0.0f;
static float knee_right_target = 0.0f;
static uint8_t split_ctrl_mode = 0;
static float hip_left_offset = 0.0f;
static float hip_right_offset = 0.0f;
static float knee_left_offset = 0.0f;
static float knee_right_offset = 0.0f;
static char cmd_buffer[CMD_BUFFER_LEN];
static uint8_t cmd_index = 0;
static uint8_t cmd_active = 0;
static uint8_t vofa_enabled = TELEMETRY_VOFA_DEFAULT;

void SystemClock_Config(void);
void Error_Handler(void);

static void TelemetryRxCallback(void);
static void ProcessCommand(const char *cmd);
static void ApplyOffsetsFromDefaults(void);
static void ApplyHipGains(void);
static void ApplyKneeGains(void);

static void Debug_DisableWatchdogs(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

static float ClampFloat(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static void TelemetryInit(void)
{
    USART_Init_Config_s config = {
        .module_callback = TelemetryRxCallback,
        .recv_buff_size = 1,
        .usart_handle = &huart6,
    };
    telemetry_usart = USARTRegister(&config);
}

static void TelemetrySend(const char *msg)
{
    if (!msg || !telemetry_usart)
        return;
    USARTSend(telemetry_usart, (uint8_t *)msg, (uint16_t)strlen(msg), USART_TRANSFER_BLOCKING);
}

static int16_t ApplyDeadzone(int16_t value, int16_t deadzone)
{
    if (value > deadzone)
        return value - deadzone;
    if (value < -deadzone)
        return value + deadzone;
    return 0;
}

static void JointInit(JointState *joint, uint8_t motor_id, float kp, float kd, float t_ff)
{
    DMMotor_InitConfig config = {
        .can_handle = &hcan1,
        .motor_id = motor_id,
        .master_id = DM_MASTER_ID,
        .auto_clear_error = true,
        .auto_enable_mit = false,
        .auto_zero_position = false,
        .use_shared_feedback_id = true,
        .position_range = DM_P_RANGE,
        .velocity_range = DM_V_RANGE,
        .torque_range = DM_T_RANGE,
    };

    joint->handle = DMMotor_Init(&config);
    joint->target_p = 0.0f;
    joint->kp = kp;
    joint->kd = kd;
    joint->t_ff = t_ff;
    joint->enabled = 0U;
    joint->last_enable_ms = 0U;
    joint->target_inited = 0U;
}

static void DisableJoint(JointState *joint)
{
    if (!joint || !joint->handle)
        return;
    DMMotor_Disable(joint->handle, DM_MODE_MIT);
    joint->enabled = 0U;
}

static void SyncJointTargetToFeedback(JointState *joint)
{
    if (!joint || !joint->handle)
        return;
    const DMMotor_Feedback *fb = DMMotor_GetFeedback(joint->handle);
    if (!fb)
        return;
    joint->target_p = fb->position_rad;
    joint->target_inited = 1U;
}

static void InitTargetsToZero(void)
{
    hip_left_target = 0.0f;
    hip_right_target = 0.0f;
    knee_left_target = 0.0f;
    knee_right_target = 0.0f;

    front_left.target_p = 0.0f;
    front_right.target_p = 0.0f;
    rear_left.target_p = 0.0f;
    rear_right.target_p = 0.0f;

    front_left.target_inited = 1U;
    front_right.target_inited = 1U;
    rear_left.target_inited = 1U;
    rear_right.target_inited = 1U;

    control_ready = 1;
    rc_online_last = 1;
    auto_zero_active = 1;
}

static void UpdateJointMIT(JointState *joint)
{
    if (!joint || !joint->handle)
        return;

    uint32_t now = HAL_GetTick();
    const DMMotor_Feedback *fb = DMMotor_GetFeedback(joint->handle);

    if (!joint->target_inited)
        return;

    if (!joint->enabled)
    {
        DMMotor_ClearError(joint->handle, DM_MODE_MIT);
        DMMotor_Enable(joint->handle, DM_MODE_MIT);
        joint->enabled = 1U;
        joint->last_enable_ms = now;
    }

    if (fb && fb->error_state != 0)
    {
        DMMotor_ClearError(joint->handle, DM_MODE_MIT);
        DMMotor_Enable(joint->handle, DM_MODE_MIT);
        joint->last_enable_ms = now;
    }

    if (now - joint->last_enable_ms > ENABLE_INTERVAL_MS)
    {
        DMMotor_Enable(joint->handle, DM_MODE_MIT);
        joint->last_enable_ms = now;
    }

    DMMotor_SendMIT(joint->handle, joint->target_p, 0.0f, joint->kp, joint->kd, joint->t_ff);
}

static void ApplyHipGains(void)
{
    front_left.kp = HIP_KP;
    front_left.kd = HIP_KD;
    front_left.t_ff = HIP_TFF;

    front_right.kp = HIP_KP;
    front_right.kd = HIP_KD;
    front_right.t_ff = HIP_TFF;
}

static void ApplyKneeGains(void)
{
    rear_left.kp = KNEE_KP;
    rear_left.kd = KNEE_KD;
    rear_left.t_ff = KNEE_TFF;

    rear_right.kp = KNEE_KP;
    rear_right.kd = KNEE_KD;
    rear_right.t_ff = KNEE_TFF;
}

static void ApplyOffsetsFromDefaults(void)
{
    hip_left_offset = DM_DegToRad(HIP_OFFSET_DEG);
    hip_right_offset = DM_DegToRad(HIP_OFFSET_DEG);
    knee_left_offset = DM_DegToRad(KNEE_LEFT_OFFSET_DEG);
    knee_right_offset = DM_DegToRad(KNEE_RIGHT_OFFSET_DEG);
}

static void ProcessCommand(const char *cmd)
{
    if (!cmd || cmd[0] == '\0')
        return;

    const char *colon = strchr(cmd, ':');
    if (!colon)
        return;

    char key[16];
    size_t key_len = (size_t)(colon - cmd);
    if (key_len >= sizeof(key))
        key_len = sizeof(key) - 1U;
    memcpy(key, cmd, key_len);
    key[key_len] = '\0';

    char *endptr = NULL;
    float value = strtof(colon + 1, &endptr);
    if (endptr == colon + 1)
        return;

    if (strcmp(key, "H_OFF") == 0)
    {
        hip_left_offset = DM_DegToRad(value);
        hip_right_offset = DM_DegToRad(value);
    }
    else if (strcmp(key, "K_OFF") == 0)
    {
        knee_left_offset = DM_DegToRad(value);
        knee_right_offset = DM_DegToRad(value);
    }
    else if (strcmp(key, "L_K_OFF") == 0)
    {
        knee_left_offset = DM_DegToRad(value);
    }
    else if (strcmp(key, "R_K_OFF") == 0)
    {
        knee_right_offset = DM_DegToRad(value);
    }
    else if (strcmp(key, "H_KP") == 0)
    {
        front_left.kp = value;
        front_right.kp = value;
    }
    else if (strcmp(key, "H_KD") == 0)
    {
        front_left.kd = value;
        front_right.kd = value;
    }
    else if (strcmp(key, "K_KP") == 0)
    {
        rear_left.kp = value;
        rear_right.kp = value;
    }
    else if (strcmp(key, "K_KD") == 0)
    {
        rear_left.kd = value;
        rear_right.kd = value;
    }
    else if (strcmp(key, "H_TFF") == 0)
    {
        front_left.t_ff = value;
        front_right.t_ff = value;
    }
    else if (strcmp(key, "K_TFF") == 0)
    {
        rear_left.t_ff = value;
        rear_right.t_ff = value;
    }
    else if (strcmp(key, "VOFA") == 0)
    {
        vofa_enabled = (value != 0.0f) ? 1U : 0U;
    }
    else
    {
        return;
    }

    if (telemetry_usart && !vofa_enabled)
    {
        char buffer[80];
        safe_snprintf(buffer, sizeof(buffer), "cmd ok: %s=%.3f\r\n", key, value);
        TelemetrySend(buffer);
    }
}

static void TelemetryRxCallback(void)
{
    if (!telemetry_usart)
        return;

    size_t len = telemetry_usart->recv_buff_size;
    for (size_t i = 0; i < len; ++i)
    {
        char c = (char)telemetry_usart->recv_buff[i];
        if (c == '*')
        {
            cmd_active = 1;
            cmd_index = 0;
            continue;
        }
        if (c == '#')
        {
            if (cmd_active)
            {
                cmd_buffer[cmd_index] = '\0';
                ProcessCommand(cmd_buffer);
            }
            cmd_active = 0;
            cmd_index = 0;
            continue;
        }
        if (!cmd_active)
        {
            continue;
        }
        if (cmd_index < CMD_BUFFER_LEN - 1U)
        {
            cmd_buffer[cmd_index++] = c;
        }
    }
}

static void TelemetrySendVofaFrame(const float *values, size_t count)
{
    if (!telemetry_usart || !values || count == 0)
        return;

    uint8_t buffer[4 * VOFA_MAX_FLOATS + 4];
    if (count > VOFA_MAX_FLOATS)
        count = VOFA_MAX_FLOATS;

    size_t offset = 0;
    for (size_t i = 0; i < count; ++i)
    {
        memcpy(&buffer[offset], &values[i], sizeof(float));
        offset += sizeof(float);
    }

    union
    {
        uint32_t u;
        float f;
    } inf_marker;
    inf_marker.u = 0x7F800000u;
    memcpy(&buffer[offset], &inf_marker.f, sizeof(float));
    offset += sizeof(float);

    USARTSend(telemetry_usart, buffer, (uint16_t)offset, USART_TRANSFER_BLOCKING);
}

static uint8_t IsSaDown(const ET08_Ctrl_t *ctrl)
{
    if (!ctrl)
        return 0;

    if (ctrl->switch_sa_sb_state != 0xFF)
    {
        return (ctrl->switch_sa_sb_state >= 3U) ? 1U : 0U;
    }

    if (ctrl->switch_sa_sb_centered < -200)
        return 1U;
    return 0U;
}

static void ProcessRemoteControl(float dt_sec)
{
    if (!ET08_IsOnline() || et08_ctrl == NULL)
    {
        if (!auto_zero_active)
        {
            DisableJoint(&front_left);
            DisableJoint(&rear_left);
            DisableJoint(&front_right);
            DisableJoint(&rear_right);
            rc_online_last = 0;
            control_ready = 0;
        }
        return;
    }

    if (auto_zero_active)
    {
        auto_zero_active = 0;
    }

    if (!rc_online_last)
    {
        SyncJointTargetToFeedback(&front_left);
        SyncJointTargetToFeedback(&rear_left);
        SyncJointTargetToFeedback(&front_right);
        SyncJointTargetToFeedback(&rear_right);

        // Serial leg mapping: hip motor controls hip angle; knee motor angle ~= hip + knee.
        hip_left_target = front_left.target_p;
        hip_right_target = front_right.target_p;
        knee_left_target = rear_left.target_p - front_left.target_p;
        knee_right_target = rear_right.target_p - front_right.target_p;
        if (front_left.target_inited && rear_left.target_inited &&
            front_right.target_inited && rear_right.target_inited)
        {
            control_ready = 1;
        }
        rc_online_last = 1;
    }

    split_ctrl_mode = IsSaDown(et08_ctrl);

    int16_t left_y = ApplyDeadzone(et08_ctrl->left.y, RC_DEADZONE);
    int16_t right_y = ApplyDeadzone(et08_ctrl->right.y, RC_DEADZONE);
    int16_t left_x = ApplyDeadzone(et08_ctrl->left.x, RC_DEADZONE);
    int16_t right_x = ApplyDeadzone(et08_ctrl->right.x, RC_DEADZONE);

    float hip_left_rate = 0.0f;
    float hip_right_rate = 0.0f;
    float knee_left_rate = 0.0f;
    float knee_right_rate = 0.0f;

    if (split_ctrl_mode)
    {
        hip_left_rate = (float)left_y / RC_STICK_MAX * HIP_RATE_DEG_S;
        hip_right_rate = (float)right_y / RC_STICK_MAX * HIP_RATE_DEG_S;
        knee_left_rate = (float)left_x / RC_STICK_MAX * KNEE_RATE_DEG_S;
        knee_right_rate = (float)right_x / RC_STICK_MAX * KNEE_RATE_DEG_S;
    }
    else
    {
        hip_left_rate = (float)left_y / RC_STICK_MAX * HIP_RATE_DEG_S;
        hip_right_rate = (float)left_y / RC_STICK_MAX * HIP_RATE_DEG_S;
        knee_left_rate = (float)right_y / RC_STICK_MAX * KNEE_RATE_DEG_S;
        knee_right_rate = (float)right_y / RC_STICK_MAX * KNEE_RATE_DEG_S;
    }

    float hip_left_step = DM_DegToRad(hip_left_rate * dt_sec);
    float hip_right_step = DM_DegToRad(hip_right_rate * dt_sec);
    float knee_left_step = DM_DegToRad(knee_left_rate * dt_sec);
    float knee_right_step = DM_DegToRad(knee_right_rate * dt_sec);

    hip_left_target += HIP_LEFT_SIGN * hip_left_step;
    hip_right_target += HIP_RIGHT_SIGN * hip_right_step;
    knee_left_target += KNEE_LEFT_SIGN * knee_left_step;
    knee_right_target += KNEE_RIGHT_SIGN * knee_right_step;

    // Hip joints: clamp to avoid wrap-around reversal near +/-P_RANGE.
    hip_left_target = ClampFloat(hip_left_target, -DM_P_RANGE, DM_P_RANGE);
    hip_right_target = ClampFloat(hip_right_target, -DM_P_RANGE, DM_P_RANGE);

    // Knee limits relative to the front limit block (zero).
    float knee_min = DM_DegToRad(KNEE_MIN_DEG);
    float knee_max = DM_DegToRad(KNEE_MAX_DEG);
    float knee_left_cmd = ClampFloat(knee_left_target + knee_left_offset, knee_min, knee_max);
    float knee_right_cmd = ClampFloat(knee_right_target + knee_right_offset, knee_min, knee_max);
    knee_left_target = knee_left_cmd - knee_left_offset;
    knee_right_target = knee_right_cmd - knee_right_offset;

    // Motor target mapping for serial leg: rear motor = hip + knee.
    float hip_left_cmd = hip_left_target + hip_left_offset;
    float hip_right_cmd = hip_right_target + hip_right_offset;
    while (hip_left_cmd > DM_P_RANGE) hip_left_cmd -= 2.0f * DM_P_RANGE;
    while (hip_left_cmd < -DM_P_RANGE) hip_left_cmd += 2.0f * DM_P_RANGE;
    while (hip_right_cmd > DM_P_RANGE) hip_right_cmd -= 2.0f * DM_P_RANGE;
    while (hip_right_cmd < -DM_P_RANGE) hip_right_cmd += 2.0f * DM_P_RANGE;

    front_left.target_p = hip_left_cmd;
    front_right.target_p = hip_right_cmd;
    rear_left.target_p = hip_left_cmd + knee_left_cmd;
    rear_right.target_p = hip_right_cmd + knee_right_cmd;

    rear_left.target_p = ClampFloat(rear_left.target_p, -DM_P_RANGE, DM_P_RANGE);
    rear_right.target_p = ClampFloat(rear_right.target_p, -DM_P_RANGE, DM_P_RANGE);
}

static void TelemetryTick(void)
{
    if (!telemetry_usart)
        return;

    const DMMotor_Feedback *front_l = front_left.handle ? DMMotor_GetFeedback(front_left.handle) : NULL;
    const DMMotor_Feedback *rear_l = rear_left.handle ? DMMotor_GetFeedback(rear_left.handle) : NULL;
    const DMMotor_Feedback *front_r = front_right.handle ? DMMotor_GetFeedback(front_right.handle) : NULL;
    const DMMotor_Feedback *rear_r = rear_right.handle ? DMMotor_GetFeedback(rear_right.handle) : NULL;

    float front_l_pos = front_l ? front_l->position_rad : 0.0f;
    float rear_l_pos = rear_l ? rear_l->position_rad : 0.0f;
    float front_r_pos = front_r ? front_r->position_rad : 0.0f;
    float rear_r_pos = rear_r ? rear_r->position_rad : 0.0f;
    float front_l_tq = front_l ? front_l->torque : 0.0f;
    float rear_l_tq = rear_l ? rear_l->torque : 0.0f;
    float front_r_tq = front_r ? front_r->torque : 0.0f;
    float rear_r_tq = rear_r ? rear_r->torque : 0.0f;
    float front_l_temp = front_l ? front_l->mos_temp : 0.0f;
    float rear_l_temp = rear_l ? rear_l->mos_temp : 0.0f;
    float front_r_temp = front_r ? front_r->mos_temp : 0.0f;
    float rear_r_temp = rear_r ? rear_r->mos_temp : 0.0f;

    if (vofa_enabled)
    {
        float values[] = {
            front_l_pos, rear_l_pos, front_r_pos, rear_r_pos,
            hip_left_target, hip_right_target, knee_left_target, knee_right_target,
            front_l_tq, rear_l_tq, front_r_tq, rear_r_tq,
            front_l_temp, rear_l_temp, front_r_temp, rear_r_temp,
            (float)(front_l ? front_l->error_state : 0xFF),
            (float)(rear_l ? rear_l->error_state : 0xFF),
            (float)(front_r ? front_r->error_state : 0xFF),
            (float)(rear_r ? rear_r->error_state : 0xFF),
        };
        TelemetrySendVofaFrame(values, sizeof(values) / sizeof(values[0]));
        return;
    }

    char buffer[160];
    safe_snprintf(buffer, sizeof(buffer),
                  "pos FL:%.3f RL:%.3f FR:%.3f RR:%.3f | tq FL:%.1f RL:%.1f FR:%.1f RR:%.1f | tM FL:%.1f RL:%.1f FR:%.1f RR:%.1f | hip[%.3f %.3f] knee[%.3f %.3f] | err[%02X %02X %02X %02X]\r\n",
                  front_l_pos, rear_l_pos, front_r_pos, rear_r_pos,
                  front_l_tq, rear_l_tq, front_r_tq, rear_r_tq,
                  front_l_temp, rear_l_temp, front_r_temp, rear_r_temp,
                  hip_left_target, hip_right_target, knee_left_target, knee_right_target,
                  (unsigned)(front_l ? front_l->error_state : 0xFF),
                  (unsigned)(rear_l ? rear_l->error_state : 0xFF),
                  (unsigned)(front_r ? front_r->error_state : 0xFF),
                  (unsigned)(rear_r ? rear_r->error_state : 0xFF));
    TelemetrySend(buffer);
}

int main(void)
{
    HAL_Init();
    Debug_DisableWatchdogs();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();

    BSPInit();

    TelemetryInit();
    ApplyOffsetsFromDefaults();
    JointInit(&front_left, FRONT_LEFT_ID, HIP_KP, HIP_KD, HIP_TFF);
    JointInit(&rear_left, REAR_LEFT_ID, KNEE_KP, KNEE_KD, KNEE_TFF);
    JointInit(&front_right, FRONT_RIGHT_ID, HIP_KP, HIP_KD, HIP_TFF);
    JointInit(&rear_right, REAR_RIGHT_ID, KNEE_KP, KNEE_KD, KNEE_TFF);

    ApplyHipGains();
    ApplyKneeGains();

    et08_ctrl = ET08_Init(&huart3);

    LOGINFO("[joint_test] ET08 left.y -> hips (front+rear), right.y -> knees (rear only)");
    LOGINFO("[joint_test] ids: frontL=1 rearL=2 frontR=3 rearR=4 (CAN1)");
    LOGINFO("[joint_test] auto-zero on boot enabled");
    LOGINFO("[joint_test] SA down => split control (L/R sticks: Y=hip X=knee)");
    LOGINFO("[joint_test] knee offset deg: left=%.2f right=%.2f",
            (double)KNEE_LEFT_OFFSET_DEG, (double)KNEE_RIGHT_OFFSET_DEG);

    InitTargetsToZero();

    uint32_t last_update_tick = 0;
    uint32_t last_control_tick = 0;
    uint32_t last_telemetry_tick = 0;

    while (1)
    {
        DaemonTask();

        uint32_t now = HAL_GetTick();
        if (now - last_update_tick >= UPDATE_INTERVAL_MS)
        {
            last_update_tick = now;
            float dt = UPDATE_INTERVAL_MS / 1000.0f;
            ProcessRemoteControl(dt);
        }

        if (now - last_control_tick >= CONTROL_INTERVAL_MS)
        {
            last_control_tick = now;
            if ((ET08_IsOnline() && control_ready) || auto_zero_active)
            {
                UpdateJointMIT(&front_left);
                UpdateJointMIT(&rear_left);
                UpdateJointMIT(&front_right);
                UpdateJointMIT(&rear_right);
            }
        }

        if (now - last_telemetry_tick >= TELEMETRY_INTERVAL_MS)
        {
            last_telemetry_tick = now;
            TelemetryTick();
        }

        HAL_Delay(1);
    }
}

void SystemClock_Config(void)
{
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

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14)
    {
        HAL_IncTick();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
