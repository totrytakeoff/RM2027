#include "dmmotor.h"

#include "bsp_dwt.h"
#include "general_def.h"
#include <stdlib.h>
#include <string.h>

#define DM_DEFAULT_P_MAX 12.5f
#define DM_DEFAULT_V_MAX 45.0f
#define DM_DEFAULT_T_MAX 54.0f
#define DM_DEFAULT_KP_MAX 500.0f
#define DM_DEFAULT_KD_MAX 5.0f
#define DM_SHARED_BUS_MAX 4U
#define DM_SHARED_MOTOR_MAX 16U

static float clampf(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    float scaled = (x - offset) * ((float)((1U << bits) - 1U)) / span;
    if (scaled < 0.0f)
        scaled = 0.0f;
    float max_val = (float)((1U << bits) - 1U);
    if (scaled > max_val)
        scaled = max_val;
    return (uint16_t)scaled;
}

static float uint_to_float(int value, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return ((float)value) * span / ((float)((1U << bits) - 1U)) + x_min;
}

typedef struct DMMotor_Handle DMMotor_Handle;

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint16_t rx_id;
    CANInstance *can_instance;
    DMMotor_Handle *motor_map[DM_SHARED_MOTOR_MAX];
} DMMotor_SharedBus;

struct DMMotor_Handle
{
    CANInstance *command_can;
    DMMotor_InitConfig config;
    DMMotor_Feedback feedback;
    DMMotor_SharedBus *shared_bus;
    uint8_t mode_enabled[4];
};

static DMMotor_SharedBus dm_shared_bus[DM_SHARED_BUS_MAX];
static uint8_t dm_shared_bus_count = 0;

static void DMMotor_SendFrame(DMMotor_Handle *motor, uint32_t id, const uint8_t *payload, uint8_t len)
{
    uint8_t backup_dlc = motor->command_can->txconf.DLC;
    uint32_t backup_id = motor->command_can->txconf.StdId;

    CANSetDLC(motor->command_can, len);
    motor->command_can->txconf.StdId = id;
    memcpy(motor->command_can->tx_buff, payload, len);
    CANTransmit(motor->command_can, 1000U);

    motor->command_can->txconf.StdId = backup_id;
    CANSetDLC(motor->command_can, backup_dlc);
}

static void DMMotor_SendModeCommand(DMMotor_Handle *motor, DMMotor_Mode mode, uint8_t command_byte)
{
    uint8_t frame[8];
    memset(frame, 0xFF, sizeof(frame));
    frame[7] = command_byte;
    DMMotor_SendFrame(motor, motor->config.motor_id + mode, frame, sizeof(frame));
}

static void DMMotor_ParseFeedback(DMMotor_Handle *motor, const uint8_t *rx)
{
    /**
     * 反馈帧格式（与上位机抓到的表一致）：
     *   D0[7:4] ERR/STATE, D0[3:0] ID
     *   D1:D2   POS (16bit)
     *   D3:D4   VEL (12bit，D3=VEL[11:4], D4[7:4]=VEL[3:0])
     *   D4:D5   T   (12bit，D4[3:0]=T[11:8], D5=T[7:0])
     *   D6      T_MOS
     *   D7      T_Rotor
     */
    motor->feedback.motor_id = (uint8_t)(rx[0] & 0x0F);
    motor->feedback.error_state = (uint8_t)(rx[0] >> 4);

    uint16_t pos_raw = ((uint16_t)rx[1] << 8) | rx[2];
    uint16_t vel_raw = ((uint16_t)rx[3] << 4) | (rx[4] >> 4);
    uint16_t tor_raw = ((uint16_t)(rx[4] & 0x0F) << 8) | rx[5];

    motor->feedback.position_rad =
        uint_to_float(pos_raw, -motor->config.position_range, motor->config.position_range, 16);
    motor->feedback.velocity_rad_s =
        uint_to_float(vel_raw, -motor->config.velocity_range, motor->config.velocity_range, 12);
    motor->feedback.torque =
        uint_to_float(tor_raw, -motor->config.torque_range, motor->config.torque_range, 12);
    motor->feedback.mos_temp = (float)rx[6];
    motor->feedback.rotor_temp = (float)rx[7];
}

static void DMMotor_Decode(CANInstance *instance)
{
    if (!instance || instance->rx_len < 8)
        return;
    DMMotor_Handle *motor = (DMMotor_Handle *)instance->id;
    if (!motor)
        return;
    DMMotor_ParseFeedback(motor, instance->rx_buff);
}

static void DMMotor_SharedDecode(CANInstance *instance)
{
    if (!instance || instance->rx_len < 8)
        return;
    DMMotor_SharedBus *bus = (DMMotor_SharedBus *)instance->id;
    if (!bus)
        return;

    const uint8_t *rx = instance->rx_buff;
    uint8_t motor_id = (uint8_t)(rx[0] & 0x0F);
    if (motor_id == 0 || motor_id >= DM_SHARED_MOTOR_MAX)
        return;
    DMMotor_Handle *motor = bus->motor_map[motor_id];
    if (!motor)
        return;
    DMMotor_ParseFeedback(motor, rx);
}

static DMMotor_SharedBus *DMMotor_FindSharedBus(CAN_HandleTypeDef *can_handle, uint16_t rx_id)
{
    for (uint8_t i = 0; i < dm_shared_bus_count; ++i) {
        if (dm_shared_bus[i].can_handle == can_handle && dm_shared_bus[i].rx_id == rx_id)
            return &dm_shared_bus[i];
    }
    return NULL;
}

static DMMotor_SharedBus *DMMotor_EnsureSharedBus(CAN_HandleTypeDef *can_handle, uint16_t rx_id)
{
    DMMotor_SharedBus *bus = DMMotor_FindSharedBus(can_handle, rx_id);
    if (bus)
        return bus;
    if (dm_shared_bus_count >= DM_SHARED_BUS_MAX)
        return NULL;

    bus = &dm_shared_bus[dm_shared_bus_count++];
    memset(bus, 0, sizeof(*bus));
    bus->can_handle = can_handle;
    bus->rx_id = rx_id;

    CAN_Init_Config_s can_cfg = {
        .can_handle = can_handle,
        .tx_id = rx_id,
        .rx_id = rx_id,
        .can_module_callback = DMMotor_SharedDecode,
        .id = bus,
    };
    bus->can_instance = CANRegister(&can_cfg);
    return bus;
}

static uint16_t DMMotor_CalcFeedbackStdId(const DMMotor_InitConfig *config, bool *use_shared)
{
    if (use_shared)
        *use_shared = false;
    if (!config)
        return 0;
    if (config->use_shared_feedback_id) {
        if (use_shared)
            *use_shared = true;
        return config->master_id;
    }
    if (config->master_id <= 0x0Fu) {
        return (uint16_t)((config->master_id << 4) | (config->motor_id & 0x0Fu));
    }
    return config->master_id;
}

DMMotor_Handle *DMMotor_Init(const DMMotor_InitConfig *config)
{
    if (config == NULL || config->can_handle == NULL) {
        return NULL;
    }

    DMMotor_Handle *motor = (DMMotor_Handle *)malloc(sizeof(DMMotor_Handle));
    memset(motor, 0, sizeof(DMMotor_Handle));
    motor->config = *config;

    if (motor->config.position_range <= 0.0f)
        motor->config.position_range = DM_DEFAULT_P_MAX;
    if (motor->config.velocity_range <= 0.0f)
        motor->config.velocity_range = DM_DEFAULT_V_MAX;
    if (motor->config.torque_range <= 0.0f)
        motor->config.torque_range = DM_DEFAULT_T_MAX;
    if (motor->config.kp_max <= 0.0f)
        motor->config.kp_max = DM_DEFAULT_KP_MAX;
    if (motor->config.kd_max <= 0.0f)
        motor->config.kd_max = DM_DEFAULT_KD_MAX;

    bool use_shared_feedback = false;
    const uint16_t feedback_std_id = DMMotor_CalcFeedbackStdId(&motor->config, &use_shared_feedback);
    motor->shared_bus = NULL;

    if (use_shared_feedback) {
        DMMotor_SharedBus *bus = DMMotor_EnsureSharedBus(motor->config.can_handle, feedback_std_id);
        if (!bus) {
            free(motor);
            return NULL;
        }
        uint8_t map_id = (uint8_t)(motor->config.motor_id & 0x0F);
        if (map_id < DM_SHARED_MOTOR_MAX)
            bus->motor_map[map_id] = motor;
        motor->shared_bus = bus;
    }

    CAN_Init_Config_s can_cfg = {
        .can_handle = motor->config.can_handle,
        .tx_id = motor->config.motor_id,
        .rx_id = use_shared_feedback ? motor->config.motor_id : feedback_std_id,
        .can_module_callback = use_shared_feedback ? NULL : DMMotor_Decode,
        .id = motor,
    };
    motor->command_can = CANRegister(&can_cfg);

    if (motor->config.auto_clear_error) {
        DMMotor_ClearError(motor, DM_MODE_MIT);
        DWT_Delay(0.01f);
    }
    if (motor->config.auto_zero_position) {
        DMMotor_SaveZero(motor, DM_MODE_MIT);
        DWT_Delay(0.01f);
    }
    if (motor->config.auto_enable_mit) {
        DMMotor_Enable(motor, DM_MODE_MIT);
    }

    return motor;
}

void DMMotor_DeInit(DMMotor_Handle *motor)
{
    if (!motor)
        return;
    if (motor->shared_bus) {
        uint8_t map_id = (uint8_t)(motor->config.motor_id & 0x0F);
        if (map_id < DM_SHARED_MOTOR_MAX && motor->shared_bus->motor_map[map_id] == motor)
            motor->shared_bus->motor_map[map_id] = NULL;
    }
    free(motor);
}

const DMMotor_Feedback *DMMotor_GetFeedback(const DMMotor_Handle *motor)
{
    if (!motor)
        return NULL;
    return &motor->feedback;
}

void DMMotor_ClearError(DMMotor_Handle *motor, DMMotor_Mode mode)
{
    if (!motor)
        return;
    DMMotor_SendModeCommand(motor, mode, 0xFB);
}

void DMMotor_Enable(DMMotor_Handle *motor, DMMotor_Mode mode)
{
    if (!motor)
        return;
    DMMotor_SendModeCommand(motor, mode, 0xFC);
    motor->mode_enabled[mode >> 8] = 1;
}

void DMMotor_Disable(DMMotor_Handle *motor, DMMotor_Mode mode)
{
    if (!motor)
        return;
    DMMotor_SendModeCommand(motor, mode, 0xFD);
    motor->mode_enabled[mode >> 8] = 0;
}

void DMMotor_SaveZero(DMMotor_Handle *motor, DMMotor_Mode mode)
{
    if (!motor)
        return;
    DMMotor_SendModeCommand(motor, mode, 0xFE);
}

void DMMotor_SendMIT(DMMotor_Handle *motor,
                     float position_rad,
                     float velocity_rad_s,
                     float kp,
                     float kd,
                     float torque)
{
    if (!motor)
        return;

    position_rad = clampf(position_rad, -motor->config.position_range, motor->config.position_range);
    velocity_rad_s = clampf(velocity_rad_s, -motor->config.velocity_range, motor->config.velocity_range);
    torque = clampf(torque, -motor->config.torque_range, motor->config.torque_range);
    kp = clampf(kp, 0.0f, motor->config.kp_max);
    kd = clampf(kd, 0.0f, motor->config.kd_max);

    uint16_t pos_uint = float_to_uint(position_rad, -motor->config.position_range, motor->config.position_range, 16);
    uint16_t vel_uint = float_to_uint(velocity_rad_s, -motor->config.velocity_range, motor->config.velocity_range, 12);
    uint16_t torque_uint = float_to_uint(torque, -motor->config.torque_range, motor->config.torque_range, 12);
    uint16_t kp_uint = float_to_uint(kp, 0.0f, motor->config.kp_max, 12);
    uint16_t kd_uint = float_to_uint(kd, 0.0f, motor->config.kd_max, 12);

    uint8_t frame[8];
    frame[0] = (uint8_t)(pos_uint >> 8);
    frame[1] = (uint8_t)(pos_uint);
    frame[2] = (uint8_t)(vel_uint >> 4);
    frame[3] = (uint8_t)(((vel_uint & 0x0F) << 4) | (kp_uint >> 8));
    frame[4] = (uint8_t)(kp_uint);
    frame[5] = (uint8_t)(kd_uint >> 4);
    frame[6] = (uint8_t)(((kd_uint & 0x0F) << 4) | (torque_uint >> 8));
    frame[7] = (uint8_t)(torque_uint);

    DMMotor_SendFrame(motor, motor->config.motor_id + DM_MODE_MIT, frame, sizeof(frame));
}

void DMMotor_SendSpeed(DMMotor_Handle *motor, float speed_rad_s)
{
    if (!motor)
        return;
    speed_rad_s = clampf(speed_rad_s, -motor->config.velocity_range, motor->config.velocity_range);
    uint8_t frame[4];
    memcpy(frame, &speed_rad_s, sizeof(float));
    DMMotor_SendFrame(motor, motor->config.motor_id + DM_MODE_SPEED, frame, sizeof(frame));
}

void DMMotor_SendPosition(DMMotor_Handle *motor, float position_rad, float max_speed_rad_s)
{
    if (!motor)
        return;
    uint8_t frame[8];
    memcpy(frame, &position_rad, sizeof(float));
    memcpy(frame + 4, &max_speed_rad_s, sizeof(float));
    DMMotor_SendFrame(motor, motor->config.motor_id + DM_MODE_POSITION, frame, sizeof(frame));
}

void DMMotor_SendMixed(DMMotor_Handle *motor, float position_rad, float velocity_rad_s, float current)
{
    if (!motor)
        return;

    uint8_t frame[8];
    memcpy(frame, &position_rad, sizeof(float));
    uint16_t vel_u16 = (uint16_t)(velocity_rad_s * 100.0f);
    uint16_t cur_u16 = (uint16_t)(current * 10000.0f);
    frame[4] = (uint8_t)(vel_u16 & 0xFF);
    frame[5] = (uint8_t)(vel_u16 >> 8);
    frame[6] = (uint8_t)(cur_u16 & 0xFF);
    frame[7] = (uint8_t)(cur_u16 >> 8);

    DMMotor_SendFrame(motor, motor->config.motor_id + DM_MODE_MIXED, frame, sizeof(frame));
}

void DMMotor_RequestRegister(DMMotor_Handle *motor, uint8_t reg)
{
    if (!motor)
        return;
    uint8_t can_id_l = motor->config.motor_id & 0xFF;
    uint8_t can_id_h = (motor->config.motor_id >> 8) & 0x07;
    uint8_t frame[4] = {can_id_l, can_id_h, 0x33, reg};
    DMMotor_SendFrame(motor, 0x7FF, frame, sizeof(frame));
}

void DMMotor_WriteRegister(DMMotor_Handle *motor, uint8_t reg, const uint8_t value[4])
{
    if (!motor || value == NULL)
        return;
    uint8_t can_id_l = motor->config.motor_id & 0x0F;
    uint8_t can_id_h = (motor->config.motor_id >> 4) & 0x0F;
    uint8_t frame[8] = {can_id_l, can_id_h, 0x55, reg, value[0], value[1], value[2], value[3]};
    DMMotor_SendFrame(motor, 0x7FF, frame, sizeof(frame));
}

void DMMotor_SaveRegisters(DMMotor_Handle *motor)
{
    if (!motor)
        return;
    uint8_t can_id_l = motor->config.motor_id & 0xFF;
    uint8_t can_id_h = (motor->config.motor_id >> 8) & 0x07;
    uint8_t frame[4] = {can_id_l, can_id_h, 0xAA, 0x01};
    DMMotor_SendFrame(motor, 0x7FF, frame, sizeof(frame));
}
