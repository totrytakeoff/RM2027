#include "tuning_telemetry.h"

#include <string.h>

#include "chassis.h"
#include "robot_config.h"
#include "gimbal.h"
#include "shoot.h"
#include "rm_time.h"
#include "usart.h"

enum {
    TUNING_MAX_CHANNEL_COUNT = 64U,
    TUNING_TAIL_SIZE = 4U,
    TUNING_MAX_FRAME_SIZE =
        (TUNING_MAX_CHANNEL_COUNT * (uint32_t)sizeof(float)) +
        TUNING_TAIL_SIZE,
};

static uint8_t tuning_tx_frame[TUNING_MAX_FRAME_SIZE]
    __attribute__((aligned(4)));
static bool tuning_initialized;
static uint32_t tuning_sent_count;
static uint32_t tuning_dropped_count;

static uint16_t GetGimbalAxisChannels(bool pitch_axis, float channels[])
{
    ChassisTuningSnapshot chassis = {0};
    GimbalTuningSnapshot gimbal = {0};
    const GimbalAxisTuningSnapshot *axis;
    uint16_t index = 0U;

    if (channels == NULL) {
        return 0U;
    }
    (void)Chassis_GetTuningSnapshot(&chassis);
    (void)Gimbal_GetTuningSnapshot(&gimbal);
    axis = pitch_axis ? &gimbal.pitch : &gimbal.yaw;

    channels[index++] = (float)RmTime_NowMs();
    channels[index++] = (float)axis->control_mode;
    channels[index++] = axis->operator_speed_command_deg_s;
    channels[index++] = axis->hold_target_deg;
    channels[index++] = axis->imu_angle_deg;
    channels[index++] = axis->angle_error_deg;
    channels[index++] = axis->imu_gyro_deg_s;
    channels[index++] = axis->angle_p_deg_s;
    channels[index++] = axis->angle_i_deg_s;
    channels[index++] = axis->angle_d_deg_s;
    channels[index++] = axis->angle_output_deg_s;
    channels[index++] = axis->angle_output_limit_ratio;
    channels[index++] = axis->speed_reference_deg_s;
    channels[index++] = axis->speed_feedback_deg_s;
    channels[index++] = axis->speed_error_deg_s;
    channels[index++] = axis->speed_p_current;
    channels[index++] = axis->speed_i_current;
    channels[index++] = axis->speed_d_current;
    channels[index++] = axis->speed_output_current;
    channels[index++] = axis->speed_output_limit_ratio;
    channels[index++] = (float)axis->motor_current_feedback;
    channels[index++] = (float)axis->encoder_ecd;
    channels[index++] = axis->current_feedforward;
    channels[index++] = gimbal.yaw_offset_logic_deg;
    channels[index++] = chassis.command_wz_rad_s;
    channels[index++] = gimbal.yaw_base_rate_estimate_rad_s;
    channels[index++] = chassis.input_x_intent;
    channels[index++] = chassis.input_y_intent;
    channels[index++] = chassis.command_vx_m_s;
    channels[index++] = chassis.command_vy_m_s;
    channels[index++] = chassis.spin_translation_scale;
    return index;
}

static uint16_t GetDjiSpeedMotorChannels(
    const DJIMotorTuningSnapshot *motor,
    Motor_Speed_Unit_e expected_unit,
    float channels[])
{
    uint16_t index = 0U;

    if (motor == NULL || channels == NULL ||
        motor->control.settings.speed_unit != expected_unit) {
        return 0U;
    }

    channels[index++] = (float)RmTime_NowMs();
    channels[index++] = (float)motor->control.output_active;
    channels[index++] = motor->control.speed.reference;
    channels[index++] = motor->control.speed.measure;
    channels[index++] = motor->control.speed.error;
    channels[index++] = motor->control.speed.p_output;
    channels[index++] = motor->control.speed.i_output;
    channels[index++] = motor->control.speed.d_output;
    channels[index++] = motor->control.speed.output;
    channels[index++] = motor->control.final_output;
    channels[index++] = (float)motor->measure.real_current;
    return index;
}

static uint16_t GetDjiCascadeMotorChannels(
    const DJIMotorTuningSnapshot *motor,
    Motor_Speed_Unit_e expected_unit,
    float channels[])
{
    bool angle_active;
    uint16_t index = 0U;

    if (motor == NULL || channels == NULL ||
        motor->control.settings.speed_unit != expected_unit) {
        return 0U;
    }
    angle_active = motor->control.settings.outer_loop_type == ANGLE_LOOP;

    channels[index++] = (float)RmTime_NowMs();
    channels[index++] = (float)motor->control.output_active;
    channels[index++] = (float)motor->control.settings.outer_loop_type;
    channels[index++] = angle_active ? motor->control.angle.reference : 0.0f;
    channels[index++] = angle_active ? motor->control.angle.measure
                                     : motor->measure.total_angle;
    channels[index++] = angle_active ? motor->control.angle.error : 0.0f;
    channels[index++] = angle_active ? motor->control.angle.p_output : 0.0f;
    channels[index++] = angle_active ? motor->control.angle.i_output : 0.0f;
    channels[index++] = angle_active ? motor->control.angle.d_output : 0.0f;
    channels[index++] = angle_active ? motor->control.angle.output : 0.0f;
    channels[index++] = motor->control.speed.reference;
    channels[index++] = motor->control.speed.measure;
    channels[index++] = motor->control.speed.error;
    channels[index++] = motor->control.speed.p_output;
    channels[index++] = motor->control.speed.i_output;
    channels[index++] = motor->control.speed.d_output;
    channels[index++] = motor->control.speed.output;
    channels[index++] = motor->control.final_output;
    channels[index++] = (float)motor->measure.real_current;
    return index;
}

uint16_t TuningTelemetry_GetYawChannels(float channels[])
{
    return GetGimbalAxisChannels(false, channels);
}

uint16_t TuningTelemetry_GetPitchChannels(float channels[])
{
    return GetGimbalAxisChannels(true, channels);
}

uint16_t TuningTelemetry_GetLoaderChannels(float channels[])
{
    DJIMotorTuningSnapshot motor;

    if (!Shoot_GetMotorTuningSnapshot(SHOOT_MOTOR_LOADER, &motor)) {
        return 0U;
    }
    return GetDjiCascadeMotorChannels(&motor, MOTOR_SPEED_DEG_PER_SEC,
                                      channels);
}

uint16_t TuningTelemetry_GetFrictionLeftChannels(float channels[])
{
    DJIMotorTuningSnapshot motor;

    if (!Shoot_GetMotorTuningSnapshot(SHOOT_MOTOR_FRICTION_LEFT, &motor)) {
        return 0U;
    }
    return GetDjiSpeedMotorChannels(&motor, MOTOR_SPEED_RAD_PER_SEC, channels);
}

uint16_t TuningTelemetry_GetFrictionRightChannels(float channels[])
{
    DJIMotorTuningSnapshot motor;

    if (!Shoot_GetMotorTuningSnapshot(SHOOT_MOTOR_FRICTION_RIGHT, &motor)) {
        return 0U;
    }
    return GetDjiSpeedMotorChannels(&motor, MOTOR_SPEED_RAD_PER_SEC, channels);
}

uint16_t TuningTelemetry_GetShootStateChannels(float channels[])
{
    ShootTuningSnapshot shoot;
    uint16_t index = 0U;

    if (channels == NULL || !Shoot_GetTuningSnapshot(&shoot)) {
        return 0U;
    }

    channels[index++] = (float)RmTime_NowMs();
    channels[index++] = (float)shoot.input_fire_mode;
    channels[index++] = (float)shoot.fire_trigger_down;
    channels[index++] = (float)shoot.fire_trigger_pressed;
    channels[index++] = (float)shoot.single_trigger_consumed;
    channels[index++] = (float)shoot.single_trigger_activation_count;
    channels[index++] = (float)shoot.shoot_state;
    channels[index++] = (float)shoot.friction_ready;
    channels[index++] = (float)shoot.single_active;
    channels[index++] = (float)shoot.pending_shots;
    channels[index++] = (float)shoot.single_start_count;
    channels[index++] = (float)shoot.single_timeout_count;
    channels[index++] = (float)shoot.loader_jam_state;
    channels[index++] = (float)shoot.loader_jam_retry_count;
    channels[index++] = (float)shoot.loader_jam_fault_count;
    return index;
}

#if ROBOT_TUNING_TELEMETRY_ENABLE
static uint16_t TuningTelemetry_FillChannels(float channels[])
{
    return TuningTelemetry_GetShootStateChannels(channels);
}
#endif

bool TuningTelemetry_Init(void)
{
#if ROBOT_TUNING_TELEMETRY_ENABLE
    UART_HandleTypeDef *const uart = &ROBOT_TUNING_UART_HANDLE;

    tuning_initialized = false;
    tuning_sent_count = 0U;
    tuning_dropped_count = 0U;
    memset(tuning_tx_frame, 0, sizeof(tuning_tx_frame));

    if (uart->Instance == NULL || uart->hdmatx == NULL) {
        return false;
    }

    uart->Init.BaudRate = ROBOT_TUNING_UART_BAUDRATE;
    uart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->Init.StopBits = UART_STOPBITS_1;
    uart->Init.Parity = UART_PARITY_NONE;
    uart->Init.Mode = UART_MODE_TX_RX;
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(uart) != HAL_OK) {
        return false;
    }

    tuning_initialized = true;
    return true;
#else
    tuning_initialized = false;
    return true;
#endif
}

void TuningTelemetry_Publish(uint32_t now_ms)
{
#if ROBOT_TUNING_TELEMETRY_ENABLE
    static const uint8_t just_float_tail[TUNING_TAIL_SIZE] = {
        0x00U, 0x00U, 0x80U, 0x7FU,
    };
    UART_HandleTypeDef *const uart = &ROBOT_TUNING_UART_HANDLE;
    float channels[TUNING_MAX_CHANNEL_COUNT];
    uint16_t channel_count;
    uint16_t payload_size;
    uint16_t frame_size;

    (void)now_ms;

    if (!tuning_initialized) {
        return;
    }
    if (uart->gState != HAL_UART_STATE_READY) {
        tuning_dropped_count++;
        return;
    }

    channel_count = TuningTelemetry_FillChannels(channels);
    if (channel_count == 0U || channel_count > TUNING_MAX_CHANNEL_COUNT) {
        tuning_dropped_count++;
        return;
    }
    payload_size = (uint16_t)(channel_count * sizeof(float));
    frame_size = (uint16_t)(payload_size + TUNING_TAIL_SIZE);
    memcpy(tuning_tx_frame, channels, payload_size);
    memcpy(tuning_tx_frame + payload_size,
           just_float_tail,
           sizeof(just_float_tail));

    if (HAL_UART_Transmit_DMA(uart,
                              tuning_tx_frame,
                              frame_size) == HAL_OK) {
        tuning_sent_count++;
    } else {
        tuning_dropped_count++;
    }
#else
    (void)now_ms;
#endif
}

uint32_t TuningTelemetry_GetSentCount(void)
{
    return tuning_sent_count;
}

uint32_t TuningTelemetry_GetDroppedCount(void)
{
    return tuning_dropped_count;
}
