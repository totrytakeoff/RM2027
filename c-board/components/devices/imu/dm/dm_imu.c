#include "dm_imu.h"

#include <string.h>

#include "stm32f4xx_hal.h"
#include "rm_time.h"

#define DM_IMU_FRAME_HEAD_0 0x55
#define DM_IMU_FRAME_HEAD_1 0xAA
#define DM_IMU_FRAME_TAIL 0x0A

#define DM_IMU_TYPE_ACCEL 1
#define DM_IMU_TYPE_GYRO 2
#define DM_IMU_TYPE_EULER 3
#define DM_IMU_TYPE_QUAT 4

#define DM_IMU_CMD_PREFIX 0xAA
#define DM_IMU_CMD_TAIL 0x0D

#define DM_IMU_ACCEL_MIN (-235.2f)
#define DM_IMU_ACCEL_MAX (235.2f)
#define DM_IMU_GYRO_MIN (-34.88f)
#define DM_IMU_GYRO_MAX (34.88f)
#define DM_IMU_PITCH_MIN (-90.0f)
#define DM_IMU_PITCH_MAX (90.0f)
#define DM_IMU_ROLL_MIN (-180.0f)
#define DM_IMU_ROLL_MAX (180.0f)
#define DM_IMU_YAW_MIN (-180.0f)
#define DM_IMU_YAW_MAX (180.0f)
#define DM_IMU_QUAT_MIN (-1.0f)
#define DM_IMU_QUAT_MAX (1.0f)

static dm_imu_t *uart_bound_instance = NULL;

static uint16_t dm_imu_crc16(const uint8_t *ptr, uint16_t len)
{
    static const uint16_t table[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129,
        0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252,
        0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C,
        0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672,
        0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738,
        0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861,
        0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC,
        0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87, 0x4CE4, 0x5CC5,
        0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B,
        0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 0x9188, 0x81A9,
        0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3,
        0x5004, 0x4025, 0x7046, 0x6067, 0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C,
        0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3,
        0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8,
        0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676,
        0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C,
        0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16,
        0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B,
        0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36,
        0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
    };

    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i)
    {
        uint8_t index = (uint8_t)((crc >> 8) ^ ptr[i]);
        crc = (uint16_t)(((crc << 1) & 0xFFFF) ^ table[index]);
    }
    return crc;
}

static float dm_imu_uint_to_float(uint16_t x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void dm_imu_mark_ready(dm_imu_t *imu, uint8_t mask)
{
    imu->data.valid_mask |= mask;
    imu->data.timestamp_ms = RmTime_NowMs();
    imu->last_update_ms = imu->data.timestamp_ms;
    imu->data_ready = true;
}

static void dm_imu_uart_send(dm_imu_t *imu, uint8_t cmd, uint8_t arg)
{
    if (!imu || !imu->uart)
        return;
    uint8_t payload[4] = {DM_IMU_CMD_PREFIX, cmd, arg, DM_IMU_CMD_TAIL};
    USARTSend(imu->uart, payload, sizeof(payload), USART_TRANSFER_BLOCKING);
}

static void dm_imu_parse_uart_buffer(dm_imu_t *imu)
{
    if (imu->uart_rx_len < 4)
        return;

    size_t idx = 0;
    while (imu->uart_rx_len - idx >= 4)
    {
        if (imu->uart_rx_buf[idx] != DM_IMU_FRAME_HEAD_0 ||
            imu->uart_rx_buf[idx + 1] != DM_IMU_FRAME_HEAD_1)
        {
            idx++;
            continue;
        }

        uint8_t type = imu->uart_rx_buf[idx + 3];
        size_t frame_len = (type == DM_IMU_TYPE_QUAT) ? 23U : 19U;
        if (imu->uart_rx_len - idx < frame_len)
            break;

        const uint8_t *frame = imu->uart_rx_buf + idx;
        if (frame[frame_len - 1] != DM_IMU_FRAME_TAIL)
        {
            idx++;
            continue;
        }

        uint16_t crc_calc = dm_imu_crc16(frame, (uint16_t)(frame_len - 3));
        uint16_t crc_le = (uint16_t)frame[frame_len - 3] | ((uint16_t)frame[frame_len - 2] << 8);
        uint16_t crc_be = (uint16_t)frame[frame_len - 2] | ((uint16_t)frame[frame_len - 3] << 8);
        if (crc_calc != crc_le && crc_calc != crc_be)
        {
            idx++;
            continue;
        }

        if (frame[2] != imu->imu_id)
        {
            idx += frame_len;
            continue;
        }

        const uint8_t *payload = frame + 4;
        if (type == DM_IMU_TYPE_ACCEL)
        {
            memcpy(imu->data.accel, payload, sizeof(float) * 3);
            dm_imu_mark_ready(imu, 0x01);
        }
        else if (type == DM_IMU_TYPE_GYRO)
        {
            memcpy(imu->data.gyro, payload, sizeof(float) * 3);
            dm_imu_mark_ready(imu, 0x02);
        }
        else if (type == DM_IMU_TYPE_EULER)
        {
            memcpy(imu->data.euler, payload, sizeof(float) * 3);
            dm_imu_mark_ready(imu, 0x04);
        }
        else if (type == DM_IMU_TYPE_QUAT)
        {
            memcpy(imu->data.quat, payload, sizeof(float) * 4);
            dm_imu_mark_ready(imu, 0x08);
        }

        idx += frame_len;
    }

    if (idx > 0)
    {
        memmove(imu->uart_rx_buf, imu->uart_rx_buf + idx, imu->uart_rx_len - idx);
        imu->uart_rx_len -= idx;
    }
}

static void dm_imu_uart_rx_callback(void)
{
    if (!uart_bound_instance || !uart_bound_instance->uart)
        return;

    USARTInstance *uart = uart_bound_instance->uart;
    if (!uart->recv_buff_size)
        return;

    size_t copy_len = uart->recv_buff_size;
    if (copy_len > DM_IMU_UART_RX_BUFFER_LEN - uart_bound_instance->uart_rx_len)
    {
        uart_bound_instance->uart_rx_len = 0;
    }

    memcpy(uart_bound_instance->uart_rx_buf + uart_bound_instance->uart_rx_len,
           uart->recv_buff, copy_len);
    uart_bound_instance->uart_rx_len += (uint16_t)copy_len;
    dm_imu_parse_uart_buffer(uart_bound_instance);
}

static void dm_imu_can_rx_callback(CANInstance *instance)
{
    dm_imu_t *imu = (dm_imu_t *)instance->id;
    if (!imu)
        return;

    if (instance->rx_len < 8)
        return;

    const uint8_t *data = instance->rx_buff;
    uint8_t type = data[0];

    if (type == 0xCC)
    {
        if (data[2] == 0xDD)
        {
            imu->last_ack_rid = data[1];
            imu->last_ack_code = data[3];
            imu->last_ack_ms = RmTime_NowMs();
        }
        return;
    }

    if (type == DM_IMU_TYPE_ACCEL)
    {
        uint16_t ax = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
        uint16_t ay = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
        uint16_t az = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
        imu->data.accel[0] = dm_imu_uint_to_float(ax, DM_IMU_ACCEL_MIN, DM_IMU_ACCEL_MAX, 16);
        imu->data.accel[1] = dm_imu_uint_to_float(ay, DM_IMU_ACCEL_MIN, DM_IMU_ACCEL_MAX, 16);
        imu->data.accel[2] = dm_imu_uint_to_float(az, DM_IMU_ACCEL_MIN, DM_IMU_ACCEL_MAX, 16);
        imu->data.temp = (float)data[1];
        dm_imu_mark_ready(imu, 0x01);
        return;
    }

    if (type == DM_IMU_TYPE_GYRO)
    {
        uint16_t gx = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
        uint16_t gy = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
        uint16_t gz = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
        imu->data.gyro[0] = dm_imu_uint_to_float(gx, DM_IMU_GYRO_MIN, DM_IMU_GYRO_MAX, 16);
        imu->data.gyro[1] = dm_imu_uint_to_float(gy, DM_IMU_GYRO_MIN, DM_IMU_GYRO_MAX, 16);
        imu->data.gyro[2] = dm_imu_uint_to_float(gz, DM_IMU_GYRO_MIN, DM_IMU_GYRO_MAX, 16);
        dm_imu_mark_ready(imu, 0x02);
        return;
    }

    if (type == DM_IMU_TYPE_EULER)
    {
        uint16_t pitch = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
        uint16_t yaw = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
        uint16_t roll = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
        imu->data.euler[0] = dm_imu_uint_to_float(roll, DM_IMU_ROLL_MIN, DM_IMU_ROLL_MAX, 16);
        imu->data.euler[1] = dm_imu_uint_to_float(pitch, DM_IMU_PITCH_MIN, DM_IMU_PITCH_MAX, 16);
        imu->data.euler[2] = dm_imu_uint_to_float(yaw, DM_IMU_YAW_MIN, DM_IMU_YAW_MAX, 16);
        dm_imu_mark_ready(imu, 0x04);
        return;
    }

    if (type == DM_IMU_TYPE_QUAT)
    {
        int w = (data[1] << 6) | ((data[2] & 0xF8) >> 2);
        int x = ((data[2] & 0x03) << 12) | (data[3] << 4) | ((data[4] & 0xF0) >> 4);
        int y = ((data[4] & 0x0F) << 10) | (data[5] << 2) | ((data[6] & 0xC0) >> 6);
        int z = ((data[6] & 0x3F) << 8) | data[7];
        imu->data.quat[0] = dm_imu_uint_to_float((uint16_t)w, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
        imu->data.quat[1] = dm_imu_uint_to_float((uint16_t)x, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
        imu->data.quat[2] = dm_imu_uint_to_float((uint16_t)y, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
        imu->data.quat[3] = dm_imu_uint_to_float((uint16_t)z, DM_IMU_QUAT_MIN, DM_IMU_QUAT_MAX, 14);
        dm_imu_mark_ready(imu, 0x08);
        return;
    }
}

void dm_imu_init_uart(dm_imu_t *imu, const dm_imu_uart_config_t *cfg)
{
    if (!imu || !cfg || !cfg->usart_handle)
        return;

    memset(imu, 0, sizeof(*imu));
    imu->iface = DM_IMU_IFACE_UART;
    imu->imu_id = cfg->imu_id;

    USART_Init_Config_s uart_cfg = {
        .recv_buff_size = (cfg->rx_len == 0 ? DM_IMU_UART_DEFAULT_RX_LEN : cfg->rx_len),
        .usart_handle = cfg->usart_handle,
        .module_callback = dm_imu_uart_rx_callback,
    };
    imu->uart = USARTRegister(&uart_cfg);
    uart_bound_instance = imu;
}

void dm_imu_init_can(dm_imu_t *imu, const dm_imu_can_config_t *cfg)
{
    if (!imu || !cfg || !cfg->can_handle)
        return;

    memset(imu, 0, sizeof(*imu));
    imu->iface = DM_IMU_IFACE_CAN;
    imu->imu_id = cfg->can_id;

    CAN_Init_Config_s can_cfg = {
        .can_handle = cfg->can_handle,
        .tx_id = cfg->can_id,
        .rx_id = cfg->mst_id,
        .can_module_callback = dm_imu_can_rx_callback,
        .id = imu,
    };
    imu->can = CANRegister(&can_cfg);
}

bool dm_imu_get_data(dm_imu_t *imu, dm_imu_data_t *out)
{
    if (!imu || !out)
        return false;
    if (!imu->data_ready)
        return false;
    *out = imu->data;
    imu->data_ready = false;
    return true;
}

const dm_imu_data_t *dm_imu_peek_data(const dm_imu_t *imu)
{
    if (!imu)
        return NULL;
    return &imu->data;
}

bool dm_imu_is_alive(const dm_imu_t *imu, uint32_t timeout_ms)
{
    if (!imu)
        return false;
    return RmTime_ElapsedMs(RmTime_NowMs(), imu->last_update_ms) <= timeout_ms;
}

bool dm_imu_get_last_ack(const dm_imu_t *imu, uint8_t *rid, uint8_t *ack)
{
    if (!imu || !rid || !ack)
        return false;
    *rid = imu->last_ack_rid;
    *ack = imu->last_ack_code;
    return true;
}

void dm_imu_uart_enter_settings(dm_imu_t *imu)
{
    dm_imu_uart_send(imu, 0x06, 0x01);
}

void dm_imu_uart_exit_settings(dm_imu_t *imu)
{
    dm_imu_uart_send(imu, 0x06, 0x00);
}

void dm_imu_uart_save_params(dm_imu_t *imu)
{
    dm_imu_uart_send(imu, 0x03, 0x01);
}

void dm_imu_uart_reboot(dm_imu_t *imu)
{
    dm_imu_uart_send(imu, 0x00, 0x00);
}

void dm_imu_uart_angle_zero(dm_imu_t *imu)
{
    dm_imu_uart_send(imu, 0x0C, 0x01);
}

void dm_imu_uart_gyro_cal(dm_imu_t *imu)
{
    dm_imu_uart_send(imu, 0x03, 0x02);
}

void dm_imu_uart_accel_cal(dm_imu_t *imu)
{
    dm_imu_uart_send(imu, 0x03, 0x03);
}

void dm_imu_uart_restore_factory(dm_imu_t *imu)
{
    dm_imu_uart_send(imu, 0x0B, 0x01);
}

void dm_imu_uart_set_iface(dm_imu_t *imu, uint8_t iface)
{
    dm_imu_uart_send(imu, 0x0A, iface);
}

void dm_imu_uart_set_interval_ms(dm_imu_t *imu, uint16_t interval_ms)
{
    if (!imu || !imu->uart)
        return;
    uint8_t payload[5] = {DM_IMU_CMD_PREFIX, 0x02, (uint8_t)(interval_ms & 0xFF),
                          (uint8_t)((interval_ms >> 8) & 0xFF), DM_IMU_CMD_TAIL};
    USARTSend(imu->uart, payload, sizeof(payload), USART_TRANSFER_BLOCKING);
}

void dm_imu_uart_set_can_id(dm_imu_t *imu, uint8_t can_id)
{
    dm_imu_uart_send(imu, 0x08, can_id);
}

void dm_imu_uart_set_mst_id(dm_imu_t *imu, uint8_t mst_id)
{
    dm_imu_uart_send(imu, 0x09, mst_id);
}

void dm_imu_uart_set_temp_control(dm_imu_t *imu, bool enable, uint8_t target_c)
{
    dm_imu_uart_send(imu, 0x04, enable ? 0x01 : 0x00);
    dm_imu_uart_send(imu, 0x05, target_c);
}

void dm_imu_uart_enable_outputs(dm_imu_t *imu, bool accel, bool gyro, bool euler, bool quat)
{
    dm_imu_uart_send(imu, 0x01, accel ? 0x14 : 0x04);
    dm_imu_uart_send(imu, 0x01, gyro ? 0x15 : 0x05);
    dm_imu_uart_send(imu, 0x01, euler ? 0x16 : 0x06);
    dm_imu_uart_send(imu, 0x01, quat ? 0x17 : 0x07);
}

void dm_imu_can_write_reg(dm_imu_t *imu, uint8_t rid, uint32_t data)
{
    if (!imu || !imu->can)
        return;
    imu->can->tx_buff[0] = 0xCC;
    imu->can->tx_buff[1] = rid;
    imu->can->tx_buff[2] = 0x01;
    imu->can->tx_buff[3] = 0xDD;
    memcpy(&imu->can->tx_buff[4], &data, 4);
    CANSetDLC(imu->can, 8);
    CANTransmit(imu->can, 1000U);
}

static void dm_imu_can_read_reg(dm_imu_t *imu, uint8_t rid)
{
    if (!imu || !imu->can)
        return;
    imu->can->tx_buff[0] = 0xCC;
    imu->can->tx_buff[1] = rid;
    imu->can->tx_buff[2] = 0x00;
    imu->can->tx_buff[3] = 0xDD;
    memset(&imu->can->tx_buff[4], 0, 4);
    CANSetDLC(imu->can, 8);
    CANTransmit(imu->can, 1000U);
}

void dm_imu_can_request_accel(dm_imu_t *imu)
{
    dm_imu_can_read_reg(imu, 0x01);
}

void dm_imu_can_request_gyro(dm_imu_t *imu)
{
    dm_imu_can_read_reg(imu, 0x02);
}

void dm_imu_can_request_euler(dm_imu_t *imu)
{
    dm_imu_can_read_reg(imu, 0x03);
}

void dm_imu_can_request_quat(dm_imu_t *imu)
{
    dm_imu_can_read_reg(imu, 0x04);
}

void dm_imu_can_set_active(dm_imu_t *imu, bool active)
{
    dm_imu_can_write_reg(imu, 0x0B, active ? 1U : 0U);
}

void dm_imu_can_set_interval_ms(dm_imu_t *imu, uint16_t interval_ms)
{
    dm_imu_can_write_reg(imu, 0x0A, interval_ms);
}
