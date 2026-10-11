#ifndef DM_IMU_H
#define DM_IMU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_can.h"
#include "bsp_usart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DM_IMU_UART_DEFAULT_RX_LEN 80U
#define DM_IMU_UART_RX_BUFFER_LEN 256U

typedef enum
{
    DM_IMU_IFACE_UART = 0,
    DM_IMU_IFACE_CAN = 1,
} dm_imu_iface_t;

typedef struct
{
    float accel[3];
    float gyro[3];
    float euler[3];   // roll, pitch, yaw
    float quat[4];    // w, x, y, z
    float temp;
    uint8_t valid_mask;
    uint32_t timestamp_ms;
} dm_imu_data_t;

typedef struct
{
    UART_HandleTypeDef *usart_handle;
    uint8_t imu_id;
    uint16_t rx_len;
} dm_imu_uart_config_t;

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint8_t can_id;
    uint8_t mst_id;
} dm_imu_can_config_t;

typedef struct
{
    dm_imu_iface_t iface;
    uint8_t imu_id;

    USARTInstance *uart;
    CANInstance *can;

    dm_imu_data_t data;
    bool data_ready;
    uint32_t last_update_ms;

    uint8_t last_ack_rid;
    uint8_t last_ack_code;
    uint32_t last_ack_ms;

    uint8_t uart_rx_buf[DM_IMU_UART_RX_BUFFER_LEN];
    uint16_t uart_rx_len;
} dm_imu_t;

void dm_imu_init_uart(dm_imu_t *imu, const dm_imu_uart_config_t *cfg);
void dm_imu_init_can(dm_imu_t *imu, const dm_imu_can_config_t *cfg);

bool dm_imu_get_data(dm_imu_t *imu, dm_imu_data_t *out);
const dm_imu_data_t *dm_imu_peek_data(const dm_imu_t *imu);
bool dm_imu_is_alive(const dm_imu_t *imu, uint32_t timeout_ms);
bool dm_imu_get_last_ack(const dm_imu_t *imu, uint8_t *rid, uint8_t *ack);

// UART quick commands (USB/485)
void dm_imu_uart_enter_settings(dm_imu_t *imu);
void dm_imu_uart_exit_settings(dm_imu_t *imu);
void dm_imu_uart_save_params(dm_imu_t *imu);
void dm_imu_uart_reboot(dm_imu_t *imu);
void dm_imu_uart_angle_zero(dm_imu_t *imu);
void dm_imu_uart_gyro_cal(dm_imu_t *imu);
void dm_imu_uart_accel_cal(dm_imu_t *imu);
void dm_imu_uart_restore_factory(dm_imu_t *imu);
void dm_imu_uart_set_iface(dm_imu_t *imu, uint8_t iface);
void dm_imu_uart_set_interval_ms(dm_imu_t *imu, uint16_t interval_ms);
void dm_imu_uart_set_can_id(dm_imu_t *imu, uint8_t can_id);
void dm_imu_uart_set_mst_id(dm_imu_t *imu, uint8_t mst_id);
void dm_imu_uart_set_temp_control(dm_imu_t *imu, bool enable, uint8_t target_c);
void dm_imu_uart_enable_outputs(dm_imu_t *imu, bool accel, bool gyro, bool euler, bool quat);

// CAN commands (request mode)
void dm_imu_can_request_accel(dm_imu_t *imu);
void dm_imu_can_request_gyro(dm_imu_t *imu);
void dm_imu_can_request_euler(dm_imu_t *imu);
void dm_imu_can_request_quat(dm_imu_t *imu);
void dm_imu_can_set_active(dm_imu_t *imu, bool active);
void dm_imu_can_set_interval_ms(dm_imu_t *imu, uint16_t interval_ms);

// CAN config (register write)
void dm_imu_can_write_reg(dm_imu_t *imu, uint8_t rid, uint32_t data);

#ifdef __cplusplus
}
#endif

#endif
