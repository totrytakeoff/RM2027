/**
 * @file minimal_referee_types.h
 * @brief 最小裁判数据类型
 */

#ifndef ROBOT_REFEREE_TYPES_H
#define ROBOT_REFEREE_TYPES_H

#include <stdint.h>

typedef struct {
    uint8_t online;
    uint16_t robot_id;
    uint16_t chassis_power_limit;
    uint16_t shooter_heat_limit;
    uint16_t shooter_heat;
    uint16_t allowance_17mm;
    float shoot_initial_speed;
    uint8_t power_management_chassis_output;
    uint8_t power_management_shooter_output;
} MinimalRefereeData_t;

#endif /* ROBOT_REFEREE_TYPES_H */
