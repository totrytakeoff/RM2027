/**
 * @file referee.h
 * @brief 最小裁判系统只读封装(联锁用)
 */

#ifndef ROBOT_REFEREE_H
#define ROBOT_REFEREE_H

#include "referee_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void MinimalReferee_Init(void);
void MinimalReferee_Update(void);
const MinimalRefereeData_t *MinimalReferee_GetData(void);

uint8_t MinimalReferee_AllowChassis(void);
uint8_t MinimalReferee_AllowShoot(void);
uint8_t MinimalReferee_AllowLoader(void);
float MinimalReferee_ChassisScale(void);
float MinimalReferee_FrictionSpeedScale(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_REFEREE_H */
