/**
 * @file minimal_referee.c
 * @brief 最小裁判系统只读封装(无UI/无任务)
 */

#include "referee.h"

#include "robot_config.h"
#include "usart.h"

#if REFEREE_ENABLE
#include "rm_referee.h"
#endif
#include "user_lib.h"

static MinimalRefereeData_t g_ref_data;

#if REFEREE_ENABLE
static referee_info_t *g_referee = NULL;
#endif

void MinimalReferee_Init(void)
{
    g_ref_data.online = 0U;
    g_ref_data.robot_id = 0U;
    g_ref_data.chassis_power_limit = 0U;
    g_ref_data.shooter_heat_limit = 0U;
    g_ref_data.shooter_heat = 0U;
    g_ref_data.allowance_17mm = 0U;
    g_ref_data.shoot_initial_speed = 0.0f;
    g_ref_data.power_management_chassis_output = 1U;
    g_ref_data.power_management_shooter_output = 1U;

#if REFEREE_ENABLE
    g_referee = RefereeInit(&REFEREE_UART);
#endif
}

void MinimalReferee_Update(void)
{
#if REFEREE_ENABLE
    if (g_referee == NULL) {
        g_ref_data.online = 0U;
        return;
    }

    g_ref_data.robot_id = g_referee->GameRobotState.robot_id;
    g_ref_data.chassis_power_limit = g_referee->GameRobotState.chassis_power_limit;
    g_ref_data.shooter_heat_limit = g_referee->GameRobotState.shooter_barrel_heat_limit;
    g_ref_data.shooter_heat = g_referee->PowerHeatData.shooter_17mm_barrel_heat;
    g_ref_data.allowance_17mm = g_referee->ProjectileAllowance.projectile_allowance_17mm;
    g_ref_data.shoot_initial_speed = g_referee->ShootData.initial_speed;
    g_ref_data.power_management_chassis_output = g_referee->GameRobotState.power_management_chassis_output;
    g_ref_data.power_management_shooter_output = g_referee->GameRobotState.power_management_shooter_output;
    g_ref_data.online = (g_ref_data.robot_id != 0U) ? 1U : 0U;
#else
    g_ref_data.online = 0U;
#endif
}

const MinimalRefereeData_t *MinimalReferee_GetData(void)
{
    return &g_ref_data;
}

uint8_t MinimalReferee_AllowChassis(void)
{
    if (!g_ref_data.online) {
        return 1U;
    }
    return g_ref_data.power_management_chassis_output ? 1U : 0U;
}

uint8_t MinimalReferee_AllowShoot(void)
{
    if (!g_ref_data.online) {
        return 1U;
    }
    return g_ref_data.power_management_shooter_output ? 1U : 0U;
}

uint8_t MinimalReferee_AllowLoader(void)
{
    float heat_ratio;

    if (!MinimalReferee_AllowShoot()) {
        return 0U;
    }
    if (!g_ref_data.online) {
        return 1U;
    }
    if (g_ref_data.allowance_17mm == 0U) {
        return 0U;
    }
    if (g_ref_data.shooter_heat_limit == 0U) {
        return 1U;
    }

    heat_ratio = (float)g_ref_data.shooter_heat / (float)g_ref_data.shooter_heat_limit;
    return (heat_ratio >= REFEREE_HEAT_STOP_RATIO) ? 0U : 1U;
}

float MinimalReferee_ChassisScale(void)
{
    float scale;

    if (!MinimalReferee_AllowChassis()) {
        return 0.0f;
    }
    if (!g_ref_data.online) {
        return 1.0f;
    }
    if (g_ref_data.chassis_power_limit == 0U) {
        return REFEREE_CHASSIS_SCALE_MIN;
    }

    scale = (float)g_ref_data.chassis_power_limit / REFEREE_CHASSIS_POWER_NOMINAL;
    return float_constrain(scale, REFEREE_CHASSIS_SCALE_MIN, 1.0f);
}

float MinimalReferee_FrictionSpeedScale(void)
{
    if (!MinimalReferee_AllowShoot()) {
        return 0.0f;
    }

    /* 热量限制发射频率，不改变已经按弹速标定的摩擦轮目标。 */
    return 1.0f;
}
