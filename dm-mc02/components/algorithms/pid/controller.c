/**
 * @file controller.c
 * @brief  PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 */
#include "controller.h"

/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (fabsf(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PIDInstance *pid)
{
    const float temp_Iout = pid->Iout + pid->ITerm;
    const float temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (fabsf(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // 积分却还在累积
        {
            pid->ITerm = 0; // 当前积分项置零
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid, const PID_Init_Config_s *config)
{
    if (pid == NULL) return;
    memset(pid, 0, sizeof(PIDInstance));
    if (config == NULL) return;
    pid->Kp = config->Kp; pid->Ki = config->Ki; pid->Kd = config->Kd;
    pid->MaxOut = config->MaxOut; pid->DeadBand = config->DeadBand;
    pid->Improve = config->Improve; pid->IntegralLimit = config->IntegralLimit;
    pid->CoefA = config->CoefA; pid->CoefB = config->CoefB;
    pid->Output_LPF_RC = config->Output_LPF_RC;
    pid->Derivative_LPF_RC = config->Derivative_LPF_RC;
}

void PIDReset(PIDInstance *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
    pid->dt = 0.0f;
    pid->ERRORHandler.ERRORCount = 0U;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref, float dt_s)
{
    if (!pid) return 0.0f;
    if (!isfinite(measure) || !isfinite(ref) || !isfinite(dt_s) || dt_s <= 0.0f ||
        !isfinite(pid->Kp) || !isfinite(pid->Ki) || !isfinite(pid->Kd) ||
        !isfinite(pid->MaxOut) || pid->MaxOut < 0.0f ||
        !isfinite(pid->DeadBand) || pid->DeadBand < 0.0f ||
        !isfinite(pid->IntegralLimit) || pid->IntegralLimit < 0.0f ||
        !isfinite(pid->Output_LPF_RC) || pid->Output_LPF_RC < 0.0f ||
        !isfinite(pid->Derivative_LPF_RC) || pid->Derivative_LPF_RC < 0.0f ||
        ((pid->Improve & PID_OutputFilter) && !isfinite(pid->Output_LPF_RC + dt_s)) ||
        ((pid->Improve & PID_DerivativeFilter) && !isfinite(pid->Derivative_LPF_RC + dt_s)) ||
        !isfinite(pid->CoefA) || !isfinite(pid->CoefB) || pid->CoefB < 0.0f ||
        ((pid->Improve & PID_ChangingIntegrationRate) &&
            (pid->CoefA <= 0.0f || !isfinite(pid->CoefA + pid->CoefB))) ||
        !isfinite(pid->Iout) || !isfinite(pid->Last_Err) || !isfinite(pid->Last_Measure) ||
        !isfinite(pid->Last_Output) || !isfinite(pid->Last_Dout)) {
        PIDReset(pid); return 0.0f;
    }
    // 堵转检测
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = dt_s;

    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;
    if (!isfinite(pid->Err)) goto numeric_failure;

    // 如果在死区外,则计算PID
    if (fabsf(pid->Err) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
        if (!isfinite(pid->Pout) || !isfinite(pid->ITerm) || !isfinite(pid->Dout)) goto numeric_failure;

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral) {
            f_Trapezoid_Intergral(pid);
            if (!isfinite(pid->ITerm)) goto numeric_failure;
        }
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate) {
            f_Changing_Integration_Rate(pid);
            if (!isfinite(pid->ITerm)) goto numeric_failure;
        }
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement) {
            f_Derivative_On_Measurement(pid);
            if (!isfinite(pid->Dout)) goto numeric_failure;
        }
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        if (!isfinite(pid->ITerm) || !isfinite(pid->Dout) ||
            !isfinite(pid->Iout + pid->ITerm) ||
            !isfinite(pid->Pout + pid->Iout + pid->Dout)) goto numeric_failure;
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出
        if (!isfinite(pid->Iout) || !isfinite(pid->Output)) goto numeric_failure;

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);
        /* Saturation must not disguise Inf as a valid maximum command. */
        if (!isfinite(pid->Output)) goto numeric_failure;

        // 输出限幅
        f_Output_Limit(pid);
    }
    else if (pid->Improve & PID_HoldIntegral)
    {
        pid->Pout = pid->Dout = pid->ITerm = 0;
        if (pid->Improve & PID_Integral_Limit)
            pid->Iout = fmaxf(-pid->IntegralLimit, fminf(pid->IntegralLimit, pid->Iout));
        pid->Output = pid->Iout;
        if (pid->Improve & PID_OutputFilter) f_Output_Filter(pid);
        if (!isfinite(pid->Output)) goto numeric_failure;
        f_Output_Limit(pid);
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    if (!isfinite(pid->Output) || !isfinite(pid->Iout) || !isfinite(pid->Dout)) {
        PIDReset(pid); return 0.0f;
    }
    return pid->Output;
numeric_failure:
    PIDReset(pid);
    return 0.0f;
}
