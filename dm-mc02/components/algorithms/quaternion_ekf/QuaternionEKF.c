/**
 ******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  ———————
 *  as + 1
 * 
 ******************************************************************************
 */
#include "QuaternionEKF.h"
#include <string.h>
#include <limits.h>


enum {
    QEKF_STATE_DIMENSION = 6U,
    QEKF_CONTROL_DIMENSION = 0U,
    QEKF_MEASUREMENT_DIMENSION = 3U,
    QEKF_WORKSPACE_SIZE =
        KALMAN_FILTER_WORKSPACE_SIZE(QEKF_STATE_DIMENSION,
                                     QEKF_CONTROL_DIMENSION,
                                     QEKF_MEASUREMENT_DIMENSION),
};


const float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};
static const float initial_covariance[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};

static float invSqrt(float x);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
bool IMU_QuaternionEKF_Init(QEKF_INS_t *ins, float* init_quaternion,float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf)
{
    if (!ins || !init_quaternion || !isfinite(process_noise1) || process_noise1<0 ||
        !isfinite(process_noise2) || process_noise2<0 || !isfinite(measure_noise) || measure_noise<=0 ||
        !isfinite(lambda) || lambda<=0 || !isfinite(lpf) || lpf<0) return false;
    for (unsigned i=0;i<4U;++i) if (!isfinite(init_quaternion[i])) return false;
    float initial_norm=sqrtf(init_quaternion[0]*init_quaternion[0]+init_quaternion[1]*init_quaternion[1]+
        init_quaternion[2]*init_quaternion[2]+init_quaternion[3]*init_quaternion[3]);
    if (!isfinite(initial_norm) || initial_norm<1e-6f) return false;
    memset(ins,0,sizeof(*ins));
    ins->Initialized = 0;
    ins->Q1 = process_noise1;
    ins->Q2 = process_noise2;
    ins->R = measure_noise;
    ins->ChiSquareTestThreshold = 1e-8;
    ins->ConvergeFlag = 0;
    ins->ErrorCount = 0;
    ins->UpdateCount = 0;
    if (lambda > 1)
    {
        lambda = 1;
    }
    ins->lambda = lambda;
    ins->accLPFcoef = lpf;

    // 初始化矩阵维度信息
    if (!Kalman_Filter_InitWithWorkspace(&ins->IMU_QuaternionEKF,
                                         QEKF_STATE_DIMENSION,
                                         QEKF_CONTROL_DIMENSION,
                                         QEKF_MEASUREMENT_DIMENSION,
                                         ins->workspace.bytes,
                                         sizeof(ins->workspace.bytes))) {
        return false;
    }
    ins->Initialized = 1;
    ins->IMU_QuaternionEKF.user_context=ins;
    Matrix_Init(&ins->ChiSquare, 1, 1, (float *)ins->ChiSquare_Data);

    // 姿态初始化
    for(int i = 0; i < 4; i++)
    {
        ins->IMU_QuaternionEKF.xhat_data[i] = init_quaternion[i]/initial_norm;
        ins->q[i]=init_quaternion[i]/initial_norm;
    }

    // 自定义函数初始化,用于扩展或增加kf的基础功能
    ins->IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    ins->IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    ins->IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    ins->IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    ins->IMU_QuaternionEKF.SkipEq3 = TRUE;
    ins->IMU_QuaternionEKF.SkipEq4 = TRUE;

    memcpy(ins->IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(ins->IMU_QuaternionEKF.P_data, initial_covariance, sizeof(initial_covariance));
    return true;
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s²
 * @param[in]       update period in s
 */
bool IMU_QuaternionEKF_Update(QEKF_INS_t *ins, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    if (!ins || !ins->Initialized || !isfinite(dt) || dt<=0 || dt>0.02f ||
        !isfinite(gx) || !isfinite(gy) || !isfinite(gz) || !isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
        ax*ax+ay*ay+az*az<1e-6f) return false;
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    float halfgxdt, halfgydt, halfgzdt;
    float accelInvNorm;

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    ins->dt = dt;

    ins->Gyro[0] = gx - ins->GyroBias[0];
    ins->Gyro[1] = gy - ins->GyroBias[1];
    ins->Gyro[2] = gz - ins->GyroBias[2];

    // set F
    halfgxdt = 0.5f * ins->Gyro[0] * dt;
    halfgydt = 0.5f * ins->Gyro[1] * dt;
    halfgzdt = 0.5f * ins->Gyro[2] * dt;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    memcpy(ins->IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    ins->IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    ins->IMU_QuaternionEKF.F_data[2] = -halfgydt;
    ins->IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    ins->IMU_QuaternionEKF.F_data[6] = halfgxdt;
    ins->IMU_QuaternionEKF.F_data[8] = halfgzdt;
    ins->IMU_QuaternionEKF.F_data[9] = -halfgydt;

    ins->IMU_QuaternionEKF.F_data[12] = halfgydt;
    ins->IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    ins->IMU_QuaternionEKF.F_data[15] = halfgxdt;

    ins->IMU_QuaternionEKF.F_data[18] = halfgzdt;
    ins->IMU_QuaternionEKF.F_data[19] = halfgydt;
    ins->IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
    if (ins->UpdateCount == 0) // 如果是第一次进入,需要初始化低通滤波
    {
        ins->Accel[0] = ax;
        ins->Accel[1] = ay;
        ins->Accel[2] = az;
    }
    ins->Accel[0] = ins->Accel[0] * ins->accLPFcoef / (ins->dt + ins->accLPFcoef) + ax * ins->dt / (ins->dt + ins->accLPFcoef);
    ins->Accel[1] = ins->Accel[1] * ins->accLPFcoef / (ins->dt + ins->accLPFcoef) + ay * ins->dt / (ins->dt + ins->accLPFcoef);
    ins->Accel[2] = ins->Accel[2] * ins->accLPFcoef / (ins->dt + ins->accLPFcoef) + az * ins->dt / (ins->dt + ins->accLPFcoef);

    // set z,单位化重力加速度向量
    accelInvNorm = invSqrt(ins->Accel[0] * ins->Accel[0] + ins->Accel[1] * ins->Accel[1] + ins->Accel[2] * ins->Accel[2]);
    for (uint8_t i = 0; i < 3; ++i)
    {
        ins->IMU_QuaternionEKF.MeasuredVector[i] = ins->Accel[i] * accelInvNorm; // 用加速度向量更新量测值
    }

    // get body state
    ins->gyro_norm = sqrtf(ins->Gyro[0] * ins->Gyro[0] +
                                        ins->Gyro[1] * ins->Gyro[1] +
                                        ins->Gyro[2] * ins->Gyro[2]);
    ins->accl_norm = 1.0f / accelInvNorm;

    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用StableFlag来确定
    if (ins->gyro_norm < 0.3f && ins->accl_norm > 9.8f - 0.5f && ins->accl_norm < 9.8f + 0.5f)
    {
        ins->StableFlag = 1;
    }
    else
    {
        ins->StableFlag = 0;
    }

    // set Q R,过程噪声和观测噪声矩阵
    ins->IMU_QuaternionEKF.Q_data[0] = ins->Q1 * ins->dt;
    ins->IMU_QuaternionEKF.Q_data[7] = ins->Q1 * ins->dt;
    ins->IMU_QuaternionEKF.Q_data[14] = ins->Q1 * ins->dt;
    ins->IMU_QuaternionEKF.Q_data[21] = ins->Q1 * ins->dt;
    ins->IMU_QuaternionEKF.Q_data[28] = ins->Q2 * ins->dt;
    ins->IMU_QuaternionEKF.Q_data[35] = ins->Q2 * ins->dt;
    ins->IMU_QuaternionEKF.R_data[0] = ins->R;
    ins->IMU_QuaternionEKF.R_data[4] = ins->R;
    ins->IMU_QuaternionEKF.R_data[8] = ins->R;

    // 调用kalman_filter.c封装好的函数,注意几个User_Funcx_f的调用
    if (!Kalman_Filter_Update(&ins->IMU_QuaternionEKF)) return false;
    float qnorm=0;
    for (unsigned i=0;i<6U;++i) {
        float value=ins->IMU_QuaternionEKF.FilteredValue[i];
        if (!isfinite(value)) return false;
        if (i<4U) qnorm+=value*value;
    }
    qnorm=sqrtf(qnorm);
    if (!isfinite(qnorm) || qnorm<1e-6f) return false;
    for (unsigned i=0;i<4U;++i) {
        ins->IMU_QuaternionEKF.FilteredValue[i]/=qnorm;
        ins->IMU_QuaternionEKF.xhat_data[i]=ins->IMU_QuaternionEKF.FilteredValue[i];
    }

    // 获取融合后的数据,包括四元数和xy零飘值
    ins->q[0] = ins->IMU_QuaternionEKF.FilteredValue[0];
    ins->q[1] = ins->IMU_QuaternionEKF.FilteredValue[1];
    ins->q[2] = ins->IMU_QuaternionEKF.FilteredValue[2];
    ins->q[3] = ins->IMU_QuaternionEKF.FilteredValue[3];
    ins->GyroBias[0] = ins->IMU_QuaternionEKF.FilteredValue[4];
    ins->GyroBias[1] = ins->IMU_QuaternionEKF.FilteredValue[5];
    ins->GyroBias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移

    // 利用四元数反解欧拉角
    ins->Yaw = atan2f(2.0f * (ins->q[0] * ins->q[3] + ins->q[1] * ins->q[2]), 2.0f * (ins->q[0] * ins->q[0] + ins->q[1] * ins->q[1]) - 1.0f) * 57.295779513f;
    ins->Pitch = atan2f(2.0f * (ins->q[0] * ins->q[1] + ins->q[2] * ins->q[3]), 2.0f * (ins->q[0] * ins->q[0] + ins->q[3] * ins->q[3]) - 1.0f) * 57.295779513f;
    ins->Roll = asinf(fmaxf(-1.0f,fminf(1.0f,-2.0f * (ins->q[1] * ins->q[3] - ins->q[0] * ins->q[2])))) * 57.295779513f;

    // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (ins->Yaw - ins->YawAngleLast > 180.0f)
    {
        if (ins->YawRoundCount==INT32_MIN) return false;
        ins->YawRoundCount--;
    }
    else if (ins->Yaw - ins->YawAngleLast < -180.0f)
    {
        if (ins->YawRoundCount==INT32_MAX) return false;
        ins->YawRoundCount++;
    }
    ins->YawTotalAngle = 360.0f * ins->YawRoundCount + ins->Yaw;
    ins->YawAngleLast = ins->Yaw;
    ins->UpdateCount++; // 初始化低通滤波用,计数测试用
    return isfinite(ins->Yaw) && isfinite(ins->Pitch) && isfinite(ins->Roll);
}

/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
 *        并对零漂的方差进行限制,防止过度收敛并限幅防止发散
 *
 * @param kf
 */
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    QEKF_INS_t *ins=kf->user_context;
    float q0, q1, q2, q3;
    float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // quaternion normalize
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; ++i)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf->F_data[4] = q1 * ins->dt / 2;
    kf->F_data[5] = q2 * ins->dt / 2;

    kf->F_data[10] = -q0 * ins->dt / 2;
    kf->F_data[11] = q3 * ins->dt / 2;

    kf->F_data[16] = -q3 * ins->dt / 2;
    kf->F_data[17] = -q0 * ins->dt / 2;

    kf->F_data[22] = q2 * ins->dt / 2;
    kf->F_data[23] = -q1 * ins->dt / 2;

    // fading filter,防止零飘参数过度收敛
    kf->P_data[28] /= ins->lambda;
    kf->P_data[35] /= ins->lambda;

    // 限幅,防止发散
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;
    }
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *
 * @param kf
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    float doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof(float) * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 * @param kf
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    QEKF_INS_t *ins=kf->user_context;
    float q0, q1, q2, q3;

    if (!RmKalman_RecordStatus(kf, Matrix_Transpose(&kf->H, &kf->HT))) return; // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    if (!RmKalman_RecordStatus(kf, Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix))) return; // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    if (!RmKalman_RecordStatus(kf, Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1))) return; // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    if (!RmKalman_RecordStatus(kf, Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S))) return; // S = H P'(k) HT + R
    if (!RmKalman_RecordStatus(kf, Matrix_Inverse(&kf->S, &kf->temp_matrix1))) return;     // temp_matrix1 = inv(H·P'(k)·HT + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 计算预测得到的重力加速度方向(通过姿态获取的)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算预测值和各个轴的方向余弦
    for (uint8_t i = 0; i < 3; ++i)
    {
        ins->OrientationCosine[i] = acosf(fminf(1.0f,fabsf(kf->temp_vector_data[i])));
    }

    // 利用加速度计数据修正
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    if (!RmKalman_RecordStatus(kf, Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1))) return; // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,卡方检验
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    if (!RmKalman_RecordStatus(kf, Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix))) return; // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    if (!RmKalman_RecordStatus(kf, Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector))) return; // temp_vector = z(k) - h(xhat'(k))'
    if (!RmKalman_RecordStatus(kf, Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &ins->ChiSquare))) return;
    // rk is small,filter converged/converging
    if (ins->ChiSquare_Data[0] < 0.5f * ins->ChiSquareTestThreshold)
    {
        ins->ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged
    if (ins->ChiSquare_Data[0] > ins->ChiSquareTestThreshold && ins->ConvergeFlag)
    {
        if (ins->StableFlag)
        {
            ins->ErrorCount++; // 载体静止时仍无法通过卡方检验
        }
        else
        {
            ins->ErrorCount = 0;
        }

        if (ins->ErrorCount > 50)
        {
            // 滤波器发散
            ins->ConvergeFlag = 0;
            kf->SkipEq5 = FALSE; // step-5 is cov mat P updating
        }
        else
        {
            //  残差未通过卡方检验 仅预测
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof(float) * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof(float) * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE; // part5 is P updating
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk越小则增益越大,否则更相信预测值
        if (ins->ChiSquare_Data[0] > 0.1f * ins->ChiSquareTestThreshold && ins->ConvergeFlag)
        {
            ins->AdaptiveGainScale = (ins->ChiSquareTestThreshold - ins->ChiSquare_Data[0]) / (0.9f * ins->ChiSquareTestThreshold);
        }
        else
        {
            ins->AdaptiveGainScale = 1;
        }
        ins->ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    if (!RmKalman_RecordStatus(kf, Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix))) return; // temp_matrix = P'(k)·HT
    if (!RmKalman_RecordStatus(kf, Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K))) return;

    // implement adaptive
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; ++i)
    {
        kf->K_data[i] *= ins->AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; ++i)
    {
        for (uint8_t j = 0; j < 3; ++j)
        {
            kf->K_data[i * 3 + j] *= ins->OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    if (!RmKalman_RecordStatus(kf, Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector))) return; // temp_vector = K(k)·(z(k) - H·xhat'(k))

    // 零漂修正限幅,一般不会有过大的漂移
    if (ins->ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; ++i)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * ins->dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * ins->dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * ins->dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * ins->dt;
            }
        }
    }

    // 不修正yaw轴数据
    kf->temp_vector.pData[3] = 0;
    if (!RmKalman_RecordStatus(kf, Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat))) return;
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf kf类型定义
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    QEKF_INS_t *ins=kf->user_context;
    memcpy(ins->covariance, kf->P_data, sizeof(ins->covariance));
    memcpy(ins->gain, kf->K_data, sizeof(ins->gain));
    memcpy(ins->observation, kf->H_data, sizeof(ins->observation));
}

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
    return 1.0f/sqrtf(x); /* No long-width assumptions or strict-aliasing violations. */
}
