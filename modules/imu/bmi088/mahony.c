#include "mahony.h"
#include "deewaz_math.h"

/**
 * @brief  由四元数更新旋转矩阵（Mahony_Update 内部调用）
 * @param  mf Mahony 滤波器句柄指针
 */
static void mahony_update_rotation_matrix(Mahony_t *mf)
{
    float q1q1 = mf->q1 * mf->q1;
    float q2q2 = mf->q2 * mf->q2;
    float q3q3 = mf->q3 * mf->q3;

    float q0q1 = mf->q0 * mf->q1;
    float q0q2 = mf->q0 * mf->q2;
    float q0q3 = mf->q0 * mf->q3;
    float q1q2 = mf->q1 * mf->q2;
    float q1q3 = mf->q1 * mf->q3;
    float q2q3 = mf->q2 * mf->q3;

    mf->rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    mf->rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    mf->rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    mf->rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    mf->rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    mf->rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    mf->rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    mf->rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    mf->rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void Mahony_Input(Mahony_t *mf, Axis3f gyro, Axis3f acc)
{
    mf->gyro = gyro;
    mf->acc  = acc;
}

void Mahony_Update(Mahony_t *mf)
{
    /* ---- 自适应增益：加速度幅值偏离 1g 越大，越不信任加速度计 ---- */
    float acc_norm = sqrtf(mf->acc.x * mf->acc.x +
                           mf->acc.y * mf->acc.y +
                           mf->acc.z * mf->acc.z);
    float acc_err  = fabsf(acc_norm - 9.8f);
    float adapt;
    if (acc_err > 3.0f)
        adapt = 0.0f;                                    // 急加速/刹车，完全跳过
    else if (acc_err > 1.0f)
        adapt = 1.0f - (acc_err - 1.0f) / 2.0f;         // 过渡区，线性衰减
    else
        adapt = 1.0f;                                    // 正常，全增益

    // 单位化加速度
    float normalise = invSqrt(mf->acc.x * mf->acc.x +
                              mf->acc.y * mf->acc.y +
                              mf->acc.z * mf->acc.z);
    mf->acc.x *= normalise;
    mf->acc.y *= normalise;
    mf->acc.z *= normalise;

    // 加速度与重力方向的叉积误差
    float ex = (mf->acc.y * mf->rMat[2][2] - mf->acc.z * mf->rMat[2][1]);
    float ey = (mf->acc.z * mf->rMat[2][0] - mf->acc.x * mf->rMat[2][2]);
    float ez = (mf->acc.x * mf->rMat[2][1] - mf->acc.y * mf->rMat[2][0]);

    float kp_eff = mf->Kp * adapt;
    float ki_eff = mf->Ki * adapt;

    // 积分误差累计
    mf->exInt += ki_eff * ex * mf->dt;
    mf->eyInt += ki_eff * ey * mf->dt;
    mf->ezInt += ki_eff * ez * mf->dt;

    // PI 修正陀螺零偏
    mf->gyro.x += kp_eff * ex + mf->exInt;
    mf->gyro.y += kp_eff * ey + mf->eyInt;
    mf->gyro.z += kp_eff * ez + mf->ezInt;

    // 一阶近似，四元数更新
    float q0Last = mf->q0;
    float q1Last = mf->q1;
    float q2Last = mf->q2;
    float q3Last = mf->q3;
    float halfT  = mf->dt * 0.5f;
    mf->q0 += (-q1Last * mf->gyro.x - q2Last * mf->gyro.y - q3Last * mf->gyro.z) * halfT;
    mf->q1 += ( q0Last * mf->gyro.x + q2Last * mf->gyro.z - q3Last * mf->gyro.y) * halfT;
    mf->q2 += ( q0Last * mf->gyro.y - q1Last * mf->gyro.z + q3Last * mf->gyro.x) * halfT;
    mf->q3 += ( q0Last * mf->gyro.z + q1Last * mf->gyro.y - q2Last * mf->gyro.x) * halfT;

    // 单位化四元数
    normalise = invSqrt(mf->q0 * mf->q0 + mf->q1 * mf->q1 + mf->q2 * mf->q2 + mf->q3 * mf->q3);
    mf->q0 *= normalise;
    mf->q1 *= normalise;
    mf->q2 *= normalise;
    mf->q3 *= normalise;

    mf->update_rotation_matrix(mf);
}

void Mahony_Output(Mahony_t *mf)
{
    mf->pitch = -asinf(mf->rMat[2][0]) * RAD2DEG;
    mf->roll  = atan2f(mf->rMat[2][1], mf->rMat[2][2]) * RAD2DEG;
    mf->yaw   = atan2f(mf->rMat[1][0], mf->rMat[0][0]) * RAD2DEG;
}

void Mahony_Init(Mahony_t *mf, float Kp, float Ki, float dt)
{
    mf->Kp = Kp;
    mf->Ki = Ki;
    mf->dt = dt;
    mf->q0 = 1.0f;
    mf->q1 = 0.0f;
    mf->q2 = 0.0f;
    mf->q3 = 0.0f;
    mf->exInt = 0.0f;
    mf->eyInt = 0.0f;
    mf->ezInt = 0.0f;

    mf->rMat[0][0] = 1.0f; mf->rMat[0][1] = 0.0f; mf->rMat[0][2] = 0.0f;
    mf->rMat[1][0] = 0.0f; mf->rMat[1][1] = 1.0f; mf->rMat[1][2] = 0.0f;
    mf->rMat[2][0] = 0.0f; mf->rMat[2][1] = 0.0f; mf->rMat[2][2] = 1.0f;

    mf->init                   = Mahony_Init;
    mf->input                  = Mahony_Input;
    mf->update                 = Mahony_Update;
    mf->output                 = Mahony_Output;
    mf->update_rotation_matrix = mahony_update_rotation_matrix;
}
