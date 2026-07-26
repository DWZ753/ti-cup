#ifndef MAHONY_H
#define MAHONY_H

#include "deewaz_math.h"

/* Mahony 互补滤波器句柄 */
typedef struct Mahony_t
{
    // 滤波器参数
    float Kp;  // 比例增益
    float Ki;  // 积分增益
    float dt;  // 采样周期 (s)

    // 传感器原始数据
    Axis3f gyro;  // 陀螺仪角速度 (rad/s)
    Axis3f acc;   // 加速度计 (m/s²)

    // 积分误差
    float exInt, eyInt, ezInt;

    // 四元数
    float q0, q1, q2, q3;

    // 旋转矩阵
    float rMat[3][3];

    // 欧拉角 (°)
    float pitch, roll, yaw;

    // 内部方法（由 Mahony_Init 绑定）
    void (*init)(struct Mahony_t *mf, float Kp, float Ki, float dt);
    void (*input)(struct Mahony_t *mf, Axis3f gyro, Axis3f acc);
    void (*update)(struct Mahony_t *mf);
    void (*output)(struct Mahony_t *mf);
    void (*update_rotation_matrix)(struct Mahony_t *mf);
} Mahony_t;

/* ========== 通用 API ========== */

/**
 * @brief  初始化 Mahony 滤波器，配置增益与采样周期
 * @param  mf Mahony 滤波器句柄指针
 * @param  Kp 比例增益
 * @param  Ki 积分增益
 * @param  dt 采样周期 (s)
 */
void Mahony_Init(Mahony_t *mf, float Kp, float Ki, float dt);

/**
 * @brief  输入最新陀螺仪和加速度计数据
 * @param  mf   Mahony 滤波器句柄指针
 * @param  gyro 陀螺仪角速度 (rad/s)
 * @param  acc  加速度计 (m/s²)
 */
void Mahony_Input(Mahony_t *mf, Axis3f gyro, Axis3f acc);

/**
 * @brief  执行一次 Mahony 互补滤波更新
 * @param  mf Mahony 滤波器句柄指针
 */
void Mahony_Update(Mahony_t *mf);

/**
 * @brief  从旋转矩阵解算欧拉角
 * @param  mf Mahony 滤波器句柄指针
 */
void Mahony_Output(Mahony_t *mf);

#endif
