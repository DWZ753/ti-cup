#include "imu.h"
#include "ti_msp_dl_config.h"
#include "spi.h"
#include "bmi088.h"
#include "mahony.h"

/* ========== 内部静态句柄 ========== */

static SPI_Handle *spi_bmi088;
static Mahony_t   mahony;
static float      accel[3], gyro[3];
static Axis3f     gyro_axis, acc_axis;

/* ========== 公共 API ========== */

void IMU_Init(void)
{
    SPI_Config spi_cfg = { .spi = SPI_BMI088_INST };
    spi_bmi088 = SPI_Init(&spi_cfg);

    BMI088_Init(spi_bmi088);

    BMI088_CalibrateGyro(200);

    Mahony_Init(&mahony, IMU_MAHONY_KP, IMU_MAHONY_KI, IMU_MAHONY_DT);
}

void IMU_Update(void)
{
    BMI088_ReadAccel(accel);
    BMI088_ReadGyro(gyro);

    acc_axis.x  = accel[0]; acc_axis.y  = accel[1]; acc_axis.z  = accel[2];
    gyro_axis.x = gyro[0];  gyro_axis.y = gyro[1];  gyro_axis.z = gyro[2];

    Mahony_Input(&mahony, gyro_axis, acc_axis);
    Mahony_Update(&mahony);
    Mahony_Output(&mahony);
}

void IMU_GetEuler(float *roll, float *pitch, float *yaw)
{
    *roll  = mahony.roll;
    *pitch = mahony.pitch;
    *yaw   = mahony.yaw;
}

void IMU_GetAccel(float out[3])
{
    out[0] = accel[0];
    out[1] = accel[1];
    out[2] = accel[2];
}

void IMU_GetGyro(float out[3])
{
    out[0] = gyro[0];
    out[1] = gyro[1];
    out[2] = gyro[2];
}

void IMU_GetQuaternion(float *q0, float *q1, float *q2, float *q3)
{
    *q0 = mahony.q0;
    *q1 = mahony.q1;
    *q2 = mahony.q2;
    *q3 = mahony.q3;
}
