#ifndef BMI088_H
#define BMI088_H

#include "spi.h"
#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ========== 硬件映射宏（SysConfig 重新生成后只需修改此部分即可适配 ========== */
#define CS_ACCEL_PORT  GPIO_BMI088_PORT
#define CS_ACCEL_PIN   GPIO_BMI088_CS1_PIN
#define CS_GYRO_PORT   GPIO_BMI088_PORT
#define CS_GYRO_PIN    GPIO_BMI088_CS2_PIN

/* ========== 寄存器地址 ========== */

//加速度计
#define BMI088_REG_ACC_CHIP_ID      0x00
#define BMI088_REG_ACC_X_LSB        0x12
#define BMI088_REG_ACC_PWR_CTRL     0x7D
#define BMI088_REG_ACC_SOFTRESET    0x7E

//陀螺仪
#define BMI088_REG_GYR_CHIP_ID      0x00
#define BMI088_REG_GYR_RANGE        0x0F
#define BMI088_REG_GYR_BANDWIDTH    0x10
#define BMI088_REG_GYR_SOFTRESET    0x14

//软复位命令
#define BMI088_SOFTRESET_VALUE      0xB6

/* ========== 量程选择 ========== */
#define BMI088_ACCEL_RANGE_3G   0x00
#define BMI088_ACCEL_RANGE_6G   0x01
#define BMI088_ACCEL_RANGE_12G  0x02
#define BMI088_ACCEL_RANGE_24G  0x03

#define BMI088_GYRO_RANGE_2000  0x00

/* ========== 灵敏度系数 ========== */
// 加速度计: ±6g → m/s², (6*9.8)/32768 ≈ 0.001795
#define BMI088_ACCEL_SENSITIVITY  0.001795f
// 陀螺仪: ±2000°/s → rad/s, (2000*π/180)/32768 ≈ 0.001065
#define BMI088_GYRO_2000_SEN      0.001065264436f * 9/7

/* ========== 通用 API ========== */

/**
 * @brief 初始化 BMI088，依次软复位加速度计和陀螺仪
 * @param spi 已注册的 SPI 句柄指针
 * @return 0 成功，非 0 失败
 */
uint8_t BMI088_Init(SPI_Handle *spi);

/**
 * @brief 读取加速度计三轴数据
 * @param accel 输出缓冲区，单位 m/s²，顺序 [x, y, z]
 */
void BMI088_ReadAccel(float accel[3]);

/**
 * @brief 读取陀螺仪三轴数据
 * @param gyro 输出缓冲区，单位 rad/s，顺序 [x, y, z]
 */
void BMI088_ReadGyro(float gyro[3]);

/**
 * @brief 读取加速度计芯片 ID
 * @return 芯片 ID（应为 0x1E）
 */
uint8_t BMI088_ReadAccelID(void);

/**
 * @brief 读取陀螺仪芯片 ID
 * @return 芯片 ID（为 0x0F）
 */
uint8_t BMI088_ReadGyroID(void);

/**
 * @brief 裸读寄存器（调试用）
 * @param reg    寄存器地址
 * @param buf    输出缓冲区
 * @param len    读取字节数
 * @param cs_sel 片选: 0 = 加速度计, 1 = 陀螺仪
 */
void BMI088_ReadRawBytes(uint8_t reg, uint8_t *buf,
                         uint8_t len, uint8_t cs_sel);

/**
 * @brief 陀螺仪零偏校准（需在静止状态下调用）
 * @param num_samples 采样次数（建议 ≥200）
 * @note  采集 num_samples 次陀螺仪原始数据取均值作为零偏，
 *         后续 BMI088_ReadGyro() 会自动减去该零偏。
 *         必须在 BMI088_Init() 之后、IMU_Update() 之前调用。
 */
void BMI088_CalibrateGyro(uint16_t num_samples);

#endif
