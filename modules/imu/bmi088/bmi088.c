#include "bmi088.h"
#include "ti_msp_dl_config.h"
#include "delay.h"

static SPI_Handle *g_bmi088_spi;

/* 陀螺仪零偏（原始 ADC 值），由 BMI088_CalibrateGyro() 写入 */
static int16_t g_gyro_offset_raw[3] = {0, 0, 0};

/**
 * @brief  拉低 CS 选中设备
 * @param  cs_sel 0=加速度计, 1=陀螺仪
 */
static void bmi088_cs_select(uint8_t cs_sel)
{
    if (cs_sel == 0) {
        DL_GPIO_clearPins(CS_ACCEL_PORT, CS_ACCEL_PIN);
    } else {
        DL_GPIO_clearPins(CS_GYRO_PORT, CS_GYRO_PIN);
    }
}

/**
 * @brief  拉高 CS 释放设备
 * @param  cs_sel 0=加速度计, 1=陀螺仪
 */
static void bmi088_cs_deselect(uint8_t cs_sel)
{
    if (cs_sel == 0) {
        DL_GPIO_setPins(CS_ACCEL_PORT, CS_ACCEL_PIN);
    } else {
        DL_GPIO_setPins(CS_GYRO_PORT, CS_GYRO_PIN);
    }
}

/**
 * @brief  通过 SPI 向寄存器写入单字节数据
 * @param  reg    寄存器地址
 * @param  data   写入数据
 * @param  cs_sel 片选: 0=加速度计, 1=陀螺仪
 */
static void bmi088_write_reg(uint8_t reg, uint8_t data, uint8_t cs_sel)
{
    bmi088_cs_select(cs_sel);

    SPI_Transfer(g_bmi088_spi, reg & 0x7F);
    SPI_Transfer(g_bmi088_spi, data);

    delay_ms(1);

    bmi088_cs_deselect(cs_sel);
}

/**
 * @brief  通过 SPI 从寄存器连续读取多字节数据
 * @param  reg    寄存器地址
 * @param  buf    输出缓冲区
 * @param  len    读取字节数
 * @param  cs_sel 片选: 0=加速度计, 1=陀螺仪
 */
static void bmi088_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len, uint8_t cs_sel)
{
    bmi088_cs_select(cs_sel);

    SPI_Transfer(g_bmi088_spi, reg | 0x80);

    if (cs_sel == 0) {
        SPI_Transfer(g_bmi088_spi, 0xFF);
    }

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = SPI_Transfer(g_bmi088_spi, 0xFF);
    }

    bmi088_cs_deselect(cs_sel);
}

void BMI088_ReadRawBytes(uint8_t reg, uint8_t *buf, uint8_t len, uint8_t cs_sel)
{
    bmi088_read_bytes(reg, buf, len, cs_sel);
}

uint8_t BMI088_ReadAccelID(void)
{
    uint8_t id;
    bmi088_read_bytes(BMI088_REG_ACC_CHIP_ID, &id, 1, 0);
    return id;
}

uint8_t BMI088_ReadGyroID(void)
{
    uint8_t id;
    bmi088_read_bytes(BMI088_REG_GYR_CHIP_ID, &id, 1, 1);
    return id;
}

/**
 * @brief  加速度计初始化：软复位 → 上电
 * @return 0 成功
 */
static uint8_t bmi088_accel_init(void)
{
    bmi088_write_reg(BMI088_REG_ACC_SOFTRESET, BMI088_SOFTRESET_VALUE, 0);
    delay_ms(10);

    bmi088_write_reg(BMI088_REG_ACC_PWR_CTRL, 0x04, 0);
    delay_ms(5);

    return 0;
}

/**
 * @brief  陀螺仪初始化：软复位
 * @return 0 成功
 */
static uint8_t bmi088_gyro_init(void)
{
    bmi088_write_reg(BMI088_REG_GYR_SOFTRESET, BMI088_SOFTRESET_VALUE, 1);
    delay_ms(30);

    /* 量程 ±2000°/s（寄存器 0x0F = 0x00，复位后默认值，显式确认） */
    bmi088_write_reg(BMI088_REG_GYR_RANGE, 0x00, 1);
    delay_ms(1);

    /* 低通滤波器带宽 116Hz（寄存器 0x10 = 0x02, ODR 2kHz）
     * 适配 100Hz 采样率，防混叠同时保持足够响应速度 */
    bmi088_write_reg(BMI088_REG_GYR_BANDWIDTH, 0x02, 1);
    delay_ms(1);

    return 0;
}

uint8_t BMI088_Init(SPI_Handle *spi)
{
    g_bmi088_spi = spi;
    uint8_t err = 0;
    err |= bmi088_accel_init();
    err |= bmi088_gyro_init();
    return err;
}

void BMI088_ReadAccel(float accel[3])
{
    uint8_t buf[6];
    bmi088_read_bytes(BMI088_REG_ACC_X_LSB, buf, 6, 0);

    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    accel[0] = (float)raw_x * BMI088_ACCEL_SENSITIVITY;
    accel[1] = (float)raw_y * BMI088_ACCEL_SENSITIVITY;
    accel[2] = (float)raw_z * BMI088_ACCEL_SENSITIVITY;
}

void BMI088_CalibrateGyro(uint16_t num_samples)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < num_samples; i++)
    {
        uint8_t buf[8] = {0};
        bmi088_read_bytes(BMI088_REG_GYR_CHIP_ID, buf, 8, 1);

        if (buf[0] == 0x0F)
        {
            sum_x += (int16_t)((buf[3] << 8) | buf[2]);
            sum_y += (int16_t)((buf[5] << 8) | buf[4]);
            sum_z += (int16_t)((buf[7] << 8) | buf[6]);
            valid++;
        }

        delay_ms(2);  /* 等待下一次陀螺仪数据更新 (ODR ≈ 2kHz) */
    }

    if (valid > 0)
    {
        g_gyro_offset_raw[0] = (int16_t)(sum_x / valid);
        g_gyro_offset_raw[1] = (int16_t)(sum_y / valid);
        g_gyro_offset_raw[2] = (int16_t)(sum_z / valid);
    }
}

void BMI088_ReadGyro(float gyro[3])
{
    uint8_t buf[8] = {0};
    bmi088_read_bytes(BMI088_REG_GYR_CHIP_ID, buf, 8, 1);

    if (buf[0] == 0x0F) {
        int16_t raw_x = (int16_t)((buf[3] << 8) | buf[2]) - g_gyro_offset_raw[0];
        int16_t raw_y = (int16_t)((buf[5] << 8) | buf[4]) - g_gyro_offset_raw[1];
        int16_t raw_z = (int16_t)((buf[7] << 8) | buf[6]) - g_gyro_offset_raw[2];

        gyro[0] = (float)raw_x * BMI088_GYRO_2000_SEN;
        gyro[1] = (float)raw_y * BMI088_GYRO_2000_SEN;
        gyro[2] = (float)raw_z * BMI088_GYRO_2000_SEN;
    }
}
