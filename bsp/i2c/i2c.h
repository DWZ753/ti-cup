#ifndef __USER_I2C_H__
#define __USER_I2C_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ========== 类型定义 ========== */

/** I2C 初始化配置 */
typedef struct {
    I2C_Regs   *i2c;             // I2C 外设基址
    GPIO_Regs  *sclPort;         // SCL GPIO 端口（SDA 解锁时用）
    uint32_t    sclPin;          // SCL GPIO 引脚位掩码
    uint32_t    sclIomux;        // SCL IOMUX
    uint32_t    sclIomuxFunc;    // SCL IOMUX 外设功能
    GPIO_Regs  *sdaPort;         // SDA GPIO 端口（SDA 解锁时用）
    uint32_t    sdaPin;          // SDA GPIO 引脚位掩码
    uint32_t    sdaIomux;        // SDA IOMUX
    uint32_t    sdaIomuxFunc;    // SDA IOMUX 外设功能
    void      (*syscfgInit)(void); // SysConfig 生成的初始化函数
} I2C_Config;

/** I2C 运行时句柄 */
typedef struct {
    I2C_Regs   *i2c;
    GPIO_Regs  *sclPort;
    uint32_t    sclPin;
    uint32_t    sclIomux;
    uint32_t    sclIomuxFunc;
    GPIO_Regs  *sdaPort;
    uint32_t    sdaPin;
    uint32_t    sdaIomux;
    uint32_t    sdaIomuxFunc;
    void      (*syscfgInit)(void);
} I2C_Handle;

/* ========== 通用 API ========== */

/**
 * @brief 注册并初始化一个 I2C 实例
 * @param config 初始化配置（指定外设、引脚、syscfg 回调等）
 * @return 成功返回句柄指针，失败返回 NULL（重复注册 / 硬件索引无效 / 实例已满）
 */
I2C_Handle* I2C_Init(const I2C_Config *config);

/**
 * @brief 通过 I2C 向从设备发送数据（阻塞，带超时和 SDA 解锁恢复）
 * @param h       I2C 句柄指针
 * @param devAddr 从设备地址（7 位地址，如 0x3C）
 * @param data    待发送数据缓冲区
 * @param len     数据长度（字节）
 * @note 超时后自动调用 I2C_SDAUnlock 尝试恢复总线
 */
void I2C_Write(I2C_Handle *h, uint8_t devAddr, uint8_t *data, uint16_t len);

/**
 * @brief I2C SDA 总线解锁（从 SDA 被从设备拉低的状态恢复）
 * @param h I2C 句柄指针
 * @note 通过 GPIO 模拟 SCL 时钟脉冲来释放 SDA 线，每脉冲间隔 1ms，最多 100 次
 */
void I2C_SDAUnlock(I2C_Handle *h);

#endif
