#include "i2c.h"
#include "ti_msp_dl_config.h"
#include "delay.h"

#define I2C_MAX_INSTANCES 2
#define I2C_TIMEOUT_CNT   100000

/* ========== 句柄注册表 ========== */
static I2C_Handle g_handle_pool[I2C_MAX_INSTANCES];
static I2C_Handle *g_i2c_instances[I2C_MAX_INSTANCES];
static uint8_t g_i2c_count;

/**
 * @brief 根据 I2C 外设基址确定硬件索引
 * @param i2c I2C 外设基址指针
 * @return 硬件索引 0~1，未匹配返回 I2C_MAX_INSTANCES
 */
static uint8_t get_i2c_index(I2C_Regs *i2c)
{
    if (i2c == I2C0) return 0;
    if (i2c == I2C1) return 1;
    return I2C_MAX_INSTANCES;
}

/**
 * @brief 禁用 I2C 外设，将 SCL/SDA 引脚临时切换为 GPIO 模式
 *
 * 用于解锁 I2C 总线（当 SDA 被从设备意外拉低时）：
 * 复位 I2C 外设，SCL 配置为 GPIO 输出低电平，SDA 配置为 GPIO 输入。
 *
 * @param h I2C 句柄指针
 */
static void i2c_disable(I2C_Handle *h)
{
    DL_I2C_reset(h->i2c);
    DL_GPIO_initDigitalOutput(h->sclIomux);
    DL_GPIO_initDigitalInputFeatures(h->sdaIomux,
             DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
             DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(h->sclPort, h->sclPin);
    DL_GPIO_enableOutput(h->sclPort, h->sclPin);
}

/**
 * @brief 重新启用 I2C 外设，将 SCL/SDA 引脚恢复为外设功能模式
 *
 * 与 i2c_disable() 配对使用，在完成 GPIO 位操作后恢复 I2C 通信功能。
 *
 * @param h I2C 句柄指针
 */
static void i2c_enable(I2C_Handle *h)
{
    DL_I2C_reset(h->i2c);
    DL_GPIO_initPeripheralInputFunctionFeatures(h->sdaIomux,
        h->sdaIomuxFunc, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(h->sclIomux,
        h->sclIomuxFunc, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(h->sdaIomux);
    DL_GPIO_enableHiZ(h->sclIomux);
    DL_I2C_enablePower(h->i2c);
    h->syscfgInit();
}

/* ========== 通用 API ========== */

I2C_Handle* I2C_Init(const I2C_Config *config)
{
    uint8_t idx;

    if (config == NULL || config->i2c == NULL) return NULL;
    idx = get_i2c_index(config->i2c);
    if (idx >= I2C_MAX_INSTANCES) return NULL;
    if (g_i2c_instances[idx] != NULL) return NULL;
    if (g_i2c_count >= I2C_MAX_INSTANCES) return NULL;

    I2C_Handle *h = &g_handle_pool[g_i2c_count];

    h->i2c          = config->i2c;
    h->sclPort      = config->sclPort;
    h->sclPin       = config->sclPin;
    h->sclIomux     = config->sclIomux;
    h->sclIomuxFunc = config->sclIomuxFunc;
    h->sdaPort      = config->sdaPort;
    h->sdaPin       = config->sdaPin;
    h->sdaIomux     = config->sdaIomux;
    h->sdaIomuxFunc = config->sdaIomuxFunc;
    h->syscfgInit   = config->syscfgInit;

    g_i2c_instances[idx] = h;
    g_i2c_count++;

    return h;
}

void I2C_Write(I2C_Handle *h, uint8_t devAddr, uint8_t *data, uint16_t len)
{
    unsigned long timeout;

    DL_I2C_fillControllerTXFIFO(h->i2c, data, len);
    DL_I2C_clearInterruptStatus(h->i2c, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    while (!(DL_I2C_getControllerStatus(h->i2c) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(h->i2c, devAddr, DL_I2C_CONTROLLER_DIRECTION_TX, len);

    timeout = I2C_TIMEOUT_CNT;
    while (!DL_I2C_getRawInterruptStatus(h->i2c, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))
    {
        if (--timeout == 0)
        {
            I2C_SDAUnlock(h);
            break;
        }
    }
}

void I2C_SDAUnlock(I2C_Handle *h)
{
    uint8_t cycleCnt = 0;

    i2c_disable(h);
    do
    {
        DL_GPIO_clearPins(h->sclPort, h->sclPin);
        delay_ms(1);
        DL_GPIO_setPins(h->sclPort, h->sclPin);
        delay_ms(1);

        if (DL_GPIO_readPins(h->sdaPort, h->sdaPin))
            break;
    } while (++cycleCnt < 100);
    i2c_enable(h);
}
