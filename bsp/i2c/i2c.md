# I2C 通信模块

## 概述

I2C 主机（Controller）通信模块，提供实例注册机制和 SDA 总线解锁恢复功能。支持同时管理最多 2 路 I2C 外设，每路 I2C 可挂载多个从设备。

## 文件

| 文件 | 说明 |
|------|------|
| `i2c.h` | 模块头文件，类型定义与 API 声明 |
| `i2c.c` | 模块实现 |

## 核心机制

### 实例注册表

模块内部维护一个**静态句柄池**（`g_handle_pool`），每个 I2C 外设（如 I2C0、I2C1）只能注册一次。调用 `I2C_Init()` 注册实例，返回句柄指针供后续操作使用。

### SDA 解锁恢复

当 I2C 从设备异常将 SDA 线拉低时（如 OLED 上电时序问题），总线将无法通信。模块提供 `I2C_SDAUnlock()` 函数，通过以下步骤尝试恢复：

1. 禁用 I2C 外设，将 SCL/SDA 引脚切换为 GPIO 模式
2. SCL 输出时钟脉冲（低→高），检测 SDA 是否释放
3. 最多尝试 100 个脉冲周期
4. 恢复 I2C 外设功能

## 类型定义

### `I2C_Config` — 初始化配置结构体

```c
typedef struct {
    I2C_Regs   *i2c;             // I2C 外设基址（如 I2C0）
    GPIO_Regs  *sclPort;         // SCL GPIO 端口
    uint32_t    sclPin;          // SCL GPIO 引脚位掩码
    uint32_t    sclIomux;        // SCL IOMUX 编号
    uint32_t    sclIomuxFunc;    // SCL IOMUX 外设功能号
    GPIO_Regs  *sdaPort;         // SDA GPIO 端口
    uint32_t    sdaPin;          // SDA GPIO 引脚位掩码
    uint32_t    sdaIomux;        // SDA IOMUX 编号
    uint32_t    sdaIomuxFunc;    // SDA IOMUX 外设功能号
    void      (*syscfgInit)(void); // SysConfig 生成的 I2C 初始化函数
} I2C_Config;
```

### `I2C_Handle` — 运行时句柄

与 `I2C_Config` 字段一致，由 `I2C_Init()` 内部填充，调用者只需保存指针。

## API

### `I2C_Handle* I2C_Init(const I2C_Config *config)`

注册并初始化一个 I2C 实例。

| 参数 | 说明 |
|------|------|
| `config` | 初始化配置结构体指针 |

| 返回值 | 说明 |
|--------|------|
| 非 NULL | 句柄指针，注册成功 |
| NULL | 注册失败（重复注册 / 硬件索引无效 / 实例已满） |

### `void I2C_Write(I2C_Handle *h, uint8_t devAddr, uint8_t *data, uint16_t len)`

通过 I2C 向从设备**发送**数据（阻塞式，带超时）。

| 参数 | 说明 |
|------|------|
| `h` | I2C 句柄指针 |
| `devAddr` | 从设备 7 位地址（如 OLED 为 `0x3C`） |
| `data` | 待发送数据缓冲区 |
| `len` | 数据长度（字节） |

**超时处理：** 发送超时后自动调用 `I2C_SDAUnlock()` 尝试恢复总线。

> **注意：** 本模块目前仅实现主机发送（Controller TX），未实现主机接收（Controller RX）。OLED 等显示设备只需单向写入即可工作。

### `void I2C_SDAUnlock(I2C_Handle *h)`

I2C SDA 总线解锁恢复（见上文"核心机制"）。

| 参数 | 说明 |
|------|------|
| `h` | I2C 句柄指针 |

## 依赖

- `ti_msp_dl_config.h`（提供 I2C 和 GPIO 外设驱动 API）
- `delay.h`（解锁过程中的延时）

## 使用示例

```c
#include "i2c.h"

// 1. 定义配置（参数由 SysConfig 生成）
I2C_Config i2c_cfg = {
    .i2c          = I2C_OLED_INST,
    .sclPort      = GPIO_I2C_OLED_SCL_PORT,
    .sclPin       = GPIO_I2C_OLED_SCL_PIN,
    .sclIomux     = GPIO_I2C_OLED_IOMUX_SCL,
    .sclIomuxFunc = GPIO_I2C_OLED_IOMUX_SCL_FUNC,
    .sdaPort      = GPIO_I2C_OLED_SDA_PORT,
    .sdaPin       = GPIO_I2C_OLED_SDA_PIN,
    .sdaIomux     = GPIO_I2C_OLED_IOMUX_SDA,
    .sdaIomuxFunc = GPIO_I2C_OLED_IOMUX_SDA_FUNC,
    .syscfgInit   = SYSCFG_DL_I2C_OLED_init,
};

// 2. 注册实例
I2C_Handle *i2c = I2C_Init(&i2c_cfg);

// 3. 向从设备发送数据
uint8_t buf[2] = {0x00, 0xAF};  // 命令 + 数据
I2C_Write(i2c, 0x3C, buf, 2);
```
