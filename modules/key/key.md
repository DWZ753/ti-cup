# Key 按键模块

## 概述

按键输入模块，支持最多 4 个独立按键。内置**有限状态机（FSM）消抖算法**，支持高/低电平有效可配，通过 PIT Control Tick（20ms 定时器）周期性扫描。提供两种使用方式：**回调注册**和**轮询标志位**。

## 文件

| 文件 | 说明 |
|------|------|
| `key.h` | 模块头文件，枚举/结构体定义与 API 声明 |
| `key.c` | 模块实现 |

## 核心机制

### 状态机消抖

每个按键使用三态 FSM 实现消抖，无需阻塞延时：

```
IDLE（空闲）
  ↓ GPIO 检测到按下 → 消抖计数器++
  ↓ 连续 N 次按下 → 进入 PRESSED
PRESSED（按下确认）
  ↓ GPIO 检测到释放 → 消抖计数器++
  ↓ 连续 N 次释放 → 进入 RELEASED，置 trigger flag
RELEASED（已触发）
  ↓ 等待主程序通过 Key_GetFlag() 读取标志 → 回到 IDLE
```

消抖次数由 `KEY_DEBOUNCE_COUNT` 定义（默认 3 次 × 20ms = 60ms 消抖窗口）。

### 极性适配

通过 `active_level` 字段支持两种按键接线方式：

| `active_level` | 含义 |
|----------------|------|
| `KEY_ACTIVE_LOW` | 低电平有效（按键一端接 GND，需要上拉） |
| `KEY_ACTIVE_HIGH` | 高电平有效（按键一端接 VCC，需要下拉） |

`Key_ReadLogic()` 自动将原始 GPIO 电平转换为统一的逻辑值（1 = 按下，0 = 释放）。

## 宏定义

```c
#define KEY_PORT               GPIO_KEYs_PORT    // 按键 GPIO 端口
#define KEY_NUM                4                 // 按键数量
#define KEY_DEBOUNCE_COUNT     3                 // 消抖确认次数（×20ms = 60ms）
```

## 类型定义

### `KeyActiveLevel_t` — 有效电平枚举

```c
typedef enum {
    KEY_ACTIVE_LOW = 0,   // 低电平有效
    KEY_ACTIVE_HIGH       // 高电平有效
} KeyActiveLevel_t;
```

### `KeyState_t` — 按键状态枚举

```c
typedef enum {
    KEY_STATE_IDLE = 0,      // 空闲
    KEY_STATE_PRESSED,       // 按下（消抖中）
    KEY_STATE_RELEASED       // 释放（已触发）
} KeyState_t;
```

### `Key_t` — 单个按键结构体

```c
typedef struct {
    uint32_t pin;                    // 按键 GPIO 引脚
    KeyActiveLevel_t active_level;  // 有效电平配置
    KeyState_t state;               // 当前状态
    uint8_t debounce_cnt;           // 消抖计数器
    uint8_t flag;                   // 按键触发标志
} Key_t;
```

### `Key_Callback_t` — 按键回调类型

```c
typedef void (*Key_Callback_t)(uint8_t key_index);
```

## API

### `void Key_Init(void)`

初始化按键模块。遍历配置数组初始化所有按键状态，注册 PIT Control Tick 回调（20ms 周期扫描）。

> 此函数由 `Board_Init()` 自动调用。

### `void Key_Scan(void)`

执行一次按键扫描（状态机处理）。由定时器回调自动调用，用户通常无需直接调用。

### `uint8_t Key_GetFlag(uint8_t key_index)`

获取按键触发标志并**自动清除**（一次性消费）。

| 参数 | 说明 |
|------|------|
| `key_index` | 按键索引（0 ~ KEY_NUM-1） |

| 返回值 | 说明 |
|--------|------|
| 非 0 | 按键已触发（完整一次按下→释放） |
| 0 | 未触发或索引无效 |

> **注意：** 调用后标志自动清除，状态机回到 IDLE。同一按键在一次触发后必须等待下一次按下→释放才会再次返回非 0。

### `void Key_RegisterCallback(Key_Callback_t callback)`

注册按键触发回调函数。

| 参数 | 说明 |
|------|------|
| `callback` | 回调函数指针，按键触发时被调用并传入按键索引 |

注册后，当任意按键触发（完成消抖的按下→释放流程），回调函数会被调用并传入按键索引。回调在 **PIT ISR 上下文**中执行，应尽量简短。

## 依赖

- `ti_msp_dl_config.h`（提供 GPIO 引脚宏）
- [PIT Control Tick 模块](../pit_tick/pit_tick.md)（20ms 定时器回调）

## 使用示例

### 方式一：回调注册（推荐）

```c
#include "key.h"

// 按键触发处理函数
void Key_Handler(uint8_t key_index) {
    switch (key_index) {
        case 0: /* KEY1 处理 */ break;
        case 1: /* KEY2 处理 */ break;
        case 2: /* KEY3 处理 */ break;
        case 3: /* KEY4 处理 */ break;
    }
}

void app_init(void) {
    Key_RegisterCallback(Key_Handler);
}
```

### 方式二：轮询标志位

```c
#include "key.h"

void main_loop(void) {
    // 在主循环中轮询
    if (Key_GetFlag(0)) {
        // KEY1 被按下（完成一次按下→释放）
    }
}
```

> **注意：** 如果同时使用回调注册和标志位轮询，回调会先于标志位消费触发事件（内部的 `Key_TickHandler` 先调用 `Key_GetFlag` 再调用用户回调）。

## 自定义按键配置

如需修改按键引脚或有效电平，编辑 `key.c` 中的 `key_config` 数组：

```c
static const struct {
    int pin;
    KeyActiveLevel_t active_level;
} key_config[KEY_NUM] = {
    {GPIO_KEYs_KEY1_PIN, KEY_ACTIVE_HIGH},
    {GPIO_KEYs_KEY2_PIN, KEY_ACTIVE_HIGH},
    {GPIO_KEYs_KEY3_PIN, KEY_ACTIVE_HIGH},
    {GPIO_KEYs_KEY4_PIN, KEY_ACTIVE_HIGH}
};
```
