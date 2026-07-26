#include "key.h"
#include "ti_msp_dl_config.h"
#include "pit_control_tick.h"

// 按键触发回调
static Key_Callback_t s_key_callback = NULL;

// 按键硬件配置数组，移植后需要根据实际情况修改
static const struct {
    int pin;
    KeyActiveLevel_t active_level;
} key_config[KEY_NUM] = {
    {GPIO_KEYs_KEY1_PIN, KEY_ACTIVE_HIGH},
    {GPIO_KEYs_KEY2_PIN, KEY_ACTIVE_HIGH},
    {GPIO_KEYs_KEY3_PIN, KEY_ACTIVE_HIGH},
    {GPIO_KEYs_KEY4_PIN, KEY_ACTIVE_HIGH}
};

// 按键数组
Key_t keys[KEY_NUM];

static void Key_TickHandler(void);


void Key_Init(void)
{
    // 初始化所有按键的配置
    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
        keys[i].pin = key_config[i].pin;
        keys[i].active_level = key_config[i].active_level;
        keys[i].state = KEY_STATE_IDLE;
        keys[i].debounce_cnt = 0;
        keys[i].flag = 0;
    }

    // 注册定时器Tick回调函数，实现周期性按键扫描
    PIT_Control_Tick_RegisterCallback(Key_TickHandler);
}


/**
 * @brief 读取按键的逻辑状态
 * 
 * 根据按键配置的极性（高电平有效或低电平有效），将GPIO原始电平转换为逻辑值。
 * 
 * @param key_index 按键索引，用于从keys数组中获取对应按键的配置信息
 * @return uint8_t 按键的逻辑状态：1表示按键按下，0表示按键未按下
 */
static uint8_t Key_ReadLogic(uint8_t key_index)
{
    // 读取GPIO原始电平
    uint32_t raw_level = DL_GPIO_readPins(KEY_PORT, keys[key_index].pin);
    
    // 根据配置的极性进行转换
    if (keys[key_index].active_level == KEY_ACTIVE_LOW)
    {
        // 低电平有效
        return !raw_level ? 1 : 0;
    }
    else
    {
        // 高电平有效
        return raw_level ? 1 : 0;
    }
}


void Key_Scan(void)
{
    for (uint8_t i = 0; i < KEY_NUM; i++) {
        // 获取统一的逻辑电平
        uint8_t logic_level = Key_ReadLogic(i);
        
        switch (keys[i].state)
        {
            case KEY_STATE_IDLE:
                // 空闲状态：检测是否有按下动作
                if (logic_level == 1)
                {
                    keys[i].debounce_cnt++;
                    if (keys[i].debounce_cnt >= KEY_DEBOUNCE_COUNT)
                    {
                        // 连续检测到按下，进入按下状态
                        keys[i].state = KEY_STATE_PRESSED;
                        keys[i].debounce_cnt = 0;
                    }
                }
                else
                {
                    keys[i].debounce_cnt = 0;
                }
                break;
                
            case KEY_STATE_PRESSED:
                // 按下确认状态：等待按键释放
                if (logic_level == 0)
                {
                    keys[i].debounce_cnt++;
                    if (keys[i].debounce_cnt >= KEY_DEBOUNCE_COUNT)
                    {
                        // 连续检测到释放，进入释放状态
                        keys[i].state = KEY_STATE_RELEASED;
                        keys[i].flag = 1;      // 置位触发标志
                        keys[i].debounce_cnt = 0;
                    }
                }
                break;
                
            case KEY_STATE_RELEASED:
                // 已触发状态：保持标志位，直到主程序通过 Key_GetFlag 读取并清除
                break;
                
            default:
                // 异常状态复位
                keys[i].state = KEY_STATE_IDLE;
                keys[i].debounce_cnt = 0;
                break;
        }
    }
}


uint8_t Key_GetFlag(uint8_t key_index)
{
    // 边界检测
    if (key_index >= KEY_NUM)
    {
        return 0;
    }
    
    uint8_t flag = keys[key_index].flag;
    if (flag)
    {
        // 清除标志并重置状态，准备下一次检测
        keys[key_index].flag = 0;
        keys[key_index].state = KEY_STATE_IDLE;
    }
    return flag;
}


void Key_RegisterCallback(Key_Callback_t callback)
{
    s_key_callback = callback;
}

/**
 * @brief 按键定时处理回调函数
 * @note 作为 PIT 定时器 Tick 回调周期性执行按键扫描，检测到触发时调用注册的回调
 */
static void Key_TickHandler(void)
{
    Key_Scan();
    if (s_key_callback == NULL) return;

    for (int i = 0; i < KEY_NUM; i++)
    {
        if (Key_GetFlag(i))
        {
            s_key_callback(i);
        }
    }
}