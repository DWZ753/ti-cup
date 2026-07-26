#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ========== 硬件映射宏（移植到其他MCU时修改此部分） ========== */
#define SERVO_PWM_INST              PWM_SERVO_INST              // 定时器外设实例
#define SERVO_PWM_CHANNEL           GPIO_PWM_SERVO_C0_IDX       // 捕获比较通道索引
#define SERVO_PWM_CLK_FREQ          PWM_SERVO_INST_CLK_FREQ     // PWM定时器时钟频率(Hz)

/* ========== 舵机参数宏（更换舵机型号时修改此部分） ========== */
#define SERVO_PULSE_MIN_US          1000    // -100 对应脉宽(us)
#define SERVO_PULSE_MAX_US          2000    // +100 对应脉宽(us)
#define SERVO_PULSE_CENTER_US       1500    //    0 对应脉宽(us)，中心位置

/**
 * @brief 舵机初始化，启动 PWM 输出并将舵机复位到中心位置
 */
void Servo_Init(void);

/**
 * @brief 将舵机控制值限制在有效范围 [-100, 100] 内
 * @param value 原始控制值
 * @return 限制后的值
 */
int32_t Servo_LimitValue(int32_t value);

/**
 * @brief 设置舵机角度
 * @param value 控制值，范围 -100（最小角度）到 100（最大角度），0 为中心位置
 */
void Servo_SetValue(int32_t value);

#endif
