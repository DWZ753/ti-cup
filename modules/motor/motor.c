/* 本模块在使用tb6612电机驱动芯片时不需要修改
 * @todo 添加使用其他电机驱动芯片的接口
 */
#include "motor.h"
#include "ti_msp_dl_config.h"
#include "tb6612.h"
#include "pit_control_tick.h"
#include <stdbool.h>

// PWM 占空比下限：低于此值电机无法转动
#define MOTOR_MIN_DUTY 100

// 编码器 A 相引脚（内部使用，不含在 .h 以避免依赖 ti_msp_dl_config.h）
#define MOTOR_ENCODER1_OUT_A_PORT  GPIO_MOTORs_GPIO_MOTOR1_OUT_A_PORT
#define MOTOR_ENCODER1_OUT_A_PIN   GPIO_MOTORs_GPIO_MOTOR1_OUT_A_PIN
#define MOTOR_ENCODER1_OUT_A_IIDX  GPIO_MOTORs_GPIO_MOTOR1_OUT_A_IIDX
#define MOTOR_ENCODER2_OUT_A_PORT  GPIO_MOTORs_GPIO_MOTOR2_OUT_A_PORT
#define MOTOR_ENCODER2_OUT_A_PIN   GPIO_MOTORs_GPIO_MOTOR2_OUT_A_PIN
#define MOTOR_ENCODER2_OUT_A_IIDX  GPIO_MOTORs_GPIO_MOTOR2_OUT_A_IIDX

// 编码器 B 相引脚（仅读电平，不触发中断）
#define MOTOR_ENCODER1_OUT_B_PORT  GPIO_MOTORs_GPIO_MOTOR1_OUT_B_PORT
#define MOTOR_ENCODER1_OUT_B_PIN   GPIO_MOTORs_GPIO_MOTOR1_OUT_B_PIN
#define MOTOR_ENCODER2_OUT_B_PORT  GPIO_MOTORs_GPIO_MOTOR2_OUT_B_PORT
#define MOTOR_ENCODER2_OUT_B_PIN   GPIO_MOTORs_GPIO_MOTOR2_OUT_B_PIN

static volatile int32_t g_encoder1_pulse;
static volatile int32_t g_encoder2_pulse;
/*
 * 原始速度（float，仅在 ISR 中写入，仅原始 Getter 使用）
 * 改为 int32_t 定点可消除 ISR 软浮点开销，但原始值未参与滤波链，
 * 保持 float 以便调试时直接查看 RPM/mm/s 值。
 */
static volatile float   g_encoder1_rpm;
static volatile float   g_encoder2_rpm;
static volatile float   g_encoder1_speed;
static volatile float   g_encoder2_speed;

/*
 * 滤波后速度/转速 — int32_t 定点（×1000），消除 ISR 中软浮点运算。
 * mm/s → μm/s，RPM → mRPM。M0+ 上对齐 int32 读写是原子的，volatile
 * 防止编译器缓存。Getter 内部转换回 float 以保持 API 兼容。
 */
static volatile int32_t g_encoder1_rpm_f;    // 滤波后 RPM × 1000
static volatile int32_t g_encoder2_rpm_f;
static volatile int32_t g_encoder1_speed_f;  // 滤波后线速度 μm/s
static volatile int32_t g_encoder2_speed_f;

/* ---------- 窗口累积器（TickHandler 内部使用） ---------- */

static int32_t g_enc1_last;          // 上一次脉冲读数
static int32_t g_enc2_last;
static int32_t g_enc1_diff_sum;      // 窗口内脉冲差值累加
static int32_t g_enc2_diff_sum;
static uint8_t g_tick_count;         // 当前窗口已累积的 tick 数
static bool    g_first_window;       // 首个窗口标志（EMA 初始化用）

void Motor_Init(void)
{
    Motor_Stop();
    Motor_ResetEncoder();
    NVIC_EnableIRQ(GPIO_MOTORs_INT_IRQN);
    PIT_Control_Tick_RegisterCallback(Motor_TickHandler);
}

void Motor_SetSpeed(float speed_mm_s)
{
    float abs_speed = (speed_mm_s >= 0.0f) ? speed_mm_s : -speed_mm_s;

    uint32_t duty = (uint32_t)(abs_speed / MOTOR_MAX_SPEED_MM_S * MOTOR_MAX_PWM_DUTY);

    if (duty > MOTOR_MAX_PWM_DUTY)
        duty = MOTOR_MAX_PWM_DUTY;

    if (duty < MOTOR_MIN_DUTY)
    {
        Motor_Brake();
        return;
    }

    if (speed_mm_s >= 0.0f)
    {
        TB6612_A_Forward(duty);
        TB6612_B_Backward(duty);
    }
    else
    {
        TB6612_A_Backward(duty);
        TB6612_B_Forward(duty);
    }
}

void Motor_SetSpeedLR(float left_mm_s, float right_mm_s)
{
    /* ---- 左轮（A 通道） ---- */
    float abs_left = (left_mm_s >= 0.0f) ? left_mm_s : -left_mm_s;
    uint32_t duty_left = (uint32_t)(abs_left / MOTOR_MAX_SPEED_MM_S * MOTOR_MAX_PWM_DUTY);
    if (duty_left > MOTOR_MAX_PWM_DUTY)
        duty_left = MOTOR_MAX_PWM_DUTY;

    if (duty_left < MOTOR_MIN_DUTY)
        TB6612_A_Brake();
    else if (left_mm_s >= 0.0f)
        TB6612_A_Forward(duty_left);
    else
        TB6612_A_Backward(duty_left);

    /* ---- 右轮（B 通道，安装方向镜像） ---- */
    float abs_right = (right_mm_s >= 0.0f) ? right_mm_s : -right_mm_s;
    uint32_t duty_right = (uint32_t)(abs_right / MOTOR_MAX_SPEED_MM_S * MOTOR_MAX_PWM_DUTY);
    if (duty_right > MOTOR_MAX_PWM_DUTY)
        duty_right = MOTOR_MAX_PWM_DUTY;

    if (duty_right < MOTOR_MIN_DUTY)
        TB6612_B_Brake();
    else if (right_mm_s >= 0.0f)
        TB6612_B_Backward(duty_right);
    else
        TB6612_B_Forward(duty_right);
}

void Motor_Brake(void)
{
    TB6612_A_Brake();
    TB6612_B_Brake();
}

void Motor_Stop(void)
{
    TB6612_A_Stop();
    TB6612_B_Stop();
}

/**
 * @brief 编码器中断服务函数
 * 
 * 该函数处理两个电机编码器的A相脉冲中断，通过检测B相信号电平判断旋转方向，
 * 并更新对应的脉冲计数器。采用正交解码方式实现电机转速和方向的测量。
 * 
 * @note 电机1和电机2的计数方向定义相反（电机1：B=0时递减，B=1时递增；
 *       电机2：B=0时递增，B=1时递减）
 */
static void encoder_isr(void)
{
    if (DL_GPIO_getEnabledInterruptStatus(MOTOR_ENCODER1_OUT_A_PORT, MOTOR_ENCODER1_OUT_A_PIN))
    {
        if (DL_GPIO_readPins(MOTOR_ENCODER1_OUT_B_PORT, MOTOR_ENCODER1_OUT_B_PIN) == 0)
            g_encoder1_pulse--;
        else
            g_encoder1_pulse++;
        DL_GPIO_clearInterruptStatus(MOTOR_ENCODER1_OUT_A_PORT, MOTOR_ENCODER1_OUT_A_PIN);
    }

    if (DL_GPIO_getEnabledInterruptStatus(MOTOR_ENCODER2_OUT_A_PORT, MOTOR_ENCODER2_OUT_A_PIN))
    {
        if (DL_GPIO_readPins(MOTOR_ENCODER2_OUT_B_PORT, MOTOR_ENCODER2_OUT_B_PIN) == 0)
            g_encoder2_pulse++;
        else
            g_encoder2_pulse--;
        DL_GPIO_clearInterruptStatus(MOTOR_ENCODER2_OUT_A_PORT, MOTOR_ENCODER2_OUT_A_PIN);
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
            encoder_isr();
            break;

        default:
            break;
    }
}

float Motor_GetEncoder1RPM(void)
{
    return g_encoder1_rpm;
}

float Motor_GetEncoder2RPM(void)
{
    return g_encoder2_rpm;
}

float Motor_GetEncoder1Speed(void)
{
    return g_encoder1_speed;
}

float Motor_GetEncoder2Speed(void)
{
    return g_encoder2_speed;
}

int32_t Motor_GetEncoder1Pulse(void)
{
    return g_encoder1_pulse;
}

int32_t Motor_GetEncoder2Pulse(void)
{
    return g_encoder2_pulse;
}

void Motor_TickHandler(void)
{
    int32_t cur1 = g_encoder1_pulse;
    int32_t cur2 = g_encoder2_pulse;

    int32_t diff1 = cur1 - g_enc1_last;
    int32_t diff2 = cur2 - g_enc2_last;

    g_enc1_last = cur1;
    g_enc2_last = cur2;

    /* ---- 累加本 tick 的脉冲差 ---- */
    g_enc1_diff_sum += diff1;
    g_enc2_diff_sum += diff2;
    g_tick_count++;

    /* ---- 窗口期满：计算速度 + EMA 滤波 ---- */
    if (g_tick_count < MOTOR_SPEED_WINDOW_TICKS)
        return;

    // 从窗口内累计脉冲差计算原始 RPM
    // RPM = total_diff / 330 * (60 / (N * 0.02)) = total_diff * 100 / (11 * N)
    float raw_rpm1 = (float)g_enc1_diff_sum * 100.0f / 11.0f
                     / (float)MOTOR_SPEED_WINDOW_TICKS;
    float raw_rpm2 = (float)g_enc2_diff_sum * 100.0f / 11.0f
                     / (float)MOTOR_SPEED_WINDOW_TICKS;

    g_encoder1_rpm   = raw_rpm1;
    g_encoder2_rpm   = raw_rpm2;
    g_encoder1_speed = raw_rpm1 * WHEEL_CIRCUMFERENCE_MM / 60.0f;
    g_encoder2_speed = raw_rpm2 * WHEEL_CIRCUMFERENCE_MM / 60.0f;

    /* ---- EMA 滤波（定点运算，消除 ISR 软浮点） ---- */
    {
        /*
         * GAIN_Q15 = 0.25 × 32768 = 8192
         * new = old + ((raw - old) * GAIN_Q15) >> 15
         */
        #define GAIN_Q15  ((int32_t)(MOTOR_SPEED_EMA_GAIN * 32768.0f))

        int32_t raw_rpm1_x1000  = (int32_t)(raw_rpm1 * 1000.0f);
        int32_t raw_rpm2_x1000  = (int32_t)(raw_rpm2 * 1000.0f);
        int32_t raw_spd1_x1000  = (int32_t)(g_encoder1_speed * 1000.0f);
        int32_t raw_spd2_x1000  = (int32_t)(g_encoder2_speed * 1000.0f);

        if (g_first_window)
        {
            g_encoder1_rpm_f   = raw_rpm1_x1000;
            g_encoder2_rpm_f   = raw_rpm2_x1000;
            g_encoder1_speed_f = raw_spd1_x1000;
            g_encoder2_speed_f = raw_spd2_x1000;
            g_first_window     = false;
        }
        else
        {
            int32_t d;

            d = raw_rpm1_x1000 - g_encoder1_rpm_f;
            g_encoder1_rpm_f += (d * GAIN_Q15) >> 15;

            d = raw_rpm2_x1000 - g_encoder2_rpm_f;
            g_encoder2_rpm_f += (d * GAIN_Q15) >> 15;

            d = raw_spd1_x1000 - g_encoder1_speed_f;
            g_encoder1_speed_f += (d * GAIN_Q15) >> 15;

            d = raw_spd2_x1000 - g_encoder2_speed_f;
            g_encoder2_speed_f += (d * GAIN_Q15) >> 15;
        }

        #undef GAIN_Q15
    }

    /* ---- 重置窗口 ---- */
    g_enc1_diff_sum = 0;
    g_enc2_diff_sum = 0;
    g_tick_count    = 0;
}

void Motor_ResetEncoder(void)
{
    g_encoder1_pulse = 0;
    g_encoder2_pulse = 0;
    g_encoder1_rpm   = 0.0f;
    g_encoder2_rpm   = 0.0f;
    g_encoder1_speed = 0.0f;
    g_encoder2_speed = 0.0f;

    g_encoder1_rpm_f   = 0;
    g_encoder2_rpm_f   = 0;
    g_encoder1_speed_f = 0;
    g_encoder2_speed_f = 0;

    g_enc1_diff_sum  = 0;
    g_enc2_diff_sum  = 0;
    g_enc1_last      = 0;
    g_enc2_last      = 0;
    g_tick_count     = 0;
    g_first_window   = true;
}

/* ========== 滤波值 Getter（定点→float 转换，API 兼容） ========== */

float Motor_GetFilteredSpeed1(void)
{
	return (float)g_encoder1_speed_f * 0.001f;
}

float Motor_GetFilteredSpeed2(void)
{
	return (float)g_encoder2_speed_f * 0.001f;
}

float Motor_GetFilteredRPM1(void)
{
	return (float)g_encoder1_rpm_f * 0.001f;
}

float Motor_GetFilteredRPM2(void)
{
	return (float)g_encoder2_rpm_f * 0.001f;
}
