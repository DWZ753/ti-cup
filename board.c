#include "board.h"
#include "ti_msp_dl_config.h"
#include "pit_fast_tick.h"
#include "pit_control_tick.h"
#include "buzzer.h"
#include "tb6612.h"
#include "motor.h"
#include "key.h"
#include "grayscale.h"
#include "servo.h"
#include "imu.h"
#include "oled.h"
#include "ultrasonic.h"

/* ========== 静态变量 ========== */

static volatile uint32_t imu_ticks;
static UART_Handle      *uart_print;
static UART_Handle      *uart_pi;

/* ========== 内部回调 ========== */

static void imu_tick_cb(void)
{
    imu_ticks++;
}

/* ========== 公共 API ========== */

void Board_Init(void)
{
    // 定时器
    PIT_Fast_Tick_Init();
    PIT_Fast_Tick_RegisterCallback(imu_tick_cb);
    PIT_Control_Tick_Init();

    // 执行器
    Buzzer_Init();
    TB6612_Init();
    Motor_Init();
    Servo_Init();

    // 输入
    Key_Init();
    Grayscale_Init();

    /* 通信 */
    UART_Config uart_cfg = {
        .uart         = UART_PRINT_INST,
        .irqNum       = UART_PRINT_INT_IRQN,
        .dmaTxChanId  = UART0_DMA_TX_CHAN_ID,
        .dmaTxTrigger = PRINT_INST_DMA_TRIGGER,
    };
    uart_print = UART_Init(&uart_cfg);

    /* 树莓派通信 UART (UART1) */
    UART_Config uart_pi_cfg = {
        .uart         = UART_PI_INST,
        .irqNum       = UART_PI_INST_INT_IRQN,
        .dmaTxChanId  = 0,
        .dmaTxTrigger = 0,
    };
    uart_pi = UART_Init(&uart_pi_cfg);

    // 传感器
    IMU_Init();

    // 超声波
    Ultrasonic_Init();

    // 显示
    OLED_Init();
}

uint32_t Board_GetTickMs(void)
{
    return imu_ticks;
}

UART_Handle* Board_GetUART(void)
{
    return uart_print;
}

UART_Handle* Board_GetUART_PI(void)
{
    return uart_pi;
}
