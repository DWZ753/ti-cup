/**
 * @file main.c
 * @brief 舵机来回摆动测试
 *
 * 控制舵机在 -100（最左）到 +100（最右）之间来回扫描，
 * OLED 实时显示当前角度值。
 */
#include "ti_msp_dl_config.h"
#include "board.h"

#define SWEEP_STEP   5    /* 每步变化量 */
#define SWEEP_DELAY  20   /* 每步延时 ms，控制摆动速度 */

int main(void)
{
	int32_t value;
	int8_t  direction;

	SYSCFG_DL_init();
	Board_Init();

	/* OLED 显示标题 */
	OLED_ShowString(0, 0, (uint8_t *)"Servo Test", 16);
	OLED_ShowString(0, 2, (uint8_t *)"Pos:", 16);

	value     = 0;
	direction = 1;   /* 1 = 正向增大, -1 = 反向减小 */

	while (1)
	{
		// /* 更新舵机位置 */
		// Servo_SetValue(value);

		// /* OLED 显示当前值 */
		// OLED_ShowNum(48, 2, (uint32_t)(value >= 0 ? value : -value), 3, 16);
		// OLED_ShowString(72, 2, (uint8_t *)(value >= 0 ? "   " : "-  "), 16);

		// /* 绘制简易进度条 (第 4 行, 128px 宽 → 映射 -100~100) */
		// {
		// 	uint8_t bar_x = (uint8_t)((value + 100) * 120 / 200);  /* 0~120 */
		// 	uint8_t i;
		// 	OLED_ShowString(0, 4, (uint8_t *)"[              ]", 16);
		// 	for (i = 1; i <= bar_x; i++)
		// 	{
		// 		OLED_ShowChar(i * 6, 4, '=', 16);
		// 	}
		// }

		// /* 步进 */
		// value += direction * SWEEP_STEP;

		// /* 到达边界反转方向 */
		// if (value >= 100)
		// {
		// 	value     = 100;
		// 	direction = -1;
		// }
		// else if (value <= -100)
		// {
		// 	value     = -100;
		// 	direction = 1;
		// }

		// delay_ms(SWEEP_DELAY);
	}
}
