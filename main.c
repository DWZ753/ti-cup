#include "ti_msp_dl_config.h"
#include "board.h"
#include "oled.h"
#include "imu.h"
#include "delay.h"
#include "stdio.h"

/**
 * @brief OLED 显示一个有符号浮点数（1 位小数）
 * @note  避免 snprintf %f 在裸机上可能的链接问题
 */
static void oled_show_float(uint8_t x, uint8_t y, float v)
{
	int8_t  int_part;
	uint8_t frac;
	char    buf[8];
	int     len;

	if (v >= 0.0f) {
		int_part = (int8_t)v;
		frac     = (uint8_t)((v - (float)int_part) * 10.0f + 0.5f);
		if (frac >= 10) { int_part++; frac = 0; }
		len = snprintf(buf, sizeof(buf), "%d.%01u", int_part, frac);
	} else {
		v        = -v;
		int_part = (int8_t)v;
		frac     = (uint8_t)((v - (float)int_part) * 10.0f + 0.5f);
		if (frac >= 10) { int_part++; frac = 0; }
		len = snprintf(buf, sizeof(buf), "-%d.%01u", int_part, frac);
	}
	if (len > 0 && len < (int)sizeof(buf)) {
		OLED_ShowString(x, y, (uint8_t *)buf, 8);
	}
}

int main(void)
{
	float    roll, pitch, yaw;
	float    accel[3], gyro[3];
	char     buf[16];
	uint8_t  x_pos;
	uint32_t last_imu;

	SYSCFG_DL_init();
	Board_Init();

	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t *)"IMU660RA Test", 8);
	OLED_ShowString(0, 1, (uint8_t *)"Ready", 8);

	last_imu = Board_GetTickMs();

	while (1)
	{
		if (Board_GetTickMs() - last_imu >= 10)
		{
			last_imu = Board_GetTickMs();

			IMU_Update();
			IMU_GetEuler(&roll, &pitch, &yaw);
			IMU_GetAccel(accel);
			IMU_GetGyro(gyro);

			/* 行 3：欧拉角 */
			x_pos = 0;
			snprintf(buf, sizeof(buf), "R");
			OLED_ShowString(x_pos, 3, (uint8_t *)buf, 8);
			x_pos += 6;
			oled_show_float(x_pos, 3, roll);
			x_pos += 38;
			snprintf(buf, sizeof(buf), "P");
			OLED_ShowString(x_pos, 3, (uint8_t *)buf, 8);
			x_pos += 6;
			oled_show_float(x_pos, 3, pitch);
			x_pos += 38;
			snprintf(buf, sizeof(buf), "Y");
			OLED_ShowString(x_pos, 3, (uint8_t *)buf, 8);
			x_pos += 6;
			oled_show_float(x_pos, 3, yaw);

			/* 行 4-5：加速度 [x y z] */
			x_pos = 0;
			snprintf(buf, sizeof(buf), "A:");
			OLED_ShowString(x_pos, 4, (uint8_t *)buf, 8);
			x_pos += 12;
			oled_show_float(x_pos, 4, accel[0]);
			x_pos += 38;
			oled_show_float(x_pos, 4, accel[1]);
			x_pos += 38;
			oled_show_float(x_pos, 4, accel[2]);

			/* 行 6-7：角速度 [x y z] */
			x_pos = 0;
			snprintf(buf, sizeof(buf), "G:");
			OLED_ShowString(x_pos, 6, (uint8_t *)buf, 8);
			x_pos += 12;
			oled_show_float(x_pos, 6, gyro[0]);
			x_pos += 38;
			oled_show_float(x_pos, 6, gyro[1]);
			x_pos += 38;
			oled_show_float(x_pos, 6, gyro[2]);
		}
	}
}
