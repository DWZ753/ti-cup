#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "delay.h"
#include "imu.h"
#include "oled.h"

void Board_Init(void);
uint32_t Board_GetTickMs(void);

#endif
