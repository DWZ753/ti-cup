#include "delay.h"
#include "ti_msp_dl_config.h"

void delay_ms(unsigned long ms)
{
    delay_cycles(ms * (CPUCLK_FREQ / 1000UL));
}
