#include "grayscale.h"
#include "ti_msp_dl_config.h"

static const uint32_t grayscale_pins[GRAYSCALE_NUM] = {
    GPIO_GRAYSCALEs_GPIO_GRAYSCALE1_PIN,
    GPIO_GRAYSCALEs_GPIO_GRAYSCALE2_PIN,
    GPIO_GRAYSCALEs_GPIO_GRAYSCALE3_PIN,
    GPIO_GRAYSCALEs_GPIO_GRAYSCALE4_PIN,
    GPIO_GRAYSCALEs_GPIO_GRAYSCALE5_PIN,
    GPIO_GRAYSCALEs_GPIO_GRAYSCALE6_PIN,
    GPIO_GRAYSCALEs_GPIO_GRAYSCALE7_PIN,
    GPIO_GRAYSCALEs_GPIO_GRAYSCALE8_PIN,
};

void Grayscale_Init(void)
{
}

uint8_t Grayscale_ReadAll(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < GRAYSCALE_NUM; i++)
    {
        if (DL_GPIO_readPins(GRAYSCALE_PORT, grayscale_pins[i]))
            mask |= (1 << i);
    }
    return mask;
}
