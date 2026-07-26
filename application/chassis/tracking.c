#include "tracking.h"

float Tracking_CalcPosition(uint8_t mask)
{
    uint8_t bits = ~mask;   // 取反后 1 = 黑线
    float sum_weight = 0;
    float count  = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        if (bits & (1 << i))
        {
            sum_weight += (float)i;
            count  += 1.0f;
        }
    }

    if (count == 0)
        return 99.0f;

    float center = sum_weight / count;
    return (center - SENSOR_CENTER) / SENSOR_CENTER;
}