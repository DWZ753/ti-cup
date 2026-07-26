#ifndef __DEEWAZ_MATH_H
#define __DEEWAZ_MATH_H

#include <math.h>

/** 三维浮点数向量 */
typedef struct {
    float x;
    float y;
    float z;
} Axis3f;

/** 角度与弧度转换常量 */
#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f

/** 快速平方根倒数（向量归一化用） */
static inline float invSqrt(float x)
{
    if (x <= 0.0f) return 1.0f;
    return 1.0f / sqrtf(x);
}

#endif
