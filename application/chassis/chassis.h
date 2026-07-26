#ifndef __CHASSIS_H
#define __CHASSIS_H

/**
 * @brief 底盘控制子系统
 * @note  当前为占位模块，后续填充综合底盘控制逻辑。
 *        包含子模块：tracking（循迹）、angle（角度保持）。
 */

void Chassis_Init(void);
void Chassis_Task(void);

#endif /* __CHASSIS_H */
