/**
 * @file    menu.h
 * @brief   OLED 菜单与状态显示模块
 *
 * 将 main.c 中的显示逻辑抽离至此，main.c 仅通过 API 驱动。
 */

#ifndef __MENU_H__
#define __MENU_H__

#include <stdint.h>
#include <stdbool.h>

/* ========== 共享枚举（main.c + menu.c 共用） ========== */

/** 应用状态 */
typedef enum {
	STATE_MENU,          /**< 选择题目 */
	STATE_READY,         /**< 已确认，等待启动 */
	STATE_RUNNING,       /**< 运行中 */
} UI_State;

/** 题目模式 */
typedef enum {
	TASK_TRACK_ONLY     = 0,  /**< 要求2：纯循迹一圈 */
	TASK_BAL_STATIC,          /**< 要求3：静态平衡 */
	TASK_TRACK_BAL_AB,        /**< 要求4：循迹+平衡 AB 段 */
	TASK_TRACK_BAL_LAP_O,     /**< 要求5：循迹+平衡一圈 O 点 */
	TASK_TRACK_BAL_LAP_X,     /**< 要求6：循迹+平衡任意位置 */
	TASK_REMOTE,              /**< 要求7：Pi 遥控 */
	TASK_COUNT
} UI_TaskMode;

/** 任务名查找表（menu.c 内部定义） */
extern uint8_t *g_task_names[TASK_COUNT];

/* ========== 调试数据（main.c 写入，menu.c 读取显示） ========== */

typedef struct {
	float   ff_accel;      /**< 滤波后底盘加速度 (m/s²)  */
	float   ff_angle;      /**< FF 输出倾角 (°)         */
	float   pid_p;         /**< PD P 项 (°)             */
	float   pid_d;         /**< PD D 项 (°)             */
	float   pid_i;         /**< PD I 项累积 (°)         */
	float   target_mm;     /**< 球目标位置 (mm)         */
	int16_t pi_ball_pos;   /**< Pi 发来的球位置 (mm)    */
} UI_DebugValues;

/* ========== API ========== */

/**
 * @brief 初始化 OLED 显示
 */
void UI_Init(void);

/**
 * @brief 每主循环调用一次，根据状态刷新 OLED
 * @param state        当前应用状态
 * @param task         当前选中/运行中的任务
 * @param elapsed_ms   运行中已过时间（STATE_RUNNING），或 0
 * @param last_time_ms 上次运行总耗时（STATE_READY 显示用），或 0
 * @param dbg          调试数据（可为 NULL）
 */
void UI_Refresh(UI_State state, UI_TaskMode task,
                uint32_t elapsed_ms, uint32_t last_time_ms,
                const UI_DebugValues *dbg);

/**
 * @brief 标记显示需重绘（如任务切换时调用）
 */
void UI_SetDirty(void);

#endif /* __MENU_H__ */
