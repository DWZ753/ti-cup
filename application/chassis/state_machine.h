#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

/* ========== 题目状态 ========== */

typedef enum {
    STATE_IDLE = 0,
    STATE_TASK1,
    STATE_TASK2,
    STATE_TASK3,
    STATE_TASK4
} QuestionState_t;

/* ========== 路径段定义 ========== */

typedef enum {
    SEG_ARC,         // 循迹段（沿黑线弧线走）
    SEG_STRAIGHT,    // 角度保持段（直线/弦线，无黑线）
    SEG_STOP,        // 停车
} SegmentType_t;

typedef struct {
    SegmentType_t type;
    float         delta_deg;     // SEG_STRAIGHT 时的相对偏转角（°）
                                 // 0=直走, >0=左转, <0=右转
} PathSegment_t;

/* ========== 初始化 ========== */

void StateMachine_Init(void);

/* ========== 题目选择 ========== */

QuestionState_t StateMachine_GetState(void);

/**
 * @brief 启动指定题目，加载对应的路径序列表
 */
void StateMachine_StartTask(QuestionState_t task);

/* ========== 路径段查询（主循环每轮调用） ========== */

/**
 * @brief 获取当前段的类型
 */
SegmentType_t StateMachine_GetCurrentSegment(void);

/**
 * @brief 获取当前直线段的相对偏转角（仅在 SEG_STRAIGHT 时有效）
 * @return 相对偏转角（°），0=直走，正值左转，负值右转
 */
float StateMachine_GetDeltaDeg(void);

/* ========== 段推进 ========== */

/**
 * @brief 当前段完成，推进到下一段（自动处理 TASK4 圈数）
 */
void StateMachine_SegmentDone(void);

/**
 * @brief 任务是否已执行完毕
 */
bool StateMachine_IsFinished(void);

/**
 * @brief 获取当前圈数（TASK4 使用）
 */
uint8_t StateMachine_GetLapCount(void);

/* ========== 弦线段 debounce ========== */

/**
 * @brief 弦线段是否仍需先"离开起点黑线"
 */
bool StateMachine_NeedLeaveFirst(void);

/**
 * @brief 通知状态机：已离开起点黑线
 */
void StateMachine_LeftLine(void);

#endif
