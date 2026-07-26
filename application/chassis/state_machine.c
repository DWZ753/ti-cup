#include "state_machine.h"
#include "ti_msp_dl_config.h"
#include "key.h"

/* ========== 路径序列表（相对偏转角，上赛道实测后填入） ========== */
// delta_deg: 进入直线段时，以当前 yaw 为基准偏转的角度
//   0 = 保持当前朝向直走
//   正值 = 左转，负值 = 右转
//   SEG_ARC 的 delta_deg 无意义（沿黑线循迹）

// TASK1: A → B（左边直线）
static const PathSegment_t path_task1[] = {
    { SEG_STRAIGHT,   0.0f },   // 保持当前朝向，直走到底
};

// TASK2: A → C(上半圆弧) → D(右边直线) → B(下半圆弧) → A(左边直线)
static const PathSegment_t path_task2[] = {
    { SEG_ARC,        0.0f },   // AC 上半圆弧
    { SEG_STRAIGHT, -24.0f },   // CD: 出弧后右转走直（偏转值实测）
    { SEG_ARC,        0.0f },   // DB 下半圆弧
    { SEG_STRAIGHT, -26.0f },   // BA: 出弧后右转走直（偏转值实测）
};

// TASK3: A → C(弦直线) → B(弧) → D(弦直线) → A(弧)
static const PathSegment_t path_task3[] = {
    { SEG_STRAIGHT,   -30.0f },   // AC 弦: 保持当前朝向直走
    { SEG_ARC,        0.0f },   // 弧线段
    { SEG_STRAIGHT,   -30.0f },   // 弦直线: 偏转值实测
    { SEG_ARC,        0.0f },   // 弧线段
};

/* ========== 内部状态 ========== */

static QuestionState_t s_current_state = STATE_IDLE;

static const PathSegment_t *s_path;
static uint8_t              s_seg_count;
static uint8_t              s_seg_index;

static uint8_t              s_lap_count;
static bool                 s_finished;
static bool                 s_waiting_leave;

/* ========== 按键回调 ========== */

static void Key_Handler(uint8_t key_index)
{
    s_current_state = (QuestionState_t)(key_index + 1);
    DL_GPIO_togglePins(GPIO_LEDs_PORT, GPIO_LEDs_GPIO_LED_PIN);
}

/* ========== 公共 API ========== */

void StateMachine_Init(void)
{
    Key_RegisterCallback(Key_Handler);
}

QuestionState_t StateMachine_GetState(void)
{
    return s_current_state;
}

void StateMachine_StartTask(QuestionState_t task)
{
    s_finished  = false;
    s_lap_count = 1;
    s_seg_index = 0;
    s_waiting_leave = false;

    switch (task)
    {
        case STATE_TASK1:
            s_path      = path_task1;
            s_seg_count = sizeof(path_task1) / sizeof(path_task1[0]);
            break;
        case STATE_TASK2:
            s_path      = path_task2;
            s_seg_count = sizeof(path_task2) / sizeof(path_task2[0]);
            break;
        case STATE_TASK3:
            s_path      = path_task3;
            s_seg_count = sizeof(path_task3) / sizeof(path_task3[0]);
            break;
        case STATE_TASK4:
            s_path      = path_task3;
            s_seg_count = sizeof(path_task3) / sizeof(path_task3[0]);
            break;
        default:
            s_path      = NULL;
            s_seg_count = 0;
            break;
    }

    if (s_path != NULL && s_seg_count > 0
        && s_path[0].type == SEG_STRAIGHT)
    {
        s_waiting_leave = true;
    }
}

SegmentType_t StateMachine_GetCurrentSegment(void)
{
    if (s_finished || s_path == NULL || s_seg_index >= s_seg_count)
        return SEG_STOP;

    return s_path[s_seg_index].type;
}

float StateMachine_GetDeltaDeg(void)
{
    if (s_path != NULL && s_seg_index < s_seg_count
        && s_path[s_seg_index].type == SEG_STRAIGHT)
    {
        return s_path[s_seg_index].delta_deg;
    }
    return 0.0f;
}

void StateMachine_SegmentDone(void)
{
    s_seg_index++;
    s_waiting_leave = false;
    if (s_seg_index >= s_seg_count)
    {
        if (s_current_state == STATE_TASK4 && s_lap_count < 4)
        {
            s_lap_count++;
            s_seg_index = 0;
        }
        else
        {
            s_finished = true;
        }
    }
}

bool StateMachine_IsFinished(void)
{
    return s_finished;
}

uint8_t StateMachine_GetLapCount(void)
{
    return s_lap_count;
}

bool StateMachine_NeedLeaveFirst(void)
{
    return s_waiting_leave;
}

void StateMachine_LeftLine(void)
{
    s_waiting_leave = false;
}
