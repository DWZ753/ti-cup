#include "pid_helper.h"
#include "pid.h"
#include <string.h>

/* 前向声明：避免循环 include board.h ↔ pid_protocol.h */
extern uint32_t Board_GetTickMs(void);

/* ========== 命令解析状态机 ========== */

enum {
    PH_PARSE_IDLE,
    PH_PARSE_GOT_AA,
    PH_PARSE_FILLING
};

/* ========== 模块内部状态 ========== */

static UART_Handle  *s_uart;
static GimbalAxis   *s_roll;
static GimbalAxis   *s_yaw;
static GimbalAxis   *s_active_axis;
static uint8_t       s_active_index;

static uint8_t       s_app_mode;       /* PH_APPMODE_SPEED 或 PH_APPMODE_POSITION */
static uint8_t       s_ctrl_mode;      /* PH_MODE_CLOSED 或 PH_MODE_OPEN */
static float         s_open_output;    /* 开环输出值 */

static PH_CommandCallback s_callback;

/* 命令帧接收 */
static uint8_t       s_rx_buf[PH_CMD_FRAME_SIZE];
static uint8_t       s_rx_pos;
static uint8_t       s_parse_state;

/* ========== 内部辅助 ========== */

/**
 * @brief 写入 float32 小端到缓冲区
 */
static void write_f32_le(uint8_t *dst, float val)
{
    uint32_t raw;
    memcpy(&raw, &val, sizeof(raw));
    dst[0] = (uint8_t)(raw);
    dst[1] = (uint8_t)(raw >> 8);
    dst[2] = (uint8_t)(raw >> 16);
    dst[3] = (uint8_t)(raw >> 24);
}

/**
 * @brief 写入 uint32 小端到缓冲区
 */
static void write_u32_le(uint8_t *dst, uint32_t val)
{
    dst[0] = (uint8_t)(val);
    dst[1] = (uint8_t)(val >> 8);
    dst[2] = (uint8_t)(val >> 16);
    dst[3] = (uint8_t)(val >> 24);
}

/**
 * @brief 从缓冲区读取 float32 小端
 */
static float read_f32_le(const uint8_t *src)
{
    uint32_t raw;
    float val;
    raw = (uint32_t)src[0]
        | ((uint32_t)src[1] << 8)
        | ((uint32_t)src[2] << 16)
        | ((uint32_t)src[3] << 24);
    memcpy(&val, &raw, sizeof(val));
    return val;
}

/**
 * @brief 写帧尾 00 00 80 7f
 */
static void write_tail(uint8_t *dst)
{
    dst[0] = PH_FRAME_TAIL_0;
    dst[1] = PH_FRAME_TAIL_1;
    dst[2] = PH_FRAME_TAIL_2;
    dst[3] = PH_FRAME_TAIL_3;
}

/**
 * @brief 计算命令帧校验和（前 16 字节逐字节累加，取低 8 位）
 */
static uint8_t calc_checksum(const uint8_t *frame)
{
    uint16_t sum = 0;
    uint8_t i;
    for (i = 0; i < PH_CMD_FRAME_CHECKSUM_OFFSET; i++)
    {
        sum += frame[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/* GimbalAxis_GetPosPID / GimbalAxis_GetVelPID 在 gimbal.h 中声明 */

/* ========== 命令处理 ========== */

static void handle_pid(uint8_t seq, float arg0, float arg1, float arg2)
{
    float kp = arg0;
    float ki = arg1;
    float kd = arg2;

    if (s_active_axis == NULL) return;

    if (s_app_mode == PH_APPMODE_POSITION)
    {
        GimbalAxis_TunePosPID(s_active_axis, kp, ki, kd);
    } else
    {
        GimbalAxis_TuneVelPID(s_active_axis, kp, ki, kd);
    }

    PH_SendAck(PH_CMD_PID, PH_STATUS_OK, 0.0f, seq);
}

static void handle_mode(uint8_t seq, float arg0)
{
    uint8_t new_mode;

    if (arg0 >= 0.5f)
    {
        new_mode = PH_MODE_OPEN;
    } else
    {
        new_mode = PH_MODE_CLOSED;
    }

    s_ctrl_mode = new_mode;

    PH_SendAck(PH_CMD_MODE, PH_STATUS_OK,
               (new_mode == PH_MODE_OPEN) ? 1.0f : 0.0f, seq);
}

static void handle_out(uint8_t seq, float arg0)
{
    float output = arg0;

    if (output < 0.0f) output = 0.0f;
    if (output > 1.0f) output = 1.0f;

    s_open_output = output;

    PH_SendAck(PH_CMD_OUT, PH_STATUS_OK,
               s_open_output, seq);
}

static void handle_sp(uint8_t seq, float arg0)
{
    if (s_active_axis == NULL) return;

    if (s_app_mode == PH_APPMODE_POSITION)
    {
        GimbalAxis_SetTarget(s_active_axis, arg0);
        PH_SendAck(PH_CMD_SP, PH_STATUS_OK,
                   s_active_axis->target_angle, seq);
    } else
    {
        /* 速度环模式：SP 直接作为速度目标 */
        PH_SendAck(PH_CMD_SP, PH_STATUS_OK, arg0, seq);
    }
}

static void handle_appmode(uint8_t seq, float arg0)
{
    if (arg0 >= 0.5f)
    {
        s_app_mode = PH_APPMODE_POSITION;
    } else
    {
        s_app_mode = PH_APPMODE_SPEED;
    }

    PH_SendAck(PH_CMD_APPMODE, PH_STATUS_OK,
               (s_app_mode == PH_APPMODE_POSITION) ? 1.0f : 0.0f, seq);
}

/**
 * @brief 验证并分发收到的命令帧
 */
static void dispatch_frame(void)
{
    uint8_t cmd;
    uint8_t seq;
    uint8_t sum;
    float arg0, arg1, arg2;

    /* 校验和 */
    sum = calc_checksum(s_rx_buf);
    if (sum != s_rx_buf[PH_CMD_FRAME_CHECKSUM_OFFSET])
    {
        return;
    }

    cmd = s_rx_buf[2];
    seq = s_rx_buf[3];

    arg0 = read_f32_le(&s_rx_buf[4]);
    arg1 = read_f32_le(&s_rx_buf[8]);
    arg2 = read_f32_le(&s_rx_buf[12]);

    /* 若注册了外部回调，先通知 */
    if (s_callback != NULL)
    {
        s_callback(cmd, seq, arg0, arg1, arg2);
    }

    switch (cmd)
    {
        case PH_CMD_PID:
            handle_pid(seq, arg0, arg1, arg2);
            break;
        case PH_CMD_MODE:
            handle_mode(seq, arg0);
            break;
        case PH_CMD_OUT:
            handle_out(seq, arg0);
            break;
        case PH_CMD_SP:
            handle_sp(seq, arg0);
            break;
        case PH_CMD_APPMODE:
            handle_appmode(seq, arg0);
            break;
        default:
            break;
    }
}

/* ========================================================================== */
/*  API                                                                       */
/* ========================================================================== */

void PH_Init(UART_Handle *uart, GimbalAxis *roll, GimbalAxis *yaw)
{
    s_uart  = uart;
    s_roll  = roll;
    s_yaw   = yaw;

    s_active_index = PH_DEFAULT_AXIS;
    if (s_active_index == PH_DEFAULT_AXIS_ROLL)
    {
        s_active_axis = roll;
    } else
    {
        s_active_axis = yaw;
    }

    s_app_mode    = PH_APPMODE_POSITION;
    s_ctrl_mode   = PH_MODE_CLOSED;
    s_open_output = 0.0f;
    s_callback    = NULL;

    s_rx_pos      = 0;
    s_parse_state = PH_PARSE_IDLE;

    if (s_uart != NULL)
    {
        UART_StartReceiveRaw(s_uart);
    }
}

void PH_SetCallback(PH_CommandCallback cb)
{
    s_callback = cb;
}

void PH_SendSample(void)
{
    uint8_t frame[PH_SAMPLE_FRAME_SIZE];
    float kp, ki, kd;
    float actual, target, input, error;
    uint32_t ts;

    if (s_uart == NULL || s_active_axis == NULL) return;

    ts = Board_GetTickMs();

    /* 根据当前模式选择数据源 */
    if (s_app_mode == PH_APPMODE_POSITION)
    {
        /* 位置环：target=角度, actual=当前角度, input=速度指令(位置PID输出), error=位置误差 */
        GimbalAxis_GetPosPID(s_active_axis, &kp, &ki, &kd);
        target = s_active_axis->target_angle;
        actual = s_active_axis->current_angle;
        input  = s_active_axis->vel_cmd;
        error  = s_active_axis->pos_error;
    } else
    {
        /* 速度环：target=速度指令, actual=当前速度, input=电机输出(速度PID输出), error=速度误差 */
        GimbalAxis_GetVelPID(s_active_axis, &kp, &ki, &kd);
        target = s_active_axis->vel_cmd;
        actual = s_active_axis->current_vel;
        input  = s_active_axis->motor_output;
        error  = s_active_axis->vel_error;
    }

    /* 打包: [timestamp_ms:u32LE] + 7×[f32LE] + tail */
    write_u32_le(&frame[0], ts);
    write_f32_le(&frame[4],  actual);
    write_f32_le(&frame[8],  target);
    write_f32_le(&frame[12], input);
    write_f32_le(&frame[16], error);
    write_f32_le(&frame[20], kp);
    write_f32_le(&frame[24], ki);
    write_f32_le(&frame[28], kd);
    write_tail(&frame[32]);

    UART_SendData(s_uart, frame, PH_SAMPLE_FRAME_SIZE);
}

void PH_SendAck(uint8_t cmd, uint8_t status, float value, uint8_t seq)
{
    uint8_t frame[PH_ACK_FRAME_SIZE];

    if (s_uart == NULL) return;

    /* 打包: [commandId:f32LE] [statusCode:f32LE] [value:f32LE] [seq:f32LE] + tail */
    write_f32_le(&frame[0],  (float)cmd);
    write_f32_le(&frame[4],  (float)status);
    write_f32_le(&frame[8],  value);
    write_f32_le(&frame[12], (float)seq);
    write_tail(&frame[16]);

    UART_SendData(s_uart, frame, PH_ACK_FRAME_SIZE);
}

void PH_Process(void)
{
    uint8_t byte;
    uint16_t avail;

    if (s_uart == NULL) return;

    avail = UART_RawRxAvailable(s_uart);

    while (avail > 0)
    {
        byte = UART_ReadRawByte(s_uart);
        --avail;

        switch (s_parse_state)
        {
            case PH_PARSE_IDLE:
                if (byte == PH_FRAME_HEADER_0)
                {
                    s_rx_buf[0] = byte;
                    s_rx_pos    = 1;
                    s_parse_state = PH_PARSE_GOT_AA;
                }
                break;

            case PH_PARSE_GOT_AA:
                if (byte == PH_FRAME_HEADER_1)
                {
                    s_rx_buf[1] = byte;
                    s_rx_pos    = 2;
                    s_parse_state = PH_PARSE_FILLING;
                } else
                {
                    /* 不是 0x55，重新检查是否为新帧头 */
                    if (byte == PH_FRAME_HEADER_0)
                    {
                        s_rx_buf[0] = byte;
                        s_rx_pos    = 1;
                        /* 保持 GOT_AA 状态 */
                    } else
                    {
                        s_rx_pos    = 0;
                        s_parse_state = PH_PARSE_IDLE;
                    }
                }
                break;

            case PH_PARSE_FILLING:
                s_rx_buf[s_rx_pos] = byte;
                ++s_rx_pos;

                if (s_rx_pos >= PH_CMD_FRAME_SIZE)
                {
                    dispatch_frame();
                    s_rx_pos    = 0;
                    s_parse_state = PH_PARSE_IDLE;
                }
                break;

            default:
                s_rx_pos    = 0;
                s_parse_state = PH_PARSE_IDLE;
                break;
        }
    }
}

void PH_SetActiveAxis(uint8_t axis_index)
{
    s_active_index = axis_index;
    if (axis_index == PH_DEFAULT_AXIS_ROLL)
    {
        s_active_axis = s_roll;
    } else
    {
        s_active_axis = s_yaw;
    }
}
