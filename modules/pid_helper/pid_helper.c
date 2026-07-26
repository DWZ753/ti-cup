/**
 * @file    pid_helper.c
 * @brief   PID Helper 串口协议实现（与 PC 端 PID Helper 软件通信）
 *
 * 下行帧: AA 55 cmd seq arg0 arg1 arg2 checksum（17 字节，arg float32LE）
 * 上行采样: timestamp_ms(u32LE) + 7×f32LE + 00 00 80 7f
 * 上行 ACK: 4×f32LE + 00 00 80 7f
 */

#include "pid_helper.h"
#include "board.h"
#include <string.h>

/* ========== 解析状态机 ========== */

enum {
	PH_PARSE_IDLE,
	PH_PARSE_GOT_AA,
	PH_PARSE_FILLING
};

/* ========== 静态状态 ========== */

static UART_Handle  *s_uart;

/* 当前绑定的轴配置 */
static PH_AxisConfig s_pos;          /**< 位置环 */
static PH_AxisConfig s_vel;          /**< 速度环 */
static PH_AxisConfig *s_active;      /**< 指向当前 appmode 对应的轴 */

static uint8_t s_app_mode;           /**< PH_APPMODE_SPEED 或 POSITION */
static uint8_t s_ctrl_mode;          /**< PH_MODE_CLOSED 或 OPEN */
static float   s_open_output;        /**< 开环输出值 */

static PH_CommandCallback s_callback;

/* 命令帧接收 */
static uint8_t s_rx_buf[PH_CMD_FRAME_SIZE];
static uint8_t s_rx_pos;
static uint8_t s_parse_state;

/* ========== 内部辅助 ========== */

static void write_f32_le(uint8_t *dst, float val)
{
	uint32_t raw;
	(void)memcpy(&raw, &val, sizeof(raw));
	dst[0] = (uint8_t)(raw);
	dst[1] = (uint8_t)(raw >> 8);
	dst[2] = (uint8_t)(raw >> 16);
	dst[3] = (uint8_t)(raw >> 24);
}

static void write_u32_le(uint8_t *dst, uint32_t val)
{
	dst[0] = (uint8_t)(val);
	dst[1] = (uint8_t)(val >> 8);
	dst[2] = (uint8_t)(val >> 16);
	dst[3] = (uint8_t)(val >> 24);
}

static float read_f32_le(const uint8_t *src)
{
	uint32_t raw;
	float    val;

	raw = (uint32_t)src[0]
	    | ((uint32_t)src[1] << 8)
	    | ((uint32_t)src[2] << 16)
	    | ((uint32_t)src[3] << 24);
	(void)memcpy(&val, &raw, sizeof(val));
	return val;
}

static void write_tail(uint8_t *dst)
{
	dst[0] = PH_FRAME_TAIL_0;
	dst[1] = PH_FRAME_TAIL_1;
	dst[2] = PH_FRAME_TAIL_2;
	dst[3] = PH_FRAME_TAIL_3;
}

/** 前 16 字节累加和低 8 位 */
static uint8_t calc_checksum(const uint8_t *frame)
{
	uint16_t sum = 0;
	uint8_t  i;

	for (i = 0; i < PH_CMD_FRAME_CHECKSUM_OFFSET; i++)
	{
		sum += frame[i];
	}
	return (uint8_t)(sum & 0xFF);
}

/* ========== 命令处理 ========== */

static void handle_pid(uint8_t seq, float arg0, float arg1, float arg2)
{
	float kp = arg0;
	float ki = arg1;
	float kd = arg2;

	if (s_active == NULL || s_active->set_pid == NULL) return;

	s_active->set_pid(s_active->ctx, kp, ki, kd);
	PH_SendAck(PH_CMD_PID, PH_STATUS_OK, 0.0f, seq);
}

static void handle_mode(uint8_t seq, float arg0)
{
	if (arg0 >= 0.5f)
	{
		s_ctrl_mode = PH_MODE_OPEN;
	} else
	{
		s_ctrl_mode = PH_MODE_CLOSED;
	}

	PH_SendAck(PH_CMD_MODE, PH_STATUS_OK,
	           (s_ctrl_mode == PH_MODE_OPEN) ? 1.0f : 0.0f, seq);
}

static void handle_out(uint8_t seq, float arg0)
{
	float output = arg0;

	if (output < 0.0f) output = 0.0f;
	if (output > 1.0f) output = 1.0f;

	s_open_output = output;
	PH_SendAck(PH_CMD_OUT, PH_STATUS_OK, s_open_output, seq);
}

static void handle_sp(uint8_t seq, float arg0)
{
	float target = arg0;

	if (s_active == NULL || s_active->set_target == NULL) return;

	/* 位置环：限制角度范围 -180~180 */
	if (s_app_mode == PH_APPMODE_POSITION)
	{
		if (target < -180.0f) target = -180.0f;
		if (target >  180.0f) target =  180.0f;
	}

	s_active->set_target(s_active->ctx, target);

	/* 回显实际设置的目标值 */
	if (s_active->target != NULL)
	{
		PH_SendAck(PH_CMD_SP, PH_STATUS_OK, *s_active->target, seq);
	} else
	{
		PH_SendAck(PH_CMD_SP, PH_STATUS_OK, target, seq);
	}
}

static void handle_appmode(uint8_t seq, float arg0)
{
	if (arg0 >= 0.5f)
	{
		s_app_mode = PH_APPMODE_POSITION;
		s_active   = &s_pos;
	} else
	{
		s_app_mode = PH_APPMODE_SPEED;
		s_active   = &s_vel;
	}

	PH_SendAck(PH_CMD_APPMODE, PH_STATUS_OK,
	           (s_app_mode == PH_APPMODE_POSITION) ? 1.0f : 0.0f, seq);
}

static void dispatch_frame(void)
{
	uint8_t cmd;
	uint8_t seq;
	uint8_t sum;
	float   arg0, arg1, arg2;

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

/* ======================================================================== */
/*  API                                                                      */
/* ======================================================================== */

void PH_Init(UART_Handle *uart,
             const PH_AxisConfig *pos,
             const PH_AxisConfig *vel)
{
	s_uart = uart;

	PH_BindAxis(pos, vel);

	s_app_mode    = PH_APPMODE_POSITION;
	s_active      = &s_pos;
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

void PH_BindAxis(const PH_AxisConfig *pos, const PH_AxisConfig *vel)
{
	if (pos != NULL)
	{
		(void)memcpy(&s_pos, pos, sizeof(PH_AxisConfig));
	}
	if (vel != NULL)
	{
		(void)memcpy(&s_vel, vel, sizeof(PH_AxisConfig));
	}
}

void PH_SendSample(void)
{
	uint8_t  frame[PH_SAMPLE_FRAME_SIZE];
	float    kp = 0.0f, ki = 0.0f, kd = 0.0f;
	float    actual_val = 0.0f;
	float    target_val = 0.0f;
	float    input_val  = 0.0f;
	float    error_val  = 0.0f;
	uint32_t ts;

	if (s_uart == NULL || s_active == NULL) return;

	ts = Board_GetTickMs();

	/* 读取 PID 参数 */
	if (s_active->get_pid != NULL)
	{
		s_active->get_pid(s_active->ctx, &kp, &ki, &kd);
	}

	/* 读取遥测数据 */
	if (s_active->actual != NULL) actual_val = *s_active->actual;
	if (s_active->target != NULL) target_val = *s_active->target;
	if (s_active->input  != NULL) input_val  = *s_active->input;
	if (s_active->error  != NULL) error_val  = *s_active->error;

	/* 打包: timestamp_ms:u32LE + 7×f32LE + tail */
	write_u32_le(&frame[0], ts);
	write_f32_le(&frame[4],  actual_val);
	write_f32_le(&frame[8],  target_val);
	write_f32_le(&frame[12], input_val);
	write_f32_le(&frame[16], error_val);
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

	write_f32_le(&frame[0],  (float)cmd);
	write_f32_le(&frame[4],  (float)status);
	write_f32_le(&frame[8],  value);
	write_f32_le(&frame[12], (float)seq);
	write_tail(&frame[16]);

	UART_SendData(s_uart, frame, PH_ACK_FRAME_SIZE);
}

void PH_Process(void)
{
	uint8_t   byte;
	uint16_t  avail;

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
				s_rx_buf[0]  = byte;
				s_rx_pos     = 1;
				s_parse_state = PH_PARSE_GOT_AA;
			}
			break;

		case PH_PARSE_GOT_AA:
			if (byte == PH_FRAME_HEADER_1)
			{
				s_rx_buf[1]  = byte;
				s_rx_pos     = 2;
				s_parse_state = PH_PARSE_FILLING;
			} else
			{
				if (byte == PH_FRAME_HEADER_0)
				{
					s_rx_buf[0] = byte;
					s_rx_pos    = 1;
				} else
				{
					s_rx_pos     = 0;
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
				s_rx_pos     = 0;
				s_parse_state = PH_PARSE_IDLE;
			}
			break;

		default:
			s_rx_pos     = 0;
			s_parse_state = PH_PARSE_IDLE;
			break;
		}
	}
}
