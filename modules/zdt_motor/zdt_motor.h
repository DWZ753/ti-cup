#ifndef __ZDT_MOTOR_H
#define __ZDT_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

/**********************************************************
*** ZDT 闭环步进电机模块
*** 基于张大头 Emm_V5.0 协议（RS485/UART 通信）
***
*** 依赖 bsp/uart 提供 UART 句柄（自注册模式）。
*** UART ISR 由 bsp/uart 统一处理，本模块不接管中断。
***
*** 协议格式：[地址] [命令] [数据...] [0x6B 校验]
*** 默认波特率：115200-8-N-1
*** 默认电机 ID：1（广播地址：0）
**********************************************************/

/* ========== 硬件映射宏（SysConfig 重新生成后只需修改此处） ========== */

#define ZDT_MOTOR_UART_INST           UART_ZDT_MOTOR_INST
#define ZDT_MOTOR_UART_INT_IRQN       UART_ZDT_MOTOR_INST_INT_IRQN

/* ========== 常量 ========== */

#define ZDT_MOTOR_RX_BUF_SIZE         128
#define ZDT_MOTOR_MMCL_LEN            512

#define ABS(x)                        ((x) > 0 ? (x) : -(x))

/* ========== 系统参数选择器 ========== */

typedef enum {
    ZDT_PARAM_VBUS  = 5,   // 读取总线电压
    ZDT_PARAM_CBUS  = 6,   // 读取总线电流
    ZDT_PARAM_CPHA  = 7,   // 读取相电流
    ZDT_PARAM_ENCO  = 8,   // 读取编码器原始值
    ZDT_PARAM_CLKC  = 9,   // 读取实时脉冲数
    ZDT_PARAM_ENCL  = 10,  // 读取经过线性校准的编码器值
    ZDT_PARAM_CLKI  = 11,  // 读取输入脉冲数
    ZDT_PARAM_TPOS  = 12,  // 读取电机目标位置
    ZDT_PARAM_SPOS  = 13,  // 读取电机实时设定的目标位置
    ZDT_PARAM_VEL   = 14,  // 读取电机实时转速
    ZDT_PARAM_CPOS  = 15,  // 读取电机实时位置
    ZDT_PARAM_PERR  = 16,  // 读取电机位置误差
    ZDT_PARAM_VBAT  = 17,  // 读取线圈/电池电压（Y42）
    ZDT_PARAM_TEMP  = 18,  // 读取电机实时温度（Y42）
    ZDT_PARAM_FLAG  = 19,  // 读取电机状态标志位
    ZDT_PARAM_OFLAG = 20,  // 读取原点状态标志位
    ZDT_PARAM_OAF   = 21,  // 读取电机状态标志位 + 原点状态标志位（Y42）
    ZDT_PARAM_PIN   = 22,  // 读取引脚状态（Y42）
} ZDT_Motor_Param_t;

/* ========== 外部全局变量 ========== */

extern volatile uint8_t  zdt_motor_rx_cmd[ZDT_MOTOR_RX_BUF_SIZE];
extern volatile uint8_t  zdt_motor_rx_count;
extern volatile uint16_t zdt_motor_mmcl_count;
extern volatile uint8_t  zdt_motor_mmcl_cmd[ZDT_MOTOR_MMCL_LEN];

/* ========== 初始化 ========== */

/**
 * @brief    ZDT 电机模块初始化
 * @note     通过 UART_Init() 自注册 UART 实例并进入原始接收模式。
 *           必须在 SYSCFG_DL_init() 之后调用。
 *           内部等待 500ms 让电机驱动板启动。
 */
void ZDT_Motor_Init(void);

/* ========== 底层通信 ========== */

/**
 * @brief    从 UART 原始缓冲区取出接收到的指令到 rx_cmd[]
 * @note     在发送读指令之后、解析响应之前调用
 */
void ZDT_Motor_GetRxCmd(void);

/* ======================================================================== */
/*  系统命令                                                               */
/* ======================================================================== */

void ZDT_Motor_Trig_Encoder_Cal(uint8_t addr);
void ZDT_Motor_Reset_Motor(uint8_t addr);
void ZDT_Motor_Reset_CurPos_To_Zero(uint8_t addr);
void ZDT_Motor_Reset_Clog_Pro(uint8_t addr);
void ZDT_Motor_Restore_Motor(uint8_t addr);

/* ======================================================================== */
/*  运动控制命令                                                           */
/* ======================================================================== */

void ZDT_Motor_Multi_Motor_Cmd(uint8_t addr);

/**
 * @brief    电机使能控制
 * @param    addr   电机地址
 * @param    state  true=使能电机，false=关闭电机
 * @param    snF    同步标志（false=立即执行，true=等待同步触发）
 */
void ZDT_Motor_En_Control(uint8_t addr, bool state, bool snF);

/**
 * @brief    速度模式控制（连续旋转）
 * @param    addr  电机地址
 * @param    dir   方向（0=CW，非0=CCW）
 * @param    vel   速度，范围 0-5000 RPM
 * @param    acc   加速度，范围 0-255（0=直冲，无加减速）
 * @param    snF   同步标志
 */
void ZDT_Motor_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                           uint8_t acc, bool snF);

/**
 * @brief    位置模式控制（移动 N 个脉冲）
 * @param    addr  电机地址
 * @param    dir   方向（0=CW，非0=CCW）
 * @param    vel   速度，范围 0-5000 RPM
 * @param    acc   加速度，范围 0-255（0=直冲，无加减速）
 * @param    clk   脉冲数，范围 0-2^32-1。16 细分下 3200=1 圈
 * @param    raF   运动参考系：0=相对上次目标位置做相对运动，
 *                 1=绝对位置运动，2=相对当前实时位置做相对运动
 * @param    snF   同步标志
 */
void ZDT_Motor_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                           uint8_t acc, uint32_t clk, uint8_t raF, bool snF);

/**
 * @brief    预设快速位置模式的运动参数
 * @note     调用一次后，可反复使用 ZDT_Motor_QPos_Control() 发送位移量
 */
void ZDT_Motor_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc,
                               uint8_t raF, bool snF);

/**
 * @brief    快速位置模式控制（带符号脉冲数）
 * @param    clk  带符号脉冲数。+N=CW 方向 N 个脉冲，-N=CCW 方向 N 个脉冲。
 *                16 细分下 ±3200=±1 圈
 * @note     需先调用 ZDT_Motor_Set_QPos_Params() 预设参数
 */
void ZDT_Motor_QPos_Control(uint8_t addr, int32_t clk);

/**
 * @brief    立即停止
 * @param    snF  同步标志
 */
void ZDT_Motor_Stop_Now(uint8_t addr, bool snF);

/**
 * @brief    触发所有 snF=1 的电机同步运动
 * @param    addr  通常为 0（广播地址）
 */
void ZDT_Motor_Synchronous_motion(uint8_t addr);

/* ======================================================================== */
/*  原点/回零命令                                                          */
/* ======================================================================== */

void ZDT_Motor_Origin_Set_O(uint8_t addr, bool svF);

/**
 * @brief    触发回零
 * @param    o_mode  回零模式：0=近限位回零，1=近限位缩回，
 *                   2=硬停回零，3=硬停缩回
 */
void ZDT_Motor_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);

void ZDT_Motor_Origin_Interrupt(uint8_t addr);
void ZDT_Motor_Origin_Read_Params(uint8_t addr);

void ZDT_Motor_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode,
    uint8_t o_dir, uint16_t o_vel, uint32_t o_tm,
    uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);

void ZDT_Motor_Origin_Read_SL_RP(uint8_t addr);
void ZDT_Motor_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp);

/* ======================================================================== */
/*  读取系统参数                                                           */
/* ======================================================================== */

void ZDT_Motor_Auto_Return_Sys_Params_Timed(uint8_t addr, ZDT_Motor_Param_t s,
                                            uint16_t time_ms);
void ZDT_Motor_Read_Sys_Params(uint8_t addr, ZDT_Motor_Param_t s);

/* ======================================================================== */
/*  配置命令                                                               */
/* ======================================================================== */

void ZDT_Motor_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id);
void ZDT_Motor_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep);
void ZDT_Motor_Modify_PDFlag(uint8_t addr, bool pdf);
void ZDT_Motor_Read_Opt_Param_Sta(uint8_t addr);
void ZDT_Motor_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype);
void ZDT_Motor_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype);
void ZDT_Motor_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode);
void ZDT_Motor_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir);
void ZDT_Motor_Modify_Lock_Btn(uint8_t addr, bool svF, bool lock);
void ZDT_Motor_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel);
void ZDT_Motor_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma);
void ZDT_Motor_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA);
void ZDT_Motor_Read_PID_Params(uint8_t addr);

void ZDT_Motor_Modify_PID_Params(uint8_t addr, bool svF,
                                 uint32_t kp, uint32_t ki, uint32_t kd);

void ZDT_Motor_Read_DMX512_Params(uint8_t addr);

void ZDT_Motor_Modify_DMX512_Params(uint8_t addr, bool svF,
    uint16_t tch, uint8_t nch, uint8_t mode,
    uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step);

void ZDT_Motor_Read_Pos_Window(uint8_t addr);
void ZDT_Motor_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw);
void ZDT_Motor_Read_Otocp(uint8_t addr);
void ZDT_Motor_Modify_Otocp(uint8_t addr, bool svF,
                            uint16_t otp, uint16_t ocp, uint16_t time_ms);
void ZDT_Motor_Read_Heart_Protect(uint8_t addr);
void ZDT_Motor_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp);
void ZDT_Motor_Read_Integral_Limit(uint8_t addr);
void ZDT_Motor_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il);
void ZDT_Motor_Read_System_State_Params(uint8_t addr);
void ZDT_Motor_Read_Motor_Conf_Params(uint8_t addr);

/* ======================================================================== */
/*  MMCL（多电机命令列表）——缓冲变体                                      */
/*  这些函数将命令排入队列而非立即发送，最后调用                           */
/*  ZDT_Motor_Multi_Motor_Cmd() 一次性发出所有排队的命令                   */
/* ======================================================================== */

void ZDT_Motor_MMCL_Trig_Encoder_Cal(uint8_t addr);
void ZDT_Motor_MMCL_Reset_Motor(uint8_t addr);
void ZDT_Motor_MMCL_Reset_CurPos_To_Zero(uint8_t addr);
void ZDT_Motor_MMCL_Reset_Clog_Pro(uint8_t addr);
void ZDT_Motor_MMCL_Restore_Motor(uint8_t addr);
void ZDT_Motor_MMCL_En_Control(uint8_t addr, bool state, bool snF);

void ZDT_Motor_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                                uint8_t acc, bool snF);

void ZDT_Motor_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                                uint8_t acc, uint32_t clk, uint8_t raF, bool snF);

void ZDT_Motor_MMCL_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc,
                                    uint8_t raF, bool snF);

void ZDT_Motor_MMCL_QPos_Control(uint8_t addr, int32_t clk);
void ZDT_Motor_MMCL_Stop_Now(uint8_t addr, bool snF);
void ZDT_Motor_MMCL_Synchronous_motion(uint8_t addr);
void ZDT_Motor_MMCL_Origin_Set_O(uint8_t addr, bool svF);

void ZDT_Motor_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode,
                                          bool snF);

void ZDT_Motor_MMCL_Origin_Interrupt(uint8_t addr);

void ZDT_Motor_MMCL_Origin_Modify_Params(uint8_t addr, bool svF,
    uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm,
    uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);

void ZDT_Motor_MMCL_Origin_Read_SL_RP(uint8_t addr);
void ZDT_Motor_MMCL_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp);

void ZDT_Motor_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr,
    ZDT_Motor_Param_t s, uint16_t time_ms);

void ZDT_Motor_MMCL_Read_Sys_Params(uint8_t addr, ZDT_Motor_Param_t s);

#endif
