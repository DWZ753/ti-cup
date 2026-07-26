#include "zdt_motor.h"
#include "uart.h"
#include "delay.h"

/**********************************************************
*** ZDT 闭环步进电机模块
*** 基于张大头 Emm_V5.0 协议（RS485/UART 通信）
***
*** 使用 bsp/uart 的 UART_Handle 进行通信，不直接操作 DL_UART。
*** UART ISR 由 bsp/uart 统一处理（UART0_IRQHandler 等）。
***
*** 协议格式：[地址] [命令] [数据...] [0x6B 校验]
*** 所有命令均以固定校验字节 0x6B 结尾
**********************************************************/

/* ========== 模块状态 ========== */

static volatile uint8_t  g_motor_initialized = 0;
static UART_Handle      *g_uart_h = NULL;   // 电机通信 UART 句柄

/* ========== 外部全局变量 ========== */

volatile uint8_t  zdt_motor_rx_cmd[ZDT_MOTOR_RX_BUF_SIZE] = {0};
volatile uint8_t  zdt_motor_rx_count = 0;
volatile uint16_t zdt_motor_mmcl_count = 0;
volatile uint8_t  zdt_motor_mmcl_cmd[ZDT_MOTOR_MMCL_LEN] = {0};

/* ======================================================================== */
/*  内部 UART 操作（通过 bsp/uart 句柄）                                   */
/* ======================================================================== */

static void send_byte(uint8_t data)
{
    UART_SendByte(g_uart_h, data);
}

static void send_cmd(const volatile uint8_t *cmd, uint8_t len)
{
    UART_SendData(g_uart_h, (const uint8_t *)cmd, len);
}

/* ======================================================================== */
/*  公共 API                                                                */
/* ======================================================================== */

void ZDT_Motor_GetRxCmd(void)
{
    uint16_t i;
    uint16_t count = UART_RawRxAvailable(g_uart_h);

    zdt_motor_rx_count = (count > ZDT_MOTOR_RX_BUF_SIZE)
                       ? ZDT_MOTOR_RX_BUF_SIZE : (uint8_t)count;
    for (i = 0; i < zdt_motor_rx_count; i++)
    {
        zdt_motor_rx_cmd[i] = UART_ReadRawByte(g_uart_h);
    }
}

/* ======================================================================== */
/*  初始化                                                                 */
/* ======================================================================== */

void ZDT_Motor_Init(void)
{
    UART_Config cfg;

    if (g_motor_initialized) return;

    /* 通过 bsp/uart 注册 UART 实例（ISR 由 bsp/uart 统一接管） */
    cfg.uart         = (UART_Regs *)ZDT_MOTOR_UART_INST;
    cfg.irqNum       = ZDT_MOTOR_UART_INT_IRQN;
    cfg.dmaTxChanId  = 0;    // 不使用 DMA
    cfg.dmaTxTrigger = 0;

    g_uart_h = UART_Init(&cfg);
    if (g_uart_h == NULL) return;

    /* 进入原始接收模式（二进制协议，无结束符） */
    UART_StartReceiveRaw(g_uart_h);

    /* 清零 MMCL 缓冲 */
    zdt_motor_mmcl_count = 0;

    /* 上电延时 500ms 等待电机驱动板初始化 */
    delay_ms(500);

    g_motor_initialized = 1;
}

/* ======================================================================== */
/*  系统命令                                                               */
/* ======================================================================== */

/**
 * @brief    触发编码器校准
 * @param    addr  电机地址
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Trig_Encoder_Cal(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;              // 地址
    cmd[1] = 0x06;              // 命令字
    cmd[2] = 0x45;              // 命令状态
    cmd[3] = 0x6B;              // 校验字节

    // 发送命令
    send_cmd(cmd, 4);
}

/**
 * @brief    重置电机（Y42）
 * @param    addr  电机地址
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Reset_Motor(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x08;
    cmd[2] = 0x97;
    cmd[3] = 0x6B;

    send_cmd(cmd, 4);
}

/**
 * @brief    将当前位置清零
 * @param    addr  电机地址
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Reset_CurPos_To_Zero(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x0A;
    cmd[2] = 0x6D;
    cmd[3] = 0x6B;

    send_cmd(cmd, 4);
}

/**
 * @brief    清除堵转保护
 * @param    addr  电机地址
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Reset_Clog_Pro(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x0E;
    cmd[2] = 0x52;
    cmd[3] = 0x6B;

    send_cmd(cmd, 4);
}

/**
 * @brief    恢复出厂设置
 * @param    addr  电机地址
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Restore_Motor(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x0F;
    cmd[2] = 0x5F;
    cmd[3] = 0x6B;

    send_cmd(cmd, 4);
}

/* ======================================================================== */
/*  运动控制命令                                                           */
/* ======================================================================== */

/**
 * @brief    组合命令（Y42）
 * @param    addr  电机地址
 * @retval   地址 + 命令字 + 包长度 + 子命令列表 + 校验字节
 */
void ZDT_Motor_Multi_Motor_Cmd(uint8_t addr)
{
    uint16_t i, j, len;
    static volatile uint8_t cmd[ZDT_MOTOR_MMCL_LEN] = {0};

    // 命令列表长度大于 0
    if (zdt_motor_mmcl_count > 0)
    {
        // 计算包总字节数
        len = (uint16_t)(zdt_motor_mmcl_count + 5);

        // 装载命令
        cmd[0] = addr;
        cmd[1] = 0xAA;
        cmd[2] = (uint8_t)(len >> 8);       // 高字节包长度
        cmd[3] = (uint8_t)(len);            // 低字节包长度
        for (i = 0, j = 4; i < zdt_motor_mmcl_count; i++, j++)
        {
            cmd[j] = zdt_motor_mmcl_cmd[i];
        }
        cmd[j] = 0x6B;
        ++j;

        // 发送命令
        send_cmd(cmd, j);
        zdt_motor_mmcl_count = 0;
    } else
    {
        zdt_motor_mmcl_count = 0;
    }
}

/**
 * @brief    使能信号控制
 * @param    addr   电机地址
 * @param    state  使能状态（true=使能，false=关闭）
 * @param    snF    同步运动标志（false=不启用，true=启用）
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_En_Control(uint8_t addr, bool state, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xF3;
    cmd[2] = 0xAB;
    cmd[3] = (uint8_t)state;
    cmd[4] = snF;
    cmd[5] = 0x6B;

    send_cmd(cmd, 6);
}

/**
 * @brief    速度模式
 * @param    addr  电机地址
 * @param    dir   方向（0=CW，非0=CCW）
 * @param    vel   速度(RPM)，范围 0-5000
 * @param    acc   加速度，范围 0-255（0=直冲）
 * @param    snF   同步运动标志
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                           uint8_t acc, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);           // 速度高 8 位
    cmd[4] = (uint8_t)(vel >> 0);           // 速度低 8 位
    cmd[5] = acc;
    cmd[6] = snF;
    cmd[7] = 0x6B;

    send_cmd(cmd, 8);
}

/**
 * @brief    位置模式
 * @param    addr  电机地址
 * @param    dir   方向（0=CW，非0=CCW）
 * @param    vel   速度(RPM)，范围 0-5000
 * @param    acc   加速度，范围 0-255（0=直冲）
 * @param    clk   脉冲数，范围 0-(2^32-1)。16 细分下 3200=1 圈
 * @param    raF   运动参考系：0=相对上次目标位置运动，
 *                 1=绝对位置运动，2=相对当前实时位置运动
 * @param    snF   同步运动标志
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                           uint8_t acc, uint32_t clk, uint8_t raF, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0]  = addr;
    cmd[1]  = 0xFD;
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(vel >> 8);
    cmd[4]  = (uint8_t)(vel >> 0);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(clk >> 24);         // 脉冲数 bit24-bit31
    cmd[7]  = (uint8_t)(clk >> 16);         // 脉冲数 bit16-bit23
    cmd[8]  = (uint8_t)(clk >> 8);          // 脉冲数 bit8-bit15
    cmd[9]  = (uint8_t)(clk >> 0);          // 脉冲数 bit0-bit7
    cmd[10] = raF;
    cmd[11] = snF;
    cmd[12] = 0x6B;

    send_cmd(cmd, 13);
}

/**
 * @brief    设置快速位置模式的运动参数
 * @param    addr  电机地址
 * @param    vel   速度(RPM)，范围 0-5000
 * @param    acc   加速度，范围 0-255（0=直冲）
 * @param    raF   运动参考系
 * @param    snF   同步运动标志
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc,
                               uint8_t raF, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xF1;
    cmd[2] = (uint8_t)(vel >> 8);
    cmd[3] = (uint8_t)(vel >> 0);
    cmd[4] = acc;
    cmd[5] = raF;
    cmd[6] = snF;
    cmd[7] = 0x6B;

    send_cmd(cmd, 8);
}

/**
 * @brief    快速位置模式
 * @param    addr  电机地址
 * @param    clk   带符号脉冲数。+N=CW N 脉冲，-N=CCW N 脉冲。
 *                 16 细分下 ±3200=±1 圈
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_QPos_Control(uint8_t addr, int32_t clk)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xFC;
    cmd[2] = (uint8_t)(clk >> 24);
    cmd[3] = (uint8_t)(clk >> 16);
    cmd[4] = (uint8_t)(clk >> 8);
    cmd[5] = (uint8_t)(clk >> 0);
    cmd[6] = 0x6B;

    send_cmd(cmd, 7);
}

/**
 * @brief    强制停止
 * @param    addr  电机地址
 * @param    snF   同步运动标志
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Stop_Now(uint8_t addr, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xFE;
    cmd[2] = 0x98;
    cmd[3] = snF;
    cmd[4] = 0x6B;

    send_cmd(cmd, 5);
}

/**
 * @brief    多机同步运动
 * @param    addr  电机地址（通常为 0，广播）
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Synchronous_motion(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xFF;
    cmd[2] = 0x66;
    cmd[3] = 0x6B;

    send_cmd(cmd, 4);
}

/* ======================================================================== */
/*  原点/回零命令                                                          */
/* ======================================================================== */

/**
 * @brief    设定线圈通电位置为原点
 * @param    addr  电机地址
 * @param    svF   是否存储标志（false=不存储，true=存储）
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Origin_Set_O(uint8_t addr, bool svF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x93;
    cmd[2] = 0x88;
    cmd[3] = svF;
    cmd[4] = 0x6B;

    send_cmd(cmd, 5);
}

/**
 * @brief    触发回零
 * @param    addr    电机地址
 * @param    o_mode  回零模式：0=近限位回零，1=近限位缩回，
 *                   2=硬停回零，3=硬停缩回
 * @param    snF     同步运动标志
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x9A;
    cmd[2] = o_mode;
    cmd[3] = snF;
    cmd[4] = 0x6B;

    send_cmd(cmd, 5);
}

/**
 * @brief    强制中断并退出回零
 * @param    addr  电机地址
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Origin_Interrupt(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x9C;
    cmd[2] = 0x48;
    cmd[3] = 0x6B;

    send_cmd(cmd, 4);
}

/**
 * @brief    读取回零参数
 * @param    addr     电机地址
 * @retval   地址 + 命令字 + 校验字节
 */
void ZDT_Motor_Origin_Read_Params(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x22;
    cmd[2] = 0x6B;

    send_cmd(cmd, 3);
}

/**
 * @brief    修改回零参数
 * @param    addr   电机地址
 * @param    svF    是否存储标志（false=不存储，true=存储）
 * @param    o_mode 回零模式：0=近限位回零，1=近限位缩回，
 *                  2=硬停回零，3=硬停缩回
 * @param    o_dir  回零方向（0=CW，非0=CCW）
 * @param    o_vel  回零速度(RPM)
 * @param    o_tm   回零超时时间(ms)
 * @param    sl_vel 硬停碰撞检测转速(RPM)
 * @param    sl_ma  硬停碰撞检测电流(mA)
 * @param    sl_ms  硬停碰撞检测时间(ms)
 * @param    potF   上电自动触发回零（false=不使能，true=使能）
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode,
    uint8_t o_dir, uint16_t o_vel, uint32_t o_tm,
    uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    static volatile uint8_t cmd[32] = {0};

    cmd[0]  = addr;
    cmd[1]  = 0x4C;
    cmd[2]  = 0xAE;
    cmd[3]  = svF;
    cmd[4]  = o_mode;
    cmd[5]  = o_dir;
    cmd[6]  = (uint8_t)(o_vel >> 8);
    cmd[7]  = (uint8_t)(o_vel >> 0);
    cmd[8]  = (uint8_t)(o_tm >> 24);
    cmd[9]  = (uint8_t)(o_tm >> 16);
    cmd[10] = (uint8_t)(o_tm >> 8);
    cmd[11] = (uint8_t)(o_tm >> 0);
    cmd[12] = (uint8_t)(sl_vel >> 8);
    cmd[13] = (uint8_t)(sl_vel >> 0);
    cmd[14] = (uint8_t)(sl_ma >> 8);
    cmd[15] = (uint8_t)(sl_ma >> 0);
    cmd[16] = (uint8_t)(sl_ms >> 8);
    cmd[17] = (uint8_t)(sl_ms >> 0);
    cmd[18] = potF;
    cmd[19] = 0x6B;

    send_cmd(cmd, 20);
}

/**
 * @brief    读取硬停回零返回角度（X42S/Y42）
 * @param    addr     电机地址
 * @retval   地址 + 命令字 + 校验字节
 */
void ZDT_Motor_Origin_Read_SL_RP(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x3F;
    cmd[2] = 0x6B;

    send_cmd(cmd, 3);
}

/**
 * @brief    修改硬停回零返回角度（X42S/Y42）
 * @param    addr     电机地址
 * @param    svF      是否存储标志（false=不存储，true=存储）
 * @param    sl_rp    硬停回零返回角度（单位 0.1°，如 40=4.0°）
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x5C;
    cmd[2] = 0xAC;
    cmd[3] = svF;
    cmd[4] = (uint8_t)(sl_rp >> 8);
    cmd[5] = (uint8_t)(sl_rp >> 0);
    cmd[6] = 0x6B;

    send_cmd(cmd, 7);
}

/* ======================================================================== */
/*  读取系统参数                                                           */
/* ======================================================================== */

/**
 * @brief    定时返回信息命令（Y42）
 * @param    addr     电机地址
 * @param    s        系统参数类型
 * @param    time_ms  间隔时间(ms)
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Auto_Return_Sys_Params_Timed(uint8_t addr,
    ZDT_Motor_Param_t s, uint16_t time_ms)
{
    uint8_t i = 0;
    static volatile uint8_t cmd[16] = {0};

    cmd[i] = addr; ++i;
    cmd[i] = 0x11; ++i;
    cmd[i] = 0x18; ++i;

    switch (s)
    {
        case ZDT_PARAM_VBUS:  cmd[i] = 0x24; ++i; break;
        case ZDT_PARAM_CBUS:  cmd[i] = 0x26; ++i; break;
        case ZDT_PARAM_CPHA:  cmd[i] = 0x27; ++i; break;
        case ZDT_PARAM_ENCO:  cmd[i] = 0x29; ++i; break;
        case ZDT_PARAM_CLKC:  cmd[i] = 0x30; ++i; break;
        case ZDT_PARAM_ENCL:  cmd[i] = 0x31; ++i; break;
        case ZDT_PARAM_CLKI:  cmd[i] = 0x32; ++i; break;
        case ZDT_PARAM_TPOS:  cmd[i] = 0x33; ++i; break;
        case ZDT_PARAM_SPOS:  cmd[i] = 0x34; ++i; break;
        case ZDT_PARAM_VEL:   cmd[i] = 0x35; ++i; break;
        case ZDT_PARAM_CPOS:  cmd[i] = 0x36; ++i; break;
        case ZDT_PARAM_PERR:  cmd[i] = 0x37; ++i; break;
        case ZDT_PARAM_VBAT:  cmd[i] = 0x38; ++i; break;
        case ZDT_PARAM_TEMP:  cmd[i] = 0x39; ++i; break;
        case ZDT_PARAM_FLAG:  cmd[i] = 0x3A; ++i; break;
        case ZDT_PARAM_OFLAG: cmd[i] = 0x3B; ++i; break;
        case ZDT_PARAM_OAF:   cmd[i] = 0x3C; ++i; break;
        case ZDT_PARAM_PIN:   cmd[i] = 0x3D; ++i; break;
        default: break;
    }

    cmd[i] = (uint8_t)(time_ms >> 8);  ++i;
    cmd[i] = (uint8_t)(time_ms >> 0);  ++i;
    cmd[i] = 0x6B; ++i;

    send_cmd(cmd, i);
}

/**
 * @brief    读取系统参数
 * @param    addr  电机地址
 * @param    s     系统参数类型
 * @retval   地址 + 命令字 + 校验字节
 */
void ZDT_Motor_Read_Sys_Params(uint8_t addr, ZDT_Motor_Param_t s)
{
    uint8_t i = 0;
    static volatile uint8_t cmd[16] = {0};

    cmd[i] = addr; ++i;

    switch (s)
    {
        case ZDT_PARAM_VBUS:  cmd[i] = 0x24; ++i; break;
        case ZDT_PARAM_CBUS:  cmd[i] = 0x26; ++i; break;
        case ZDT_PARAM_CPHA:  cmd[i] = 0x27; ++i; break;
        case ZDT_PARAM_ENCO:  cmd[i] = 0x29; ++i; break;
        case ZDT_PARAM_CLKC:  cmd[i] = 0x30; ++i; break;
        case ZDT_PARAM_ENCL:  cmd[i] = 0x31; ++i; break;
        case ZDT_PARAM_CLKI:  cmd[i] = 0x32; ++i; break;
        case ZDT_PARAM_TPOS:  cmd[i] = 0x33; ++i; break;
        case ZDT_PARAM_SPOS:  cmd[i] = 0x34; ++i; break;
        case ZDT_PARAM_VEL:   cmd[i] = 0x35; ++i; break;
        case ZDT_PARAM_CPOS:  cmd[i] = 0x36; ++i; break;
        case ZDT_PARAM_PERR:  cmd[i] = 0x37; ++i; break;
        case ZDT_PARAM_VBAT:  cmd[i] = 0x38; ++i; break;
        case ZDT_PARAM_TEMP:  cmd[i] = 0x39; ++i; break;
        case ZDT_PARAM_FLAG:  cmd[i] = 0x3A; ++i; break;
        case ZDT_PARAM_OFLAG: cmd[i] = 0x3B; ++i; break;
        case ZDT_PARAM_OAF:   cmd[i] = 0x3C; ++i; break;
        case ZDT_PARAM_PIN:   cmd[i] = 0x3D; ++i; break;
        default: break;
    }

    cmd[i] = 0x6B; ++i;
    send_cmd(cmd, i);
}

/* ======================================================================== */
/*  配置命令                                                               */
/* ======================================================================== */

/**
 * @brief    修改电机 ID 地址
 * @param    addr     电机地址
 * @param    svF      是否存储标志（false=不存储，true=存储）
 * @param    id       默认电机 ID 为 1，可修改为 1-255（0 为广播地址）
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xAE;
    cmd[2] = 0x4B;
    cmd[3] = svF;
    cmd[4] = id;
    cmd[5] = 0x6B;

    send_cmd(cmd, 6);
}

/**
 * @brief    修改细分值
 * @param    addr     电机地址
 * @param    svF      是否存储标志（false=不存储，true=存储）
 * @param    mstep    默认细分 16，可修改为 1-255（0=256 细分）
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x84;
    cmd[2] = 0x8A;
    cmd[3] = svF;
    cmd[4] = mstep;
    cmd[5] = 0x6B;

    send_cmd(cmd, 6);
}

/**
 * @brief    修改电机标志
 * @param    addr     电机地址
 * @param    pdf      电机标志
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_PDFlag(uint8_t addr, bool pdf)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x50;
    cmd[2] = pdf;
    cmd[3] = 0x6B;

    send_cmd(cmd, 4);
}

/**
 * @brief    读取选择电机状态（Y42）
 * @param    addr     电机地址
 * @retval   地址 + 命令字 + 校验字节
 */
void ZDT_Motor_Read_Opt_Param_Sta(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x1A;
    cmd[2] = 0x6B;

    send_cmd(cmd, 3);
}

/**
 * @brief    修改电机类型（Y42）
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    mottype  电机类型：false=1.8°步进电机，true=0.9°步进电机
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype)
{
    static volatile uint8_t cmd[16] = {0};
    uint8_t mot_type_val = mottype ? 25 : 50;

    cmd[0] = addr;
    cmd[1] = 0xD7;
    cmd[2] = 0x35;
    cmd[3] = svF;
    cmd[4] = mot_type_val;
    cmd[5] = 0x6B;

    send_cmd(cmd, 6);
}

/**
 * @brief    修改固件类型（Y42）
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    fwtype   固件类型：0=X 固件，1=Emm 固件
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xD5;
    cmd[2] = 0x69;
    cmd[3] = svF;
    cmd[4] = fwtype;
    cmd[5] = 0x6B;

    send_cmd(cmd, 6);
}

/**
 * @brief    修改开环/闭环控制模式（Y42）
 * @param    addr       电机地址
 * @param    svF        是否存储标志
 * @param    ctrl_mode  控制模式：0=开环模式，1=闭环 FOC 模式
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x46;
    cmd[2] = 0x69;
    cmd[3] = svF;
    cmd[4] = ctrl_mode;
    cmd[5] = 0x6B;

    send_cmd(cmd, 6);
}

/**
 * @brief    修改电机运动方向（Y42）
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    dir      电机运动方向：0=CW（顺时针），1=CCW
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xD4;
    cmd[2] = 0x60;
    cmd[3] = svF;
    cmd[4] = dir;
    cmd[5] = 0x6B;

    send_cmd(cmd, 6);
}

/**
 * @brief    修改按键锁定功能（Y42）
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    lock     按键锁定功能：0=不使能，1=使能
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_Lock_Btn(uint8_t addr, bool svF, bool lock)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xD0;
    cmd[2] = 0xB3;
    cmd[3] = svF;
    cmd[4] = lock;
    cmd[5] = 0x6B;

    send_cmd(cmd, 6);
}

/**
 * @brief    修改输入速度值是否缩小 10 倍输入（Y42）
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    s_vel    速度缩放：0=不使能，1=使能
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x4F;
    cmd[2] = 0x71;
    cmd[3] = svF;
    cmd[4] = s_vel;
    cmd[5] = 0x6B;

    send_cmd(cmd, 6);
}

/**
 * @brief    修改开环模式电流(mA)
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    om_ma    开环模式电流，单位 mA
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x44;
    cmd[2] = 0x33;
    cmd[3] = svF;
    cmd[4] = (uint8_t)(om_ma >> 8);
    cmd[5] = (uint8_t)(om_ma >> 0);
    cmd[6] = 0x6B;

    send_cmd(cmd, 7);
}

/**
 * @brief    修改闭环模式电流(mA)
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    foc_mA   闭环模式电流，单位 mA
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x45;
    cmd[2] = 0x66;
    cmd[3] = svF;
    cmd[4] = (uint8_t)(foc_mA >> 8);
    cmd[5] = (uint8_t)(foc_mA >> 0);
    cmd[6] = 0x6B;

    send_cmd(cmd, 7);
}

/**
 * @brief    读取 PID 参数
 * @param    addr     电机地址
 * @retval   地址 + 命令字 + 校验字节
 */
void ZDT_Motor_Read_PID_Params(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x21;
    cmd[2] = 0x6B;

    send_cmd(cmd, 3);
}

/**
 * @brief    修改 PID 参数
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    kp       比例系数，默认 Y42/18000
 * @param    ki       积分系数，默认 Y42/10
 * @param    kd       微分系数，默认 Y42/18000
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_PID_Params(uint8_t addr, bool svF,
                                 uint32_t kp, uint32_t ki, uint32_t kd)
{
    static volatile uint8_t cmd[20] = {0};

    cmd[0]  = addr;
    cmd[1]  = 0x4A;
    cmd[2]  = 0xC3;
    cmd[3]  = svF;
    cmd[4]  = (uint8_t)(kp >> 24);
    cmd[5]  = (uint8_t)(kp >> 16);
    cmd[6]  = (uint8_t)(kp >> 8);
    cmd[7]  = (uint8_t)(kp >> 0);
    cmd[8]  = (uint8_t)(ki >> 24);
    cmd[9]  = (uint8_t)(ki >> 16);
    cmd[10] = (uint8_t)(ki >> 8);
    cmd[11] = (uint8_t)(ki >> 0);
    cmd[12] = (uint8_t)(kd >> 24);
    cmd[13] = (uint8_t)(kd >> 16);
    cmd[14] = (uint8_t)(kd >> 8);
    cmd[15] = (uint8_t)(kd >> 0);
    cmd[16] = 0x6B;

    send_cmd(cmd, 17);
}

/**
 * @brief    读取 DMX512 协议参数（Y42）
 * @param    addr     电机地址
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Read_DMX512_Params(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x49;
    cmd[2] = 0x78;
    cmd[3] = 0x6B;

    send_cmd(cmd, 4);
}

/**
 * @brief    修改 DMX512 协议参数（Y42）
 * @param    addr      电机地址
 * @param    svF       是否存储标志
 * @param    tch       起始通道数，默认 192
 * @param    nch       每台电机占用的通道数：1=单通道模式，2=双通道模式
 * @param    mode      运动模式：0=实时位置模式，1=增量式位置模式
 * @param    vel       单通道模式运动速度(RPM)，默认 1000
 * @param    acc       加速度
 * @param    vel_step  双通道模式速度步进，默认 10
 * @param    pos_step  双通道模式运动步进，默认 100
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_DMX512_Params(uint8_t addr, bool svF,
    uint16_t tch, uint8_t nch, uint8_t mode,
    uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step)
{
    static volatile uint8_t cmd[32] = {0};

    cmd[0]  = addr;
    cmd[1]  = 0xD9;
    cmd[2]  = 0x90;
    cmd[3]  = svF;
    cmd[4]  = (uint8_t)(tch >> 8);
    cmd[5]  = (uint8_t)(tch >> 0);
    cmd[6]  = nch;
    cmd[7]  = mode;
    cmd[8]  = (uint8_t)(vel >> 8);
    cmd[9]  = (uint8_t)(vel >> 0);
    cmd[10] = (uint8_t)(acc >> 8);
    cmd[11] = (uint8_t)(acc >> 0);
    cmd[12] = (uint8_t)(vel_step >> 8);
    cmd[13] = (uint8_t)(vel_step >> 0);
    cmd[14] = (uint8_t)(pos_step >> 24);
    cmd[15] = (uint8_t)(pos_step >> 16);
    cmd[16] = (uint8_t)(pos_step >> 8);
    cmd[17] = (uint8_t)(pos_step >> 0);
    cmd[18] = 0x6B;

    send_cmd(cmd, 19);
}

/**
 * @brief    读取位置到达窗口（Y42）
 * @param    addr     电机地址
 * @retval   地址 + 命令字 + 校验字节
 */
void ZDT_Motor_Read_Pos_Window(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x41;
    cmd[2] = 0x6B;

    send_cmd(cmd, 3);
}

/**
 * @brief    修改位置到达窗口（Y42）
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    prw      位置到达窗口，默认值 8（即 0.8°）
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xD1;
    cmd[2] = 0x07;
    cmd[3] = svF;
    cmd[4] = (uint8_t)(prw >> 8);
    cmd[5] = (uint8_t)(prw >> 0);
    cmd[6] = 0x6B;

    send_cmd(cmd, 7);
}

/**
 * @brief    读取过热过流保护阈值（Y42）
 * @param    addr     电机地址
 * @retval   地址 + 命令字 + 校验字节
 */
void ZDT_Motor_Read_Otocp(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x13;
    cmd[2] = 0x6B;

    send_cmd(cmd, 3);
}

/**
 * @brief    修改过热过流保护阈值（Y42）
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    otp      过热保护阈值，默认 100°
 * @param    ocp      过流保护阈值，默认 6600mA
 * @param    time_ms  过热过流触发时间，默认 1000ms
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_Otocp(uint8_t addr, bool svF,
                            uint16_t otp, uint16_t ocp, uint16_t time_ms)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0]  = addr;
    cmd[1]  = 0xD3;
    cmd[2]  = 0x56;
    cmd[3]  = svF;
    cmd[4]  = (uint8_t)(otp >> 8);
    cmd[5]  = (uint8_t)(otp >> 0);
    cmd[6]  = (uint8_t)(ocp >> 8);
    cmd[7]  = (uint8_t)(ocp >> 0);
    cmd[8]  = (uint8_t)(time_ms >> 8);
    cmd[9]  = (uint8_t)(time_ms >> 0);
    cmd[10] = 0x6B;

    send_cmd(cmd, 11);
}

/**
 * @brief    读取心跳保护时间（Y42）
 * @param    addr     电机地址
 * @retval   地址 + 命令字 + 校验字节
 */
void ZDT_Motor_Read_Heart_Protect(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x16;
    cmd[2] = 0x6B;

    send_cmd(cmd, 3);
}

/**
 * @brief    修改心跳保护时间（Y42）
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    hp       心跳保护时间，单位 ms
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x68;
    cmd[2] = 0x38;
    cmd[3] = svF;
    cmd[4] = (uint8_t)(hp >> 24);
    cmd[5] = (uint8_t)(hp >> 16);
    cmd[6] = (uint8_t)(hp >> 8);
    cmd[7] = (uint8_t)(hp >> 0);
    cmd[8] = 0x6B;

    send_cmd(cmd, 9);
}

/**
 * @brief    读取积分限幅/微分系数（Y42）
 * @param    addr     电机地址
 * @retval   地址 + 命令字 + 校验字节
 */
void ZDT_Motor_Read_Integral_Limit(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x23;
    cmd[2] = 0x6B;

    send_cmd(cmd, 3);
}

/**
 * @brief    修改积分限幅/微分系数（Y42）
 * @param    addr     电机地址
 * @param    svF      是否存储标志
 * @param    il       积分限幅，默认值 65535
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x4B;
    cmd[2] = 0x57;
    cmd[3] = svF;
    cmd[4] = (uint8_t)(il >> 24);
    cmd[5] = (uint8_t)(il >> 16);
    cmd[6] = (uint8_t)(il >> 8);
    cmd[7] = (uint8_t)(il >> 0);
    cmd[8] = 0x6B;

    send_cmd(cmd, 9);
}

/**
 * @brief    读取系统状态参数
 * @param    addr     电机地址
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Read_System_State_Params(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x43;
    cmd[2] = 0x7A;
    cmd[3] = 0x6B;

    send_cmd(cmd, 4);
}

/**
 * @brief    读取电机配置参数
 * @param    addr     电机地址
 * @retval   地址 + 命令字 + 命令状态 + 校验字节
 */
void ZDT_Motor_Read_Motor_Conf_Params(uint8_t addr)
{
    static volatile uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0x42;
    cmd[2] = 0x6C;
    cmd[3] = 0x6B;

    send_cmd(cmd, 4);
}

/* ======================================================================== */
/*  MMCL（多电机命令列表）——缓冲变体                                      */
/*  这些函数将命令排入 MMCL_cmd[] 队列而非立即发送。                       */
/*  最后调用 ZDT_Motor_Multi_Motor_Cmd() 一次性发出所有命令。              */
/* ======================================================================== */

static void mmcl_append(const volatile uint8_t *cmd, uint8_t len)
{
    uint8_t j;
    for (j = 0; j < len; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Trig_Encoder_Cal(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x06; cmd[2] = 0x45; cmd[3] = 0x6B;
    mmcl_append(cmd, 4);
}

void ZDT_Motor_MMCL_Reset_Motor(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x08; cmd[2] = 0x97; cmd[3] = 0x6B;
    mmcl_append(cmd, 4);
}

void ZDT_Motor_MMCL_Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x0A; cmd[2] = 0x6D; cmd[3] = 0x6B;
    mmcl_append(cmd, 4);
}

void ZDT_Motor_MMCL_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x0E; cmd[2] = 0x52; cmd[3] = 0x6B;
    mmcl_append(cmd, 4);
}

void ZDT_Motor_MMCL_Restore_Motor(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x0F; cmd[2] = 0x5F; cmd[3] = 0x6B;
    mmcl_append(cmd, 4);
}

void ZDT_Motor_MMCL_En_Control(uint8_t addr, bool state, bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xF3; cmd[2] = 0xAB;
    cmd[3] = (uint8_t)state; cmd[4] = snF; cmd[5] = 0x6B;
    for (j = 0; j < 6; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                                uint8_t acc, bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xF6; cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8); cmd[4] = (uint8_t)(vel >> 0);
    cmd[5] = acc; cmd[6] = snF; cmd[7] = 0x6B;
    for (j = 0; j < 8; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                                uint8_t acc, uint32_t clk, uint8_t raF,
                                bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0]  = addr; cmd[1]  = 0xFD; cmd[2]  = dir;
    cmd[3]  = (uint8_t)(vel >> 8);  cmd[4]  = (uint8_t)(vel >> 0);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(clk >> 24); cmd[7]  = (uint8_t)(clk >> 16);
    cmd[8]  = (uint8_t)(clk >> 8);  cmd[9]  = (uint8_t)(clk >> 0);
    cmd[10] = raF; cmd[11] = snF; cmd[12] = 0x6B;
    for (j = 0; j < 13; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc,
                                    uint8_t raF, bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xF1;
    cmd[2] = (uint8_t)(vel >> 8); cmd[3] = (uint8_t)(vel >> 0);
    cmd[4] = acc; cmd[5] = raF; cmd[6] = snF; cmd[7] = 0x6B;
    for (j = 0; j < 8; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_QPos_Control(uint8_t addr, int32_t clk)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xFC;
    cmd[2] = (uint8_t)(clk >> 24); cmd[3] = (uint8_t)(clk >> 16);
    cmd[4] = (uint8_t)(clk >> 8);  cmd[5] = (uint8_t)(clk >> 0);
    cmd[6] = 0x6B;
    for (j = 0; j < 7; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Stop_Now(uint8_t addr, bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xFE; cmd[2] = 0x98;
    cmd[3] = snF; cmd[4] = 0x6B;
    for (j = 0; j < 5; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Synchronous_motion(uint8_t addr)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0xFF; cmd[2] = 0x66; cmd[3] = 0x6B;
    for (j = 0; j < 4; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Origin_Set_O(uint8_t addr, bool svF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x93; cmd[2] = 0x88;
    cmd[3] = svF; cmd[4] = 0x6B;
    for (j = 0; j < 5; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode,
                                          bool snF)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x9A;
    cmd[2] = o_mode; cmd[3] = snF; cmd[4] = 0x6B;
    for (j = 0; j < 5; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Origin_Interrupt(uint8_t addr)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x9C; cmd[2] = 0x48; cmd[3] = 0x6B;
    for (j = 0; j < 4; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Origin_Modify_Params(uint8_t addr, bool svF,
    uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm,
    uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t j, cmd[32] = {0};
    cmd[0]  = addr; cmd[1]  = 0x4C; cmd[2]  = 0xAE; cmd[3]  = svF;
    cmd[4]  = o_mode; cmd[5]  = o_dir;
    cmd[6]  = (uint8_t)(o_vel >> 8);  cmd[7]  = (uint8_t)(o_vel >> 0);
    cmd[8]  = (uint8_t)(o_tm >> 24);  cmd[9]  = (uint8_t)(o_tm >> 16);
    cmd[10] = (uint8_t)(o_tm >> 8);   cmd[11] = (uint8_t)(o_tm >> 0);
    cmd[12] = (uint8_t)(sl_vel >> 8); cmd[13] = (uint8_t)(sl_vel >> 0);
    cmd[14] = (uint8_t)(sl_ma >> 8);  cmd[15] = (uint8_t)(sl_ma >> 0);
    cmd[16] = (uint8_t)(sl_ms >> 8);  cmd[17] = (uint8_t)(sl_ms >> 0);
    cmd[18] = potF; cmd[19] = 0x6B;
    for (j = 0; j < 20; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Origin_Read_SL_RP(uint8_t addr)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x3F; cmd[2] = 0x6B;
    for (j = 0; j < 3; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Origin_Modify_SL_RP(uint8_t addr, bool svF,
                                        uint16_t sl_rp)
{
    uint8_t j, cmd[16] = {0};
    cmd[0] = addr; cmd[1] = 0x5C; cmd[2] = 0xAC; cmd[3] = svF;
    cmd[4] = (uint8_t)(sl_rp >> 8); cmd[5] = (uint8_t)(sl_rp >> 0);
    cmd[6] = 0x6B;
    for (j = 0; j < 7; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr,
    ZDT_Motor_Param_t s, uint16_t time_ms)
{
    uint8_t i = 0, j, cmd[16] = {0};
    cmd[i] = addr; ++i;
    cmd[i] = 0x11; ++i;
    cmd[i] = 0x18; ++i;
    switch (s)
    {
        case ZDT_PARAM_VBUS:  cmd[i] = 0x24; ++i; break;
        case ZDT_PARAM_CBUS:  cmd[i] = 0x26; ++i; break;
        case ZDT_PARAM_CPHA:  cmd[i] = 0x27; ++i; break;
        case ZDT_PARAM_ENCO:  cmd[i] = 0x29; ++i; break;
        case ZDT_PARAM_CLKC:  cmd[i] = 0x30; ++i; break;
        case ZDT_PARAM_ENCL:  cmd[i] = 0x31; ++i; break;
        case ZDT_PARAM_CLKI:  cmd[i] = 0x32; ++i; break;
        case ZDT_PARAM_TPOS:  cmd[i] = 0x33; ++i; break;
        case ZDT_PARAM_SPOS:  cmd[i] = 0x34; ++i; break;
        case ZDT_PARAM_VEL:   cmd[i] = 0x35; ++i; break;
        case ZDT_PARAM_CPOS:  cmd[i] = 0x36; ++i; break;
        case ZDT_PARAM_PERR:  cmd[i] = 0x37; ++i; break;
        case ZDT_PARAM_VBAT:  cmd[i] = 0x38; ++i; break;
        case ZDT_PARAM_TEMP:  cmd[i] = 0x39; ++i; break;
        case ZDT_PARAM_FLAG:  cmd[i] = 0x3A; ++i; break;
        case ZDT_PARAM_OFLAG: cmd[i] = 0x3B; ++i; break;
        case ZDT_PARAM_OAF:   cmd[i] = 0x3C; ++i; break;
        case ZDT_PARAM_PIN:   cmd[i] = 0x3D; ++i; break;
        default: break;
    }
    cmd[i] = (uint8_t)(time_ms >> 8); ++i;
    cmd[i] = (uint8_t)(time_ms >> 0); ++i;
    cmd[i] = 0x6B; ++i;
    for (j = 0; j < i; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

void ZDT_Motor_MMCL_Read_Sys_Params(uint8_t addr, ZDT_Motor_Param_t s)
{
    uint8_t i = 0, j, cmd[16] = {0};
    cmd[i] = addr; ++i;
    switch (s)
    {
        case ZDT_PARAM_VBUS:  cmd[i] = 0x24; ++i; break;
        case ZDT_PARAM_CBUS:  cmd[i] = 0x26; ++i; break;
        case ZDT_PARAM_CPHA:  cmd[i] = 0x27; ++i; break;
        case ZDT_PARAM_ENCO:  cmd[i] = 0x29; ++i; break;
        case ZDT_PARAM_CLKC:  cmd[i] = 0x30; ++i; break;
        case ZDT_PARAM_ENCL:  cmd[i] = 0x31; ++i; break;
        case ZDT_PARAM_CLKI:  cmd[i] = 0x32; ++i; break;
        case ZDT_PARAM_TPOS:  cmd[i] = 0x33; ++i; break;
        case ZDT_PARAM_SPOS:  cmd[i] = 0x34; ++i; break;
        case ZDT_PARAM_VEL:   cmd[i] = 0x35; ++i; break;
        case ZDT_PARAM_CPOS:  cmd[i] = 0x36; ++i; break;
        case ZDT_PARAM_PERR:  cmd[i] = 0x37; ++i; break;
        case ZDT_PARAM_VBAT:  cmd[i] = 0x38; ++i; break;
        case ZDT_PARAM_TEMP:  cmd[i] = 0x39; ++i; break;
        case ZDT_PARAM_FLAG:  cmd[i] = 0x3A; ++i; break;
        case ZDT_PARAM_OFLAG: cmd[i] = 0x3B; ++i; break;
        case ZDT_PARAM_OAF:   cmd[i] = 0x3C; ++i; break;
        case ZDT_PARAM_PIN:   cmd[i] = 0x3D; ++i; break;
        default: break;
    }
    cmd[i] = 0x6B; ++i;
    for (j = 0; j < i; j++)
    {
        zdt_motor_mmcl_cmd[zdt_motor_mmcl_count] = cmd[j];
        ++zdt_motor_mmcl_count;
    }
}

/* UART ISR 由 bsp/uart 统一处理（UART0_IRQHandler → UART_HandleIRQ），*/
/* 本模块不定义自己的 ISR。                                              */
