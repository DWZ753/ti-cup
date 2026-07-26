#ifndef __USER_UART_H__
#define __USER_UART_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ========== 硬件映射宏（SysConfig 重新生成后只需修改此部分即可适配） ========== */
#define UART_PRINT_INST             PRINT_INST              // UART 外设实例
#define UART_PRINT_INT_IRQN         PRINT_INST_INT_IRQN      // UART 中断号
#define UART_PRINT_IRQHandler       PRINT_INST_IRQHandler    // UART 中断服务函数名

/* ========== 通用参数宏 ========== */
#define UART_TX_BUF_SIZE            256   // 发送缓冲区长度（字节）
#define UART_RX_BUF_SIZE            256   // 接收缓冲区长度（字节）
#define UART_RX_TERMINATOR          '\n'  // 接收结束符（行模式）
#define UART_BUSY_TIMEOUT           10000 // 忙等待超时（防止死锁）

/* ========== 接收模式 ========== */
#define UART_RX_MODE_LINE           0     // 行模式：收到 \n 时 rxDone=1
#define UART_RX_MODE_RAW            1     // 原始模式：持续接收，FIFO 环形缓冲

/* ========== 类型定义 ========== */

/** UART 初始化配置 */
typedef struct {
    UART_Regs  *uart;          // UART 外设基址指针（如 UART0）
    IRQn_Type   irqNum;        // NVIC 中断号
    uint8_t     dmaTxChanId;   // DMA TX 通道号
    uint8_t     dmaTxTrigger;  // DMA 发送触发源
} UART_Config;

/** UART 运行时句柄，封装一个 UART 实例的全部状态 */
typedef struct {
    /* ---- 硬件引用 ---- */
    UART_Regs  *uart;          // UART 外设基址指针
    IRQn_Type   irqNum;        // NVIC 中断号
    DMA_Regs   *dma;           // DMA 控制器基址指针
    uint8_t     dmaTxChanId;   // DMA TX 通道号
    uint8_t     dmaTxTrigger;  // DMA 发送触发源

    /* ---- 发送状态 ---- */
    volatile uint8_t  txDMADone;                        // DMA 发送完成标志

    /* ---- 接收状态（行模式） ---- */
    volatile uint8_t  rxDone;                           // 接收完成标志
    volatile uint8_t  rxBuf[UART_RX_BUF_SIZE];          // 接收缓冲区
    volatile uint16_t rxPos;                            // 当前接收位置
    volatile uint16_t rxLen;                            // 接收长度（不含结束符）
    volatile uint8_t  rxOvf;                            // 溢出数据

    /* ---- 接收状态（原始模式） ---- */
    volatile uint8_t  rxMode;                           // 0=行模式, 1=原始模式
    volatile uint8_t  rxRawBuf[UART_RX_BUF_SIZE];       // 原始 RX 环形缓冲区
    volatile uint16_t rxRawHead;                        // ISR 写入位置
    volatile uint16_t rxRawTail;                        // 应用读出位置
} UART_Handle;

/* ========== 通用 API ========== */

/**
 * @brief 注册并初始化一个 UART 实例
 * @param config 初始化配置（指定外设、中断号、DMA 通道等）
 * @return 成功返回句柄指针，失败返回 NULL（重复注册 / 硬件索引无效 / 实例已满）
 * @note 调用后 NVIC 中断自动使能；DMA 初始化由 SYSCFG_DL_init() 完成
 */
UART_Handle* UART_Init(const UART_Config *config);

/* ---- 字符串 TX（line mode） ---- */

int  UART_SendStr(UART_Handle *h, const char *str);
int  UART_Printf(UART_Handle *h, char *fmt, ...);
void UART_SendStrDMA(UART_Handle *h, const char *str, uint16_t len);
void UART_PrintfDMA(UART_Handle *h, char *fmt, ...);

/* ---- 二进制 TX ---- */

/**
 * @brief 阻塞发送单个字节
 * @param h    UART 句柄指针
 * @param data 待发送字节
 * @note  忙等待超时后静默返回
 */
void UART_SendByte(UART_Handle *h, uint8_t data);

/**
 * @brief 阻塞发送字节数组
 * @param h    UART 句柄指针
 * @param data 字节数组指针
 * @param len  字节数
 */
void UART_SendData(UART_Handle *h, const uint8_t *data, uint16_t len);

/* ---- 行模式 RX ---- */

void UART_StartReceive(UART_Handle *h);

/* ---- 原始模式 RX（二进制协议用） ---- */

/**
 * @brief 切换到原始接收模式
 * @param h UART 句柄指针
 * @note  启用后 ISR 持续将接收字节写入环形缓冲区，不检测结束符。
 *        应用层通过 UART_RawRxAvailable() / UART_ReadRawByte() 读取。
 */
void UART_StartReceiveRaw(UART_Handle *h);

/**
 * @brief 获取原始缓冲区中可读字节数
 * @param h UART 句柄指针
 * @return 可读字节数（0 = 无数据）
 */
uint16_t UART_RawRxAvailable(UART_Handle *h);

/**
 * @brief 从原始缓冲区读取一个字节
 * @param h UART 句柄指针
 * @return 读出的字节（缓冲区为空时返回 0）
 */
uint8_t UART_ReadRawByte(UART_Handle *h);

/**
 * @brief 清空原始缓冲区
 * @param h UART 句柄指针
 */
void UART_FlushRawRx(UART_Handle *h);

/* ---- 中断处理 ---- */

void UART_HandleIRQ(UART_Handle *h);

/* ========== ISR 入口（由向量表调用，自动分发到已注册句柄） ========== */

void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void UART3_IRQHandler(void);

#endif
