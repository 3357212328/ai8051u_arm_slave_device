#include "uart.h"
#include "set_uart.h"
#include <string.h>

// ====================================================================
// CLI 串口 (UART1) — 基于 AI8051U 库函数 set_uart
// --------------------------------------------------------------------
// 发送: 库的 uart_printf 走 DMA, 发送忙时会丢弃数据, 因此
//       UART_SendString 按块等待发送完成, 并支持超长字符串分块。
// 接收: 库的 DMA 空闲超时中断触发 get_uart_state, 数据在
//       _uart1_rx_buff[] 中 (含 \r\n), 这里剥离行结束符。
// ====================================================================

void UART_Init(void)
{
    set_uart_mode(Uart1, UART_BAUDRATE, Uart_End);
    /* set_uart 库只配置了 UR1TOCR 的 ENTO/ENTOI 和 DMA，
       但没有打开 CPU 级的串口1中断允许位 ES。
       不开 ES，接收超时中断(向量4)永远不会进入，
       uart_flag/rx_cnt 不会更新，CLI 将永远收不到命令。 */
    ES = 1;
}

/**
 * @brief  发送字符串 (自动分块, 保证每块不超过库缓冲长度)
 */
void UART_SendString(const char* str)
{
    char chunk[UART_TX_CHUNK];

    while (*str)
    {
        uint16_t n = 0;
        while (*str && n < (uint16_t)(UART_TX_CHUNK - 1))
        {
            chunk[n++] = *str++;
        }
        chunk[n] = '\0';

        // 等待上一次 DMA 发送完成 (tx_state==0 表示空闲)
        // 注意: tx_state 由 DMA 发送中断修改, 必须以 volatile 方式轮询,
        //       否则 C251 高优化等级可能把读操作提升到循环外导致死等
        while (*((volatile char *)&tx_state[Uart1])) {}

        uart_printf(Uart1, "%s", chunk);
    }
}

/**
 * @brief  获取一条完整命令 (收到后清除标志)
 * @retval 1=有命令, 0=无命令
 * @note   库的 DMA 空闲超时按"字节间隔"触发, 打字/分片发送时一帧可能
 *         只有半个命令行, 连续发送时一帧又可能包含多行。这里用行缓冲
 *         跨帧累积, 只有收到 \r/\n 才返回一条命令, 与原 STM32 版本的
 *         逐字节行接收行为一致。
 */
static char xdata s_rx_line[RX_BUFFER_SIZE];
static uint8_t xdata s_rx_line_len = 0;

/* 把 DMA 收到的一帧追加到行缓冲; 返回 1=读到了新帧 */
static uint8_t UART_LoadFrame(void)
{
    int len;
    int i;

    // 关中断后再查询/拷贝, 避免新帧的 DMA 超时中断覆盖 rx_cnt/缓冲区
    EA = 0;
    if (!get_uart_state(Uart1))
    {
        EA = 1;
        return 0;
    }

    len = rx_cnt[Uart1];
    if (len > Uart1_Rx - 1) len = Uart1_Rx - 1;
    for (i = 0; i < len && s_rx_line_len < (RX_BUFFER_SIZE - 1); i++)
    {
        s_rx_line[s_rx_line_len++] = _uart1_rx_buff[i];
    }
    EA = 1;
    return 1;
}

/**
 * @brief  从行缓冲取出一行
 * @retval 2=取到非空命令, 1=消耗了一个空行, 0=还没有行结束符
 */
static uint8_t UART_TakeLine(char* cmd_buffer)
{
    uint8_t i;
    uint8_t j = 0;
    uint8_t end = 0;
    uint8_t k = 0;

    while (end < s_rx_line_len &&
           s_rx_line[end] != '\r' && s_rx_line[end] != '\n') end++;

    /* 没收到行结束符: 继续等后续帧 (缓冲满则强制返回) */
    if (end == s_rx_line_len && s_rx_line_len < (RX_BUFFER_SIZE - 1)) return 0;

    for (i = 0; i < end; i++)
    {
        char c = s_rx_line[i];
        if (c == '\b' || c == 0x7F) { if (j > 0) j--; continue; }  /* 退格 (BS/DEL) */
        cmd_buffer[j++] = c;
    }
    cmd_buffer[j] = '\0';

    /* 跳过行结束符 (兼容 \r\n) */
    if (end < s_rx_line_len)
    {
        if (s_rx_line[end] == '\r' && (uint8_t)(end + 1) < s_rx_line_len &&
            s_rx_line[end + 1] == '\n') end++;
        end++;
    }

    /* 剩余数据前移, 留待下一次取行 */
    while (end < s_rx_line_len) s_rx_line[k++] = s_rx_line[end++];
    s_rx_line_len = k;

    return (uint8_t)(j > 0 ? 2 : 1);
}

uint8_t UART_GetCommand(char* cmd_buffer)
{
    while (1)
    {
        uint8_t state = UART_TakeLine(cmd_buffer);
        if (state == 2) return 1;   /* 取到命令 */
        if (state == 1) continue;   /* 空行, 继续取下一行 */
        if (!UART_LoadFrame()) return 0;  /* 行未结束, 追加下一帧 */
    }
}
