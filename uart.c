#include "uart.h"
#include "set_uart.h"
#include "set_io.h"

// ====================================================================
// CLI 串口 (UART1 + UART4) — 基于 AI8051U 库函数 set_uart
// --------------------------------------------------------------------
// UART1: P3.0=RXD  / P3.1=TXD  (板载 USB 转串口)
// UART4: P0.2=RXD4 / P0.3=TXD4 (第二路主机接口)
// 两路接收独立维护行缓冲, 命令处理完全相同 (见 main.c)。
//
// 发送: 库的 uart_printf 走 DMA, 发送忙时会丢弃数据, 因此
//       UART_SendString 按块等待发送完成, 再把同一份内容广播到两路。
// 接收: 库的 DMA 空闲超时中断触发 get_uart_state, 数据在
//       _uart1_rx_buff[] / _uart4_rx_buff[] 中 (含 \r\n),
//       这里剥离行结束符。
// ====================================================================

void UART_Init(void)
{
    /* --- 第 1 路: UART1 (P3.0/P3.1) --- */
    set_uart_mode(Uart1, UART_BAUDRATE, Uart_End);
    /* set_uart 库只配置了 UR1TOCR 的 ENTO/ENTOI 和 DMA，
       但没有打开 CPU 级的串口1中断允许位 ES。
       不开 ES，接收超时中断(向量4)永远不会进入，
       uart_flag/rx_cnt 不会更新，CLI 将永远收不到命令。 */
    ES = 1;

    /* --- 第 2 路: UART4 (P0.2=RXD4 / P0.3=TXD4), 与 UART1 完全一致 --- */
    /* TXD4 配成推挽输出, 提高 115200 下的上升沿质量; RXD4 保持准双向口 */
    set_io_mode(pp_mode, Pin03, Pin_End);
    set_uart_mode(Uart4, UART_BAUDRATE, Uart4_P02_3, Uart_End);
    /* 同理: 库不打开 IE2 的 ES4 位, 必须手动打开, 否则收不到数据 */
    ES4 = 1;
}

/**
 * @brief  单路发送字符串 (自动分块, 等待上一次 DMA 完成)
 * @note   UART4 的库发送缓冲只有 60 字节, 因此分块统一按
 *         UART_TX_CHUNK(48) 切分, 两路共用同一套逻辑。
 */
static void UART_SendPort(uart_name port, const char* str)
{
    char chunk[UART_TX_CHUNK];

    if (str == 0) return;

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
        while (*((volatile char *)&tx_state[port])) {}

        uart_printf(port, "%s", chunk);
    }
}

/**
 * @brief  发送字符串 (同时广播到 UART1 + UART4)
 * @note   两路共用同一份 CLI 输出, 保证任一主机接口看到的内容一致
 */
void UART_SendString(const char* str)
{
    UART_SendPort(Uart1, str);
    UART_SendPort(Uart4, str);
}

/* 每路独立的行缓冲, 跨 DMA 帧累积 */
static char xdata s_rx_line[UART_PORT_COUNT][RX_BUFFER_SIZE];
static uint8_t xdata s_rx_line_len[UART_PORT_COUNT] = {0, 0};

static uart_name UART_HwPort(uint8_t port)
{
    return (port == UART_PORT_4) ? Uart4 : Uart1;
}

/* 把 DMA 收到的一帧追加到对应行缓冲; 返回 1=读到了新帧 */
static uint8_t UART_LoadFrame(uint8_t port)
{
    char xdata *rx_buff;
    uart_name hw = UART_HwPort(port);
    int max_len;
    int len;
    int i;

    // 关中断后再查询/拷贝, 避免新帧的 DMA 超时中断覆盖 rx_cnt/缓冲区
    EA = 0;
    if (!get_uart_state(hw))
    {
        EA = 1;
        return 0;
    }

    max_len = (port == UART_PORT_4) ? (int)Uart4_Rx : (int)Uart1_Rx;
    rx_buff = (port == UART_PORT_4) ? _uart4_rx_buff : _uart1_rx_buff;

    len = rx_cnt[hw];
    if (len > max_len - 1) len = max_len - 1;
    for (i = 0; i < len && s_rx_line_len[port] < (RX_BUFFER_SIZE - 1); i++)
    {
        s_rx_line[port][s_rx_line_len[port]++] = rx_buff[i];
    }
    EA = 1;
    return 1;
}

/**
 * @brief  从指定端口的行缓冲取出一行
 * @retval 2=取到非空命令, 1=消耗了一个空行, 0=还没有行结束符
 * @note   库的 DMA 空闲超时按"字节间隔"触发, 打字/分片发送时一帧可能
 *         只有半个命令行, 连续发送时一帧又可能包含多行。这里用行缓冲
 *         跨帧累积, 只有收到 \r/\n 才返回一条命令。
 */
static uint8_t UART_TakeLine(uint8_t port, char* cmd_buffer)
{
    uint8_t i;
    uint8_t j = 0;
    uint8_t end = 0;
    uint8_t k = 0;

    while (end < s_rx_line_len[port] &&
           s_rx_line[port][end] != '\r' && s_rx_line[port][end] != '\n') end++;

    /* 没收到行结束符: 继续等后续帧 (缓冲满则强制返回) */
    if (end == s_rx_line_len[port] && s_rx_line_len[port] < (RX_BUFFER_SIZE - 1)) return 0;

    for (i = 0; i < end; i++)
    {
        char c = s_rx_line[port][i];
        if (c == '\b' || c == 0x7F) { if (j > 0) j--; continue; }  /* 退格 (BS/DEL) */
        cmd_buffer[j++] = c;
    }
    cmd_buffer[j] = '\0';

    /* 跳过行结束符 (兼容 \r\n) */
    if (end < s_rx_line_len[port])
    {
        if (s_rx_line[port][end] == '\r' && (uint8_t)(end + 1) < s_rx_line_len[port] &&
            s_rx_line[port][end + 1] == '\n') end++;
        end++;
    }

    /* 剩余数据前移, 留待下一次取行 */
    while (end < s_rx_line_len[port]) s_rx_line[port][k++] = s_rx_line[port][end++];
    s_rx_line_len[port] = k;

    return (uint8_t)(j > 0 ? 2 : 1);
}

/**
 * @brief  从指定端口获取一条完整命令 (收到后清除标志)
 * @retval 1=有命令, 0=无命令
 */
uint8_t UART_GetCommand(uint8_t port, char* cmd_buffer)
{
    if (port >= UART_PORT_COUNT) return 0;

    while (1)
    {
        uint8_t state = UART_TakeLine(port, cmd_buffer);
        if (state == 2) return 1;              /* 取到命令 */
        if (state == 1) continue;              /* 空行, 继续取下一行 */
        if (!UART_LoadFrame(port)) return 0;   /* 行未结束, 追加下一帧 */
    }
}
