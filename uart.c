#include "uart.h"
#include "set_uart.h"
#include "set_io.h"
#include "AI_USB.H"

// ====================================================================
// CLI 主机接口 (USB-CDC + UART4) — 两路功能完全一致
// --------------------------------------------------------------------
// 第 1 路 USB-CDC: P3.0=D- / P3.1=D+, 由 STC USB 库 (stc_usb_cdc_32g.LIB)
//                  实现, 电脑端识别为虚拟串口 (Win10 1903 以上免驱)。
//                  P3.0/P3.1 由 USB 收发器直接驱动, 不能再配成 UART1 复用。
//                  发送: 查询 bUsbInBusy, 按 64 字节分块调用 USB_SendData;
//                  接收: 查询 bUsbOutReady, 从 UsbOutBuffer[OutNumber] 取数,
//                        取完调用 usb_OUT_done() 释放 OUT 端点。
// 第 2 路 UART4: P0.2=RXD4 / P0.3=TXD4, 115200 8N1。
//
// 发送: UART4 的库发送走 DMA, 发送忙时会丢数据, 因此按块等待后广播。
// 接收: 两路都只往各自的行缓冲追加数据, 收到 \r/\n 才交给 CLI 解析。
// ====================================================================

/**
 * @brief  第 1 路 CLI: USB-CDC 虚拟串口初始化 (P3.0=D- / P3.1=D+)
 * @note   必须在 TFPU_Init() 之前调用: usb_init() 内部会写
 *         `USBCLK = 0x00`, 把 TFPU 用的 PLL 倍频位清零。
 *         USB 时钟必须用专门的 48MHz IRC: 本工程 PLL 给了 TFPU (120MHz),
 *         PLLCLK/2 得不到 48MHz (见 AI8051U 手册 8.5.1 USBCKS 表)。
 */
void UART_UsbInit(void)
{
    IRCBAND |= 0x80;      /* USBCKS  = 1 */
    IRCBAND &= ~0x40;     /* USBCKS2 = 0  ->  USB 时钟 = IRC48M */
    /* usb_init() 会启动 IRC48M 并等待稳定, 配置端点并使能 USB 中断;
       同时把 P3.0/P3.1 改成高阻输入, 交给 USB 收发器 */
#ifndef UART_SIM_SKIP_USB
    usb_init();
#endif
}

/**
 * @brief  第 2 路 CLI: UART4 初始化 (P0.2=RXD4 / P0.3=TXD4)
 * @note   库的波特率计算用到浮点, 因此必须在 TFPU_Init() 之后调用。
 */
void UART_Init(void)
{
    /* TXD4 配成推挽输出, 提高 115200 下的上升沿质量; RXD4 保持准双向口 */
    set_io_mode(pp_mode, Pin03, Pin_End);
    set_uart_mode(Uart4, UART_BAUDRATE, Uart4_P02_3, Uart_End);
    /* 同理: 库不打开 IE2 的 ES4 位, 必须手动打开, 否则收不到数据 */
    ES4 = 1;
}

/**
 * @brief  USB-CDC 发送 (按 64 字节分块)
 * @note   USB 未枚举完成 (DeviceState != CONFIGURED) 或上一包长时间发不出去时
 *         放弃本次输出: CLI 在主循环里轮询, 不能在这里无限等待主机。
 */
static void UART_UsbSend(const char* str)
{
    const char* p = str;
    uint16_t n;
    uint16_t wait;

    if (p == 0) return;

    while (*p)
    {
        n = 0;
        while (p[n] != '\0' && n < (uint16_t)USB_PACKET_SIZE) n++;

        if (DeviceState != DEVSTATE_CONFIGURED) return;   /* 主机还没枚举完成 */

        wait = USB_TX_WAIT;
        while (bUsbInBusy && --wait) {}                   /* 等上一包发送完成 */
        if (bUsbInBusy) return;                           /* 超时: 丢弃本包, 不阻塞 */

        USB_SendData((BYTE *)p, (int)n);
        p += n;
    }
}

/**
 * @brief  单路发送字符串 (自动分块, 等待上一次 DMA 完成)
 * @note   UART4 的库发送缓冲只有 60 字节, 因此分块统一按
 *         UART_TX_CHUNK(48) 切分。
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
 * @brief  发送字符串 (同时广播到 USB-CDC + UART4)
 * @note   两路共用同一份 CLI 输出, 保证任一主机接口看到的内容一致
 */
void UART_SendString(const char* str)
{
    UART_UsbSend(str);
    UART_SendPort(Uart4, str);
}

/* 每路独立的行缓冲, 跨 USB 包 / DMA 帧累积 */
static char xdata s_rx_line[UART_PORT_COUNT][RX_BUFFER_SIZE];
static uint8_t xdata s_rx_line_len[UART_PORT_COUNT] = {0, 0};

/**
 * @brief  取一帧新数据追加到对应端口的行缓冲
 * @retval 1=取到了新数据, 0=暂时没有新数据
 */
static uint8_t UART_LoadFrame(uint8_t port)
{
    int len;
    int max_len;
    int i;

    if (port == UART_PORT_USB)
    {
        /* 查询方式: 主机把一包数据放进 UsbOutBuffer 后 bUsbOutReady=1 */
        if (!bUsbOutReady) return 0;

        len = (int)OutNumber;
        if (len > (int)USB_PACKET_SIZE) len = (int)USB_PACKET_SIZE;
        for (i = 0; i < len && s_rx_line_len[port] < (RX_BUFFER_SIZE - 1); i++)
        {
            s_rx_line[port][s_rx_line_len[port]++] = (char)UsbOutBuffer[i];
        }
        usb_OUT_done();                 /* 释放 OUT 端点, 允许接收下一包 */
        return 1;
    }

    /* --- UART4: 库的 DMA 空闲超时中断把数据放到 _uart4_rx_buff[] --- */
    max_len = (int)Uart4_Rx;

    // 关中断后再查询/拷贝, 避免新帧的 DMA 超时中断覆盖 rx_cnt/缓冲区
    EA = 0;
    if (!get_uart_state(Uart4))
    {
        EA = 1;
        return 0;
    }

    len = rx_cnt[Uart4];
    if (len > max_len - 1) len = max_len - 1;
    for (i = 0; i < len && s_rx_line_len[port] < (RX_BUFFER_SIZE - 1); i++)
    {
        s_rx_line[port][s_rx_line_len[port]++] = _uart4_rx_buff[i];
    }
    EA = 1;
    return 1;
}

/**
 * @brief  从指定端口的行缓冲取出一行
 * @retval 2=取到非空命令, 1=消耗了一个空行, 0=还没有行结束符
 * @note   USB-CDC 一包最多 64 字节, UART4 的 DMA 空闲超时按"字节间隔"触发,
 *         一帧可能只有半个命令行, 也可能包含多行。这里用行缓冲跨帧累积,
 *         只有收到 \r/\n 才返回一条命令。
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
