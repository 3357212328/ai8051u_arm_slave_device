#ifndef __UART_H
#define __UART_H

#include "def.h"

// --- Configuration ---
// CLI 串口: 使用 AI8051U 库函数 set_uart 库, 两路主机接口功能完全一致
//   第 1 路 UART1: P3.0=RXD / P3.1=TXD (板载 USB 转串口)
//   第 2 路 UART4: P0.2=RXD4 / P0.3=TXD4 (第二路主机串口)
#define UART_BAUDRATE       "115200bps"   // 可改为 "9600bps" 等
#define RX_BUFFER_SIZE      128           // 单条命令最大长度
#define UART_TX_CHUNK       48            // 单次 DMA 分块长度 (需 < UART4 库缓冲60)
#define UART_PORT_COUNT     2             // CLI 串口数量 (UART1 + UART4)
#define UART_PORT_1         0             // 端口索引: UART1
#define UART_PORT_4         1             // 端口索引: UART4

// --- Function Prototypes ---
void UART_Init(void);
void UART_SendString(const char* str);                  // 广播到两路 CLI 串口
uint8_t UART_GetCommand(uint8_t port, char* cmd_buffer); // port: UART_PORT_1 / UART_PORT_4

#endif
