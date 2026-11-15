#ifndef __UART_H
#define __UART_H

#include "def.h"

// --- Configuration ---
// CLI 串口: 使用 AI8051U 库函数 set_uart 库, UART1 (默认 P3.0=RXD, P3.1=TXD)
#define UART_BAUDRATE       "115200bps"   // 可改为 "9600bps" 等
#define RX_BUFFER_SIZE      128           // 单条命令最大长度
#define UART_TX_CHUNK       100           // 单次 DMA 发送的最大分块长度 (需 < 库缓冲120)

// --- Function Prototypes ---
void UART_Init(void);
void UART_SendString(const char* str);
uint8_t UART_GetCommand(char* cmd_buffer);

#endif
