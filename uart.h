#ifndef __UART_H
#define __UART_H

#include "def.h"

// --- Configuration ---
// CLI 主机接口: 两路功能完全一致, 输出广播到两路, 任一入口都能操作整机
//   第 1 路 USB-CDC:  P3.0=D- / P3.1=D+   (AI_USB.H + stc_usb_cdc_32g.LIB)
//                     电脑识别为 "USB-CDC 虚拟串口", 不需要 USB-TTL 模块
//   第 2 路 UART4:    P0.2=RXD4 / P0.3=TXD4 (普通 TTL 串口, 115200 8N1)
#define UART_BAUDRATE       "115200bps"   // 仅 UART4 使用; USB-CDC 与波特率无关
#define RX_BUFFER_SIZE      128           // 单条命令最大长度
#define UART_TX_CHUNK       48            // UART4 单次 DMA 分块 (需 < 库缓冲 60)
#define USB_PACKET_SIZE     64            // USB-CDC 端点缓冲/单包最大字节数
#define USB_TX_WAIT         20000u        // 等待 USB 上一包发完的轮询上限 (防死等)
#define UART_PORT_COUNT     2             // CLI 端口数量 (USB-CDC + UART4)
#define UART_PORT_USB       0             // 端口索引: USB-CDC 虚拟串口
#define UART_PORT_4         1             // 端口索引: UART4

// Keil 软件仿真器没有 USB 外设模型, usb_init() 会卡在 USB 寄存器忙位
// (USBADR 的 bit7 永远不会清零)。用仿真器调试本工程前, 取消下面这行
// 注释并重新编译即可跳过 USB 初始化; 烧录真机的固件必须保持注释状态。
//#define UART_SIM_SKIP_USB

// --- Function Prototypes ---
void UART_UsbInit(void);                                 // USB-CDC 初始化, 必须在 TFPU_Init() 之前
void UART_Init(void);                                    // UART4 初始化, 在 TFPU_Init() 之后
void UART_SendString(const char* str);                   // 广播到两路 CLI 端口
uint8_t UART_GetCommand(uint8_t port, char* cmd_buffer); // port: UART_PORT_USB / UART_PORT_4

#endif
