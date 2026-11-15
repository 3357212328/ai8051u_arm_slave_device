#ifndef __DELAY_H
#define __DELAY_H

#include "def.h"     // uint8_t/uint16_t 等类型定义 (直接引用系统头文件)

/**
 * @brief  微秒级延时 (阻塞)
 * @param  nus: 延时微秒数
 * @note   按 CPU 主频 40MHz (内部高速IRC) 校准，误差约 ±10%。
 *         若修改主频，需同步调整 delay.c 中 NOP 数量。
 */
void delay_us(uint16_t nus);

/**
 * @brief  毫秒级延时 (阻塞)
 * @param  nms: 延时毫秒数
 */
void delay_ms(uint16_t nms);

#endif // __DELAY_H
