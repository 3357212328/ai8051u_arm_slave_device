#ifndef __BUZZER_H
#define __BUZZER_H

#include "def.h"

// 定义蜂鸣器控制引脚 (AI8051U): P2.1, 低电平触发
#define BUZZER_PIN  P21

/**
 * @brief 初始化蜂鸣器控制引脚 (推挽输出, 默认关闭)
 */
void Buzzer_Init(void);

/**
 * @brief 打开蜂鸣器 (引脚拉低)
 */
void Buzzer_On(void);

/**
 * @brief 关闭蜂鸣器 (引脚拉高)
 */
void Buzzer_Off(void);

/**
 * @brief 发出一次短促的蜂鸣声
 * @param ms: 蜂鸣声的持续时间 (毫秒)
 */
void Buzzer_Beep(uint16_t ms);

#endif // __BUZZER_H
