#ifndef __ENCODER_H
#define __ENCODER_H

#include "def.h"

void Encoder_Init(void);
int16_t Encoder_Get_Count(void);
uint8_t Encoder_Key_Scan(void);
void Encoder_Tick(void);   // 由 1ms 定时器中断调用, 软件正交解码采样

#endif
