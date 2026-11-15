#ifndef __PCA9685_H
#define __PCA9685_H

#include "def.h"

// PCA9685 I2C 地址 (A0-A5接地时), 7位地址 0x40 左移一位
#define PCA9685_ADDRESS 0x80

// PCA9685 寄存器地址
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06

// 函数声明
void PCA9685_Init(void);
void PCA9685_SetPWMFreq(uint16_t freq);
void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off);
void PCA9685_SetServoPulse(uint8_t channel, uint16_t pulse);
void PCA9685_SetServoAngle(uint8_t channel, uint8_t angle);
void PCA9685_SetServoAngleSmooth(uint8_t channel, uint8_t current_angle, uint8_t target_angle, uint16_t step_delay_ms);

#endif
