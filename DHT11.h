#ifndef __DHT11_H
#define __DHT11_H

#include "def.h"

// DHT11 数据引脚 (AI8051U): P2.0, 准双向口模式 (单总线)
#define DHT_PIN_P20     P20

/**
 * @brief  获取温湿度数据
 * @param  buffer: 长度为5的uint8_t数组，用于存放数据
 *         - buffer[0]: 湿度整数部分
 *         - buffer[1]: 湿度小数部分
 *         - buffer[2]: 温度整数部分
 *         - buffer[3]: 温度小数部分
 *         - buffer[4]: 校验和
 * @retval 1: 成功, 0: 失败 (校验错误或无响应)
 */
uint8_t DHT_Get_Temp_Humi_Data(uint8_t buffer[]);

#endif
