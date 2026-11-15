#include "DHT11.h"
#include "delay.h"
#include "set_io.h"

// 数据引脚模式 (保留原接口形态)
#define DHT_MODE_OUT   1
#define DHT_MODE_IN    0

// 内部函数声明
static void DHT_GPIO_Init(uint8_t Mode);
static uint8_t DHT_Start(void);
static uint8_t DHT_Get_Byte_Data(void);

/**
 * @brief  配置DHT11数据引脚的模式
 * @note   AI8051U 使用准双向口 (pu_mode) 即可同时实现 "拉低输出" 和
 *         "释放/输入上拉读取" 两种操作, 无需反复切换引脚模式。
 * @param  Mode: DHT_MODE_OUT 或 DHT_MODE_IN (兼容原代码结构)
 */
static void DHT_GPIO_Init(uint8_t Mode)
{
    set_io_mode(pu_mode, Pin20, Pin_End); // P2.0 准双向口

    if (Mode == DHT_MODE_OUT)
    {
        P20 = 1;   // 总线空闲状态: 高电平
    }
}

/**
 * @brief  主机发送起始信号
 * @retval 1: 成功, 0: DHT11无响应
 */
static uint8_t DHT_Start(void)
{
    uint16_t timeout = 1000;

    // 主机拉低总线至少18ms
    DHT_GPIO_Init(DHT_MODE_OUT);
    P20 = 0;
    delay_ms(20); // 延时20ms

    // 主机释放总线(拉高)并等待DHT11响应
    P20 = 1;
    delay_us(30);

    // 等待DHT11拉低总线 (响应信号)
    while (P20 && timeout) {
        timeout--;
        delay_us(1);
    }
    if (timeout == 0) return 0;

    // 等待DHT11拉高总线 (80us低电平结束)
    timeout = 1000;
    while (!P20 && timeout) {
        timeout--;
        delay_us(1);
    }
    if (timeout == 0) return 0;

    // 等待DHT11再次拉低总线 (80us高电平结束，准备开始发送数据)
    timeout = 1000;
    while (P20 && timeout) {
        timeout--;
        delay_us(1);
    }
    if (timeout == 0) return 0;

    return 1;
}

/**
 * @brief  接收一个字节的数据
 * @retval 接收到的8位数据
 */
static uint8_t DHT_Get_Byte_Data(void)
{
    uint8_t i;
    uint8_t data_byte = 0;
    uint16_t timeout;

    for (i = 0; i < 8; i++) {
        // 等待50us的低电平结束
        timeout = 1000;
        while (!P20 && timeout) {
            timeout--;
            delay_us(1);
        }
        if (timeout == 0) return 0;

        // 延时30-40us，然后检测电平
        delay_us(40);

        data_byte <<= 1; // 左移一位，准备接收新bit

        if (P20) {
            // 如果是高电平，说明是数据'1'
            data_byte |= 0x01;

            // 等待高电平结束
            timeout = 1000;
            while (P20 && timeout) {
                timeout--;
                delay_us(1);
            }
            if (timeout == 0) return 0;
        }
        // 如果是低电平，说明是数据'0'，data_byte的最低位已经是0，无需操作
    }
    return data_byte;
}

/**
 * @brief  获取完整的温湿度数据
 * @param  buffer: 长度为5的uint8_t数组
 * @retval 1: 成功, 0: 失败
 */
uint8_t DHT_Get_Temp_Humi_Data(uint8_t buffer[])
{
    // 清空缓冲区
    buffer[0] = buffer[1] = buffer[2] = buffer[3] = buffer[4] = 0;

    if (DHT_Start()) {
        buffer[0] = DHT_Get_Byte_Data(); // 湿度整数
        buffer[1] = DHT_Get_Byte_Data(); // 湿度小数
        buffer[2] = DHT_Get_Byte_Data(); // 温度整数
        buffer[3] = DHT_Get_Byte_Data(); // 温度小数
        buffer[4] = DHT_Get_Byte_Data(); // 校验和

        // 读取结束后，将引脚设置回输出高电平状态，释放总线
        P20 = 1;

        // 校验数据
        if ((uint8_t)(buffer[0] + buffer[1] + buffer[2] + buffer[3]) == buffer[4]) {
            return 1; // 校验成功
        } else {
            return 0; // 校验失败
        }
    } else {
        // 如果起始失败，也要释放总线
        P20 = 1;
        return 0; // 起始失败
    }
}
