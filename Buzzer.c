#include "Buzzer.h"
#include "delay.h"
#include "set_io.h"

/**
 * @brief 初始化蜂鸣器控制引脚 (推挽输出, 初始为高电平关闭)
 */
void Buzzer_Init(void)
{
    set_io_mode(pp_mode, Pin21, Pin_End);
    Buzzer_Off();
}

/**
 * @brief 打开蜂鸣器 (引脚拉低)
 */
void Buzzer_On(void)
{
    BUZZER_PIN = 0;
}

/**
 * @brief 关闭蜂鸣器 (引脚拉高)
 */
void Buzzer_Off(void)
{
    BUZZER_PIN = 1;
}

/**
* @brief 发出一次短促的蜂鸣声
* @param ms: 蜂鸣声的持续时间 (毫秒)
*/
void Buzzer_Beep(uint16_t ms)
{
    Buzzer_On();
    delay_ms(ms);
    Buzzer_Off();
}
