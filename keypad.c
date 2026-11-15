#include "keypad.h"
#include "delay.h"
#include "set_io.h"
#include "intrins.h"

// ======================= 引脚定义 (AI8051U) =======================
// 4x4 矩阵键盘:
//   行 (输出): P1.0 ~ P1.3
//   列 (输入, 上拉): P1.4 ~ P1.7
// ==================================================================
#define KEYPAD_ROW_PORT P1
#define KEYPAD_COL_PORT P1

// 内部辅助函数的静态前向声明
static uint8_t Keypad_Scan_Raw(void);

void Keypad_Init(void)
{
    // 行为推挽输出
    set_io_mode(pp_mode, Pin10, Pin11, Pin12, Pin13, Pin_End);
    // 列为高阻输入 + 内部上拉
    set_io_mode(hz_mode, Pin14, Pin15, Pin16, Pin17, Pin_End);
    set_io_mode(en_pur, Pin14, Pin15, Pin16, Pin17, Pin_End);

    // 初始时将所有行都设置为高电平
    KEYPAD_ROW_PORT = 0xFF;
}

uint8_t Keypad_Scan(void)
{
    static uint8_t last_key_state = 0; // 0: released, 1-16: pressed
    uint8_t current_key = Keypad_Scan_Raw();

    if (current_key != 0 && last_key_state == 0)
    {
        delay_ms(20);
        if (current_key == Keypad_Scan_Raw())
        {
            last_key_state = current_key;
            return current_key;
        }
    }
    else if (current_key == 0)
    {
        last_key_state = 0;
    }

    return 0;
}

uint8_t Keypad_Scan_Raw(void)
{
    uint8_t r, c;
    uint8_t key_val = 0;
    // 行驱动低电平的掩码: 依次将 P1.0~P1.3 拉低
    const uint8_t code row_low[4] = {0xFE, 0xFD, 0xFB, 0xF7};

    for (r = 0; r < 4; r++)
    {
        KEYPAD_ROW_PORT = 0xFF;       // 所有行高电平
        KEYPAD_ROW_PORT = row_low[r]; // 将第 r 行拉低
        _nop_(); _nop_(); _nop_(); _nop_(); // 等待信号稳定

        // 读取列状态 (P1.4 ~ P1.7)
        {
            uint8_t cols = (uint8_t)((KEYPAD_COL_PORT >> 4) & 0x0F);
            for (c = 0; c < 4; c++)
            {
                if (!(cols & (1 << c))) // 该列为低电平 => 按键按下
                {
                    key_val = (uint8_t)(r * 4 + c + 1);
                    break;
                }
            }
        }
        if (key_val != 0) break;
    }
    KEYPAD_ROW_PORT = 0xFF;
    return key_val;
}
