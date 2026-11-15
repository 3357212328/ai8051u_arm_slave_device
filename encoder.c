#include "encoder.h"
#include "delay.h"
#include "set_io.h"

// ======================= 引脚定义 (AI8051U) =======================
// 旋转编码器 (软件正交解码, 1ms中断采样):
//   A 相: P4.2
//   B 相: P4.3
//   按键: P4.4
// ==================================================================
#define ENCODER_A_PIN   P42
#define ENCODER_B_PIN   P43
#define ENCODER_KEY_PIN P44

// 正交解码状态表: 索引 = (前一次状态<<2) | 当前状态, 值为计数增量
// 状态编码: bit0=A, bit1=B
static const int8_t code enc_state_table[16] = {
     0, -1,  1,  0,      // 00->00,00->01,00->10,00->11
     1,  0,  0, -1,      // 01->00,01->01,01->10,01->11
    -1,  0,  0,  1,      // 10->00,10->01,10->10,10->11
     0,  1, -1,  0       // 11->00,11->01,11->10,11->11
};

static uint8_t edata enc_prev_state = 0;     // 上次采样状态
static volatile int16_t edata enc_count = 0; // 累计计数 (ISR 与主循环共享)

void Encoder_Init(void)
{
    // A/B/按键 均为高阻输入 + 内部上拉
    set_io_mode(hz_mode, Pin42, Pin43, Pin44, Pin_End);
    set_io_mode(en_pur, Pin42, Pin43, Pin44, Pin_End);

    enc_prev_state = (uint8_t)((ENCODER_A_PIN ? 1 : 0) | (ENCODER_B_PIN ? 2 : 0));
}

/**
 * @brief  1ms 定时器中断中调用: 采样 A/B 相并进行正交解码
 */
void Encoder_Tick(void)
{
    uint8_t cur = (uint8_t)((ENCODER_A_PIN ? 1 : 0) | (ENCODER_B_PIN ? 2 : 0));
    enc_count += enc_state_table[(enc_prev_state << 2) | cur];
    enc_prev_state = cur;
}

/**
 * @brief  读取编码器计数并清零 (每次读取后归零)
 * @note   短暂关闭总中断, 防止与 1ms ISR 的递增产生竞态
 */
int16_t Encoder_Get_Count(void)
{
    int16_t c;
    EA = 0;
    c = enc_count;
    enc_count = 0;
    EA = 1;
    return c;
}

/**
 * @brief  编码器按键扫描 (带消抖, 边沿触发)
 */
uint8_t Encoder_Key_Scan(void)
{
    static uint8_t last_state = 1; // 1: released, 0: pressed
    uint8_t current_state = (uint8_t)(ENCODER_KEY_PIN ? 1 : 0);

    if (current_state == 0 && last_state == 1) // Key pressed event
    {
        delay_ms(20); // Debounce
        if (ENCODER_KEY_PIN == 0)
        {
            last_state = 0;
            return 1; // Return 1 on press
        }
    }
    else if (current_state == 1)
    {
        last_state = 1; // Key released
    }

    return 0; // No new press event
}
