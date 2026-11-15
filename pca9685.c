#pragma maxargs (200) //拓展变长数组参数解析长度

#include "pca9685.h"
#include "set_i2c.h"
#include "set_io.h"
#include "delay.h"

// 使用 AI8051U 库函数 set_i2c 完成 PCA9685 的寄存器读写
// I2C0 引脚: 默认 I2c_P24_3 (SCL=P2.4, SDA=P2.3)
#define PCA9685_I2C_TASK   0       // 占用 I2C 任务槽 0
#define I2C_TIMEOUT        20000   // 总线等待超时 (~10-20ms), 正常一次事务仅 ~0.2ms

static uint8_t edata s_rd_val = 0; // 单字节读缓存 (放 edata 加快访问)

/**
 * @brief  等待 I2C 任务完成 (带超时保护)
 */
static void i2c_wait_done(void)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (!get_i2c_state(I2c0, PCA9685_I2C_TASK))
    {
        if (--timeout == 0)
        {
            /* 总线异常: 复位 I2C 状态机, 否则任务永远停在 State_Running,
               后续每次写都会白等到超时, 主循环被拖死 */
            set_i2c_recover(I2c0);
            return;
        }
    }
}

// 封装I2C写操作 (注意: C251 中 data 是保留字, 参数命名为 dat)
static void pca9685_write(uint8_t reg, uint8_t dat)
{
    set_i2c_cmd(I2c0, PCA9685_I2C_TASK, S_Tx_Rack, PCA9685_ADDRESS,
                Tx_Rack, reg, Tx_Rack, dat, Stop, Cmd_End);
    i2c_wait_done();
}

// 封装I2C读操作 (MODE1 回读, 供 SetPWMFreq 使用)
static uint8_t pca9685_read(uint8_t reg)
{
    s_rd_val = 0;
    set_i2c_cmd(I2c0, PCA9685_I2C_TASK, S_Tx_Rack, PCA9685_ADDRESS, Tx_Rack, reg,
                S_Tx_Rack, PCA9685_ADDRESS | 1, Rx_Tnak, (char *)&s_rd_val, Stop, Cmd_End);
    i2c_wait_done();
    return s_rd_val;
}

/**
 * @brief  初始化PCA9685
 */
void PCA9685_Init(void)
{
    set_i2c_mode(I2c0, "400khz", I2c_P24_3, I2c_End); // 初始化I2C总线 (400kHz)
    pca9685_write(PCA9685_MODE1, 0x00);               // 复位PCA9685
    PCA9685_SetPWMFreq(50);                            // 50Hz, 适合舵机
}

/**
 * @brief  设置PWM频率
 * @param  freq: 频率值 (24Hz - 1526Hz)
 * @note   使用纯整数运算, 避免浮点库依赖 (可用TFPU加速时亦可改用浮点)
 */
void PCA9685_SetPWMFreq(uint16_t freq)
{
    uint8_t prescale, oldmode, newmode;
    uint32_t div = 4096UL * (uint32_t)freq;

    // prescale = round(25000000 / (4096 * freq)) - 1
    uint32_t ps = 25000000UL / div;                    // 向下取整
    if ((25000000UL % div) >= (div >> 1)) ps++;        // 四舍五入
    prescale = (uint8_t)(ps - 1);

    // 根据数据手册, 设置频率前需要先让芯片进入睡眠模式
    oldmode = pca9685_read(PCA9685_MODE1);
    newmode = (oldmode & 0x7F) | 0x10; // 设置SLEEP位
    pca9685_write(PCA9685_MODE1, newmode);      // 进入睡眠模式
    pca9685_write(PCA9685_PRESCALE, prescale);  // 设置预分频值
    pca9685_write(PCA9685_MODE1, oldmode);      // 恢复原来的模式
    delay_ms(5);                                // 等待振荡器稳定
    // 写入 0xa1 自动递增模式，方便连续写入
    pca9685_write(PCA9685_MODE1, oldmode | 0xa1);
}

/**
 * @brief  设置单个通道的PWM值
 * @param  channel: 通道号 (0-15)
 * @param  on: PWM上升沿时刻 (0-4095)
 * @param  off: PWM下降沿时刻 (0-4095)
 */
void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    pca9685_write((uint8_t)(LED0_ON_L + 4 * channel), (uint8_t)(on & 0xFF));
    pca9685_write((uint8_t)(LED0_ON_L + 1 + 4 * channel), (uint8_t)(on >> 8));
    pca9685_write((uint8_t)(LED0_ON_L + 2 + 4 * channel), (uint8_t)(off & 0xFF));
    pca9685_write((uint8_t)(LED0_ON_L + 3 + 4 * channel), (uint8_t)(off >> 8));
}

/**
 * @brief  设置舵机脉冲宽度
 * @param  channel: 通道号 (0-15)
 * @param  pulse: 脉冲宽度，单位us (通常在 500-2500 之间)
 */
void PCA9685_SetServoPulse(uint8_t channel, uint16_t pulse)
{
    uint16_t off_value;
    // 50Hz下, 周期为20000us, 分辨率为 20000/4096 ≈ 4.88us/bit
    off_value = (uint16_t)(((uint32_t)pulse * 4096UL) / 20000UL);
    PCA9685_SetPWM(channel, 0, off_value);
}

/**
 * @brief  设置舵机角度 (更友好的接口)
 * @param  channel: 通道号 (0-15)
 * @param  angle: 角度 (0-180)
 * @note   500-2500us脉宽对应0-180度
 */
void PCA9685_SetServoAngle(uint8_t channel, uint8_t angle)
{
    uint16_t pulse;
    if (angle > 180) angle = 180;

    // 0-180度线性映射到 570-2650us
    pulse = (uint16_t)(570 + ((uint32_t)(2650 - 570) * angle) / 180);
    PCA9685_SetServoPulse(channel, pulse);
}

/**
 * @brief  平滑设置舵机角度，逐步过渡避免抖动和冲击
 * @param  channel: 通道号 (0-15)
 * @param  current_angle: 当前角度 (0-180)
 * @param  target_angle: 目标角度 (0-180)
 * @param  step_delay_ms: 每步之间的延时(ms)，典型值10-20ms
 */
void PCA9685_SetServoAngleSmooth(uint8_t channel, uint8_t current_angle, uint8_t target_angle, uint16_t step_delay_ms)
{
    int16_t current = (int16_t)current_angle;
    int16_t target = (int16_t)target_angle;
    int8_t step_dir;

    if (target > 180) target = 180;
    if (current > 180) current = 180;

    if (current == target) return;

    step_dir = (target > current) ? 1 : -1;

    while (current != target) {
        current += step_dir;
        PCA9685_SetServoAngle(channel, (uint8_t)current);
        delay_ms(step_delay_ms);
    }
}
