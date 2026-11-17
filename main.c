#pragma maxargs (200) //拓展变长数组参数解析长度

#include "AI8051U.H"
#include "def.h"
#include "delay.h"
#include "flash_if.h"
#include "pca9685.h"
#include "OLED.h"
#include "keypad.h"
#include "encoder.h"
#include "DHT11.h"
#include "Buzzer.h"
#include "uart.h"
#include "set_io.h"
#include "set_timer.h"
#include "set_uart.h"
#include "set_i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ====================================================================
// 机械臂从机系统 (STM32 工程移植)
// --------------------------------------------------------------------
// 主频: 40MHz 内部高速IRC (HIRC)
// 加速: TFPU (浮点) + MDU32 (乘除) 硬件加速, 库文件已加入工程
// 快速变量: 速度敏感的全局变量使用 edata (片内 2KB 高速RAM)
// 大数组: 配置/显存等大块数据放 xdata (默认大模式)
// ====================================================================

// ====================================================================
//                       1. 宏定义和数据结构
// ====================================================================
#define ANGLE_STEP 5
#define DEFAULT_INITIAL_ANGLE 90
#define DEFAULT_TEMP_THRESHOLD 40
#define DEFAULT_HUMI_THRESHOLD 80
#define SMOOTH_POSE_DELAY_MS   20   /* 多舵机同步移动每步延时(ms) */
#define SMOOTH_SINGLE_DELAY_MS 10   /* 单舵机平滑移动每步延时(ms) */

/* 按键映射 — 4x4矩阵键盘 S1~S16 */
#define KEY_RESET       1    /* S1  舵机复位到初始位置 */
#define KEY_SYS_RESET   2    /* S2  系统重置(格式化) */
#define KEY_STOP        5    /* S5  停止播放 */
#define KEY_UP          7    /* S7  光标上移 / 上一个 */
#define KEY_DOWN        8    /* S8  光标下移 / 下一个 */
#define KEY_CONFIRM     11   /* S11 确认 / 进入 */
#define KEY_CANCEL      12   /* S12 返回 / 取消 / 暂停 */
#define KEY_INFO        3    /* S3  播放详情预览 */

/* 播放速度档位: 索引 -> 倍率 (×10) */
#define SPEED_COUNT        3
#define SPEED_INDEX_0_5X   0   /* 0.5x = 5 */
#define SPEED_INDEX_1X     1   /* 1.0x = 10 */
#define SPEED_INDEX_2X     2   /* 2.0x = 20 */
static const uint8_t s_speed_x10[SPEED_COUNT] = {5, 10, 20};

/* 配置数据版本幻数 */
#define CONFIG_MAGIC_NUM     0xA5A5B4B5UL
#define CONFIG_MAGIC_NUM_V1  0xA5A5B4B4UL

typedef enum {
    MODE_MAIN_MENU, MODE_MANUAL_CONTROL, MODE_SET_INITIAL_ANGLES,
    MODE_RECORDING_ANGLES, MODE_RECORDING_DELAY, MODE_PLAYBACK,
    MODE_MONITOR, MODE_SET_THRESHOLDS, MODE_SERVO_SETUP,
    MODE_EDIT_SEQUENCE, MODE_PLAY_SPEED,
} SystemMode;

typedef enum {
    PLAYBACK_STOPPED,
    PLAYBACK_RUNNING,
    PLAYBACK_PAUSED,
} PlaybackStatus;

// ====================================================================
//                          2. 全局变量
// ====================================================================
// 大配置结构 (3647B) -> 显式放 xdata (片外/外部RAM), 避免占用 edata
SystemConfig_t xdata g_config;

// ---- edata: 速度敏感的全局变量 (片内高速RAM) ----
SystemMode edata current_mode;
uint8_t edata g_temperature, g_humidity;

uint8_t edata servo_angles[MAX_SERVOS];
uint8_t edata active_servo_map[MAX_SERVOS];
uint8_t edata active_servo_count = 0;
uint8_t edata current_servo_map_index = 0;
uint8_t edata current_editing_step = 0;
uint8_t edata is_editing_step = 0;       /* 1=编辑已有步骤, 0=追加新步骤 */
uint8_t edata edit_seq_cursor = 0;       /* 编辑序列模式的光标 */
uint8_t edata edit_seq_page = 0;
uint8_t edata edit_seq_preview_servo = 0;/* 步骤列表中预览哪个舵机的角度 */

PlaybackStatus edata playback_status = PLAYBACK_STOPPED;
uint8_t edata playback_current_step = 0;
uint32_t edata playback_step_timer = 0;
uint8_t edata playback_detail_mode = 0;  /* 0=概览, 1=显示各舵机角度详情 */
uint8_t edata playback_detail_page = 0;  /* 详情模式分页 */

uint8_t edata menu_cursor_pos = 0;
uint8_t edata current_page = 0;

uint8_t edata threshold_cursor_pos = 0;
uint8_t edata speed_before_edit = 1;     /* 进入速度菜单前的设置, 用于取消恢复 */

uint8_t edata servo_setup_cursor_pos = 0;
uint8_t edata servo_setup_page = 0;
uint16_t edata temp_servo_mask = 0;

uint8_t edata sync_enabled = 0;
uint8_t edata alarm_active = 0;
volatile uint32_t edata g_sys_tick_count = 0;   /* 1ms 系统节拍 */

/* 非阻塞平滑移动状态 */
uint8_t edata s_smooth_active = 0;
uint8_t edata s_smooth_target[MAX_SERVOS];
uint32_t edata s_smooth_step_timer = 0;

/* 编辑步骤时保存原始角度, S12 取消时恢复 */
uint8_t edata s_edit_backup_angles[MAX_SERVOS];

#define servo_active_mask g_config.servo_active_mask
#define recorded_sequence g_config.recorded_sequence
#define recorded_step_count g_config.recorded_step_count
#define initial_angles g_config.initial_angles
#define temp_threshold g_config.temperature_threshold
#define humi_threshold g_config.humidity_threshold

#define MENU_ITEMS_PER_PAGE 4
static const char* main_menu_items[] = {"Manual Control", "Record Sequence", "Play Sequence", "Set Initial Angles", "Environment Monitor", "Set Thresholds", "Servo Setup", "Edit Sequence", "Play Speed"};
static const uint8_t main_menu_item_count = 9;
static const uint8_t total_pages = 3;

#define SERVO_SETUP_ITEMS_PER_PAGE 4
#define EDIT_SEQ_ITEMS_PER_PAGE 4
static const uint8_t servo_setup_item_count = 17;
static const uint8_t servo_setup_total_pages = 5;

// ====================================================================
//                     3. 函数声明
// ====================================================================
void System_Hardware_Init(void);
void Clock_Init(void);
void TFPU_Init(void);
void System_Tick_ISR(void);
void Load_Default_Settings(void);
void Rebuild_Active_Servo_Map(void);
void Arm_Set_To_Pose(const uint8_t* target_angles);
void Arm_Set_To_Pose_Smooth(const uint8_t* target_angles, uint16_t step_delay_ms);
void Arm_Start_Smooth_Move(const uint8_t* target_angles);
uint8_t Arm_Smooth_Tick(uint16_t step_delay_ms);
void Arm_Set_To_Initial_Position(void);
void Check_Environment_And_Alarm(void);
void Check_And_Process_Serial(void);
void process_serial_command(char* cmd);
void handle_input(void);
void handle_main_menu_input(uint8_t key);
void handle_manual_control_input(uint8_t key, int16_t encoder_delta);
void handle_set_initial_angles_input(uint8_t key, int16_t encoder_delta);
void handle_recording_angles_input(uint8_t key, int16_t encoder_delta);
void handle_recording_delay_input(uint8_t key, int16_t encoder_delta);
void handle_playback_tick(void);
void handle_monitor_input(uint8_t key);
void handle_set_thresholds_input(uint8_t key, int16_t encoder_delta);
void handle_servo_setup_input(uint8_t key);
void handle_edit_sequence_input(uint8_t key, int16_t encoder_delta);
void handle_play_speed_input(uint8_t key, int16_t encoder_delta);
void draw_ui(void);
void draw_main_menu(void);
void draw_manual_control_ui(void);
void draw_set_initial_angles_ui(void);
void draw_recording_angles_ui(void);
void draw_recording_delay_ui(void);
void draw_playback_ui(void);
void draw_monitor_ui(void);
void draw_set_thresholds_ui(void);
void draw_alarm_ui(void);
void draw_servo_setup_ui(void);
void draw_edit_sequence_ui(void);
void draw_play_speed_ui(void);
void Send_Status_Update(void);

// ====================================================================
//                            4. 主函数
// ====================================================================
void main(void)
{
    System_Hardware_Init();
    Flash_Read_Config(&g_config);
    if (g_config.magic_number != CONFIG_MAGIC_NUM) {
        /* 检查是否是v1格式 (旧版本升级) */
        if (g_config.magic_number == CONFIG_MAGIC_NUM_V1 && Flash_Has_V1_Config()) {
            /* v1配置有效, 保留所有旧字段, 仅补齐新字段默认值 */
            g_config.playback_speed_index = 1; /* 1x */
            memset(g_config.manual_draft_angles, 0, sizeof(g_config.manual_draft_angles));
            g_config.manual_draft_valid = 0;
            Flash_Write_Config(&g_config); /* 升级为 v2 并写回 */
        } else {
            Load_Default_Settings();
            Flash_Write_Config(&g_config);
        }
    }
    Rebuild_Active_Servo_Map();

    /* 如果有手动模式草稿, 恢复; 否则用 initial_angles */
    if (g_config.manual_draft_valid) {
        memcpy(servo_angles, g_config.manual_draft_angles, sizeof(servo_angles));
    } else {
        memcpy(servo_angles, initial_angles, sizeof(servo_angles));
    }

    /* 防御: 越界的 speed_index 修正为默认值 */
    if (g_config.playback_speed_index >= SPEED_COUNT) {
        g_config.playback_speed_index = SPEED_INDEX_1X;
    }

    current_mode = MODE_MAIN_MENU;
    Arm_Set_To_Initial_Position();
    UART_SendString("\r\n--- Robotic Arm CLI Ready (ACK Mode) ---\r\n> ");

    while (1)
    {
        Check_Environment_And_Alarm();
        Check_And_Process_Serial();
        handle_input();

        if (current_mode == MODE_PLAYBACK) {
            if (s_smooth_active && playback_status == PLAYBACK_RUNNING) {
                Arm_Smooth_Tick(SMOOTH_POSE_DELAY_MS);
            } else if (!s_smooth_active) {
                handle_playback_tick();
            }
        }

        /* 关闭同步时, 每 ~3 秒定期上报一次状态供上位机刷新 */
        {
            static uint32_t periodic_report_counter = 0;
            if (!sync_enabled) {
                periodic_report_counter++;
                if (periodic_report_counter > (3000 / 30)) {
                    periodic_report_counter = 0;
                    Send_Status_Update();
                }
            } else {
                periodic_report_counter = 0;
            }
        }

        draw_ui();

        delay_ms(30);
    }
}

// ====================================================================
//                     5. 核心功能函数
// ====================================================================

/**
 * @brief  系统时钟: 内部高速IRC (HIRC) 设为 40MHz
 * @note   40MHz 校准值读取自芯片出厂CHIPID区 (需 EAXFR=1)
 */
void Clock_Init(void)
{
    uint16_t hirc_wait = 0;
    HIRCCR = 0x80;                    // 启动内部高速IRC
    /* 等待HIRC稳定位; 带超时保护: Keil仿真器不模拟HIRCCR的稳定位,
       若为死等则仿真永远停在此处 (真机上该位在数十us内即置位) */
    while (!(HIRCCR & 1) && --hirc_wait);
    CLKDIV = 0x04;                    // 切换前先降频, 防止频率跳变时跑飞
    IRTRIM = CHIPID18;                // 40MHz IRC 频率校准值
    VRTRIM = CHIPID24;                // 44M 频段电压校准值 (40MHz 属于该频段)
    IRCBAND = (IRCBAND & ~0x03) | 0x03; // 选择 44M 频段
    CLKSEL = 0x00;                    // 选择内部IRC作为主时钟源
    CLKDIV = 0x00;                    // 恢复全速 40MHz
}

/**
 * @brief  1ms 系统节拍中断: 更新系统节拍 + 编码器软件采样
 * @note   通过 set_timer_isr(Timer0, System_Tick_ISR) 挂到定时器0
 */
void System_Tick_ISR(void)
{
    g_sys_tick_count++;
    Encoder_Tick();
}

void System_Hardware_Init(void)
{
    /* --- 1. 系统时钟: 40MHz 内部高速IRC --- */
    EAXFR = 1;   // 允许访问扩展寄存器(XFR)
    WTST = 0;    // 程序取指不等待, 最快速度
    CKCON = 1;   // XRAM 访问+1T: 避免 UART DMA 与 CPU 同时访问 xdata 的总线冲突
    Clock_Init();

    /* --- 2. TFPU 浮点协处理器: PLL 120MHz, 必须早于第一次浮点运算 --- */
    TFPU_Init();

    /* --- 3. 通知各库函数主频 (屏蔽自动测量, 直接使用 40MHz) --- */
    set_timer_fosc(40000000UL);
    set_uart_fosc(40000000UL);
    set_i2c_fosc(40000000UL);

    /* --- 4. 1ms 系统节拍 (g_sys_tick_count + 编码器) --- */
    set_timer_mode(Timer0, "1ms", Timer_End);
    set_timer_isr(Timer0, System_Tick_ISR);

    EA = 1;  // 打开总中断 (I2C/UART DMA 均依赖中断)

    /* --- 5. 外设初始化 --- */
    PCA9685_Init();
    OLED_Init();
    Keypad_Init();
    Encoder_Init();
    Buzzer_Init();
    UART_Init();

    Buzzer_Beep(50);
}

void Load_Default_Settings(void)
{
    uint8_t i;

    servo_active_mask = 0x007F;
    for (i = 0; i < MAX_SERVOS; i++) {
        initial_angles[i] = DEFAULT_INITIAL_ANGLE;
    }
    recorded_step_count = 0;
    is_editing_step = 0;
    current_editing_step = 0;
    memset(recorded_sequence, 0, sizeof(recorded_sequence));
    temp_threshold = DEFAULT_TEMP_THRESHOLD;
    humi_threshold = DEFAULT_HUMI_THRESHOLD;
    g_config.playback_speed_index = 1;  /* 1x */
    memset(g_config.manual_draft_angles, 0, sizeof(g_config.manual_draft_angles));
    g_config.manual_draft_valid = 0;
}

void Rebuild_Active_Servo_Map(void)
{
    uint8_t i;

    active_servo_count = 0;
    for (i = 0; i < MAX_SERVOS; i++) {
        if ((servo_active_mask >> i) & 1) {
            active_servo_map[active_servo_count++] = i;
        }
    }

    /* 串口 servo on/off 可能在使用界面期间改变活动舵机数量,
       保证所有以 active_servo_count 为模的索引仍然有效 */
    if (active_servo_count == 0) {
        current_servo_map_index = 0;
        edit_seq_preview_servo = 0;
    } else {
        if (current_servo_map_index >= active_servo_count) current_servo_map_index = 0;
        if (edit_seq_preview_servo >= active_servo_count) edit_seq_preview_servo = 0;
    }
}

void Arm_Set_To_Pose(const uint8_t* target_angles)
{
    uint8_t i;

    for (i = 0; i < active_servo_count; i++) {
        uint8_t channel = active_servo_map[i];
        servo_angles[channel] = target_angles[channel];
        PCA9685_SetServoAngle(channel, servo_angles[channel]);
        delay_ms(20);
    }
}

/**
 * @brief  平滑移动所有活动舵机到目标姿态 (阻塞)
 * @note   所有舵机同时移动，每步1度，避免逐个舵机跳转带来的抖动和冲击
 */
void Arm_Set_To_Pose_Smooth(const uint8_t* target_angles, uint16_t step_delay_ms)
{
    uint8_t current_copy[MAX_SERVOS];
    uint8_t target_copy[MAX_SERVOS];
    int8_t dir[MAX_SERVOS];
    uint8_t moving;
    uint8_t i;

    /* 1. 复制当前角度和目标角度，计算每个舵机的移动方向 */
    moving = 0;
    for (i = 0; i < active_servo_count; i++) {
        uint8_t ch = active_servo_map[i];
        current_copy[ch] = servo_angles[ch];
        target_copy[ch] = target_angles[ch];
        if (target_copy[ch] > 180) target_copy[ch] = 180;

        if (current_copy[ch] != target_copy[ch]) {
            moving = 1;
            dir[ch] = (target_copy[ch] > current_copy[ch]) ? 1 : -1;
        } else {
            dir[ch] = 0;
        }
    }

    if (!moving) {
        /* 即使角度不需要变化, 也要把当前角度写入 PCA9685:
           上电时 servo_angles 已等于 initial_angles, 若不写一次,
           舵机在第一次手动操作前不会收到任何 PWM 脉冲 */
        for (i = 0; i < active_servo_count; i++) {
            uint8_t ch = active_servo_map[i];
            PCA9685_SetServoAngle(ch, servo_angles[ch]);
        }
        return;
    }

    /* 2. 所有舵机同步逐步移动，每次1度 */
    while (moving) {
        moving = 0;
        for (i = 0; i < active_servo_count; i++) {
            uint8_t ch = active_servo_map[i];
            if (current_copy[ch] != target_copy[ch]) {
                current_copy[ch] = (uint8_t)((int16_t)current_copy[ch] + dir[ch]);
                servo_angles[ch] = current_copy[ch];
                PCA9685_SetServoAngle(ch, current_copy[ch]);
                moving = 1;
            }
        }
        if (moving) {
            delay_ms(step_delay_ms);
        }
    }
}

/**
 * @brief  启动非阻塞平滑移动 (播放模式使用)
 */
void Arm_Start_Smooth_Move(const uint8_t* target_angles)
{
    uint8_t needs_move = 0;
    uint8_t i;

    for (i = 0; i < active_servo_count; i++) {
        uint8_t ch = active_servo_map[i];
        s_smooth_target[ch] = target_angles[ch];
        if (s_smooth_target[ch] > 180) s_smooth_target[ch] = 180;
        if (servo_angles[ch] != s_smooth_target[ch]) {
            needs_move = 1;
        }
    }

    if (needs_move) {
        s_smooth_active = 1;
        s_smooth_step_timer = g_sys_tick_count;
    }
}

/**
 * @brief  非阻塞平滑移动的每帧驱动 (播放模式主循环中调用)
 * @return 0=移动完成, 1=仍在移动中
 */
uint8_t Arm_Smooth_Tick(uint16_t step_delay_ms)
{
    uint8_t still_moving = 0;
    uint8_t i;
    uint8_t steps;
    uint32_t elapsed;

    if (!s_smooth_active) return 0;

    if (step_delay_ms == 0) step_delay_ms = 1;

    elapsed = g_sys_tick_count - s_smooth_step_timer;

    /* 未到下一帧时间则跳过 */
    if (elapsed < (uint32_t)step_delay_ms) {
        return 1;
    }

    /* AI8051U 主循环要刷新整屏 OLED, 周期约 50~65ms, 远大于 20ms 的
       步进间隔。这里按实际经过的时间补步 (单次最多 4 度), 既不会像
       原 STM32 版本那样丢弃所有超时帧导致永远不前进, 也不会在长时间
       阻塞后突然大幅跳动。 */
    steps = (uint8_t)(elapsed / step_delay_ms);
    if (steps > 4) steps = 4;
    s_smooth_step_timer = g_sys_tick_count;

    /* 所有活动舵机同步移动 steps 度 */
    for (i = 0; i < active_servo_count; i++) {
        uint8_t ch = active_servo_map[i];
        uint8_t angle = servo_angles[ch];
        uint8_t target = s_smooth_target[ch];
        if (angle != target) {
            if (angle < target) {
                if ((uint16_t)angle + steps >= target) angle = target;
                else angle = (uint8_t)(angle + steps);
            } else {
                if (angle <= (uint8_t)(target + steps)) angle = target;
                else angle = (uint8_t)(angle - steps);
            }
            servo_angles[ch] = angle;
            PCA9685_SetServoAngle(ch, angle);
            still_moving = 1;
        }
    }

    if (!still_moving) {
        s_smooth_active = 0;
        /* 移动完成, 重置步骤计时器, 使 delay_s 从"到达姿态"开始计时 */
        playback_step_timer = g_sys_tick_count;
        return 0;
    }
    return 1;
}

void Arm_Set_To_Initial_Position(void)
{
    Buzzer_Beep(50);
    Arm_Set_To_Pose_Smooth(initial_angles, SMOOTH_POSE_DELAY_MS);
}

void Check_Environment_And_Alarm(void)
{
    static uint32_t loop_counter = 0;
    uint8_t dht_buffer[5];
    uint8_t old_temp = g_temperature;
    uint8_t old_humi = g_humidity;
    uint8_t exceeds;

    /* DHT11 定时采样 (约每2秒一次) */
    loop_counter++;
    if (loop_counter > (2000 / 30)) {
        loop_counter = 0;
        if (DHT_Get_Temp_Humi_Data(dht_buffer)) {
            g_humidity = dht_buffer[0];
            g_temperature = dht_buffer[2];
        }
    }

    if (sync_enabled && (old_temp != g_temperature || old_humi != g_humidity)) {
        Send_Status_Update();
    }

    /* 非阻塞报警状态机 */
    exceeds = (uint8_t)((g_temperature > temp_threshold || g_humidity > humi_threshold) ? 1 : 0);

    if (alarm_active) {
        if (exceeds) {
            /* 仍然超标，保持报警但不阻塞系统。
               其他界面的短鸣(Buzzer_Beep)结尾会关闭蜂鸣器,
               这里每轮重新拉响, 避免报警被静音后不再响 */
            Buzzer_On();
        } else {
            /* 恢复正常，关闭报警 */
            Buzzer_Off();
            alarm_active = 0;
            if (sync_enabled) Send_Status_Update();
        }
    } else {
        if (exceeds) {
            /* 触发新报警 */
            Buzzer_On();
            alarm_active = 1;
        }
    }
}

// ====================================================================
//                   6. 逻辑处理句柄
// ====================================================================
/* 串口虚拟输入: key / enc 命令把按键和编码器增量注入主循环,
   与真实按键/编码器走完全相同的处理路径 (光标/选中/数值) */
static uint8_t s_serial_key_pending = 0;
static int16_t s_serial_enc_pending = 0;

void Check_And_Process_Serial(void)
{
    char cmd_buffer[RX_BUFFER_SIZE];
    uint8_t port;

    /* UART1 与 UART4 功能完全一致: 两路都轮询, 响应广播回两路 */
    for (port = 0; port < UART_PORT_COUNT; port++) {
        if (UART_GetCommand(port, cmd_buffer)) {
            process_serial_command(cmd_buffer);
            UART_SendString("> ");
        }
    }
}

void process_serial_command(char* cmd)
{
    char response[128];
    int s, a, t, h, d;

    /* 播放模式下, 只允许只读/非破坏性命令 */
    #define IS_PLAYBACK_DESTRUCTIVE() (current_mode == MODE_PLAYBACK)

    if (strcmp(cmd, "help") == 0) {
        UART_SendString("--- Robotic Arm CLI ---\r\n");
        UART_SendString("Ports: UART1 (P3.0/P3.1) + UART4 (P0.2/P0.3), 115200 8N1\r\n");
        UART_SendString("System: help, status, reset_arm, reset_system, sync on/off\r\n");
        UART_SendString("Manual: angle <s> <a> (s=servo 1-16, a=angle 0-180)\r\n");
        UART_SendString("Record: rec_step, rec_insert <i>, rec_delay <d>, rec_clear\r\n");
        UART_SendString("Playback: play, stop, pause, resume\r\n");
        UART_SendString("Setup: servo on/off <s>, servo_save, set_thresh <t> <h>, speed <0.5|1|2>\r\n");
        UART_SendString("Remote: key <up|down|ok|cancel|info|stop|reset|sysreset>, enc <n>\r\n");
    } else if (strcmp(cmd, "status") == 0) {
        Send_Status_Update();
    } else if (strcmp(cmd, "sync on") == 0) {
        sync_enabled = 1; Send_Status_Update(); UART_SendString("OK: Sync enabled.\r\n");
    } else if (strcmp(cmd, "sync off") == 0) {
        sync_enabled = 0; UART_SendString("OK: Sync disabled.\r\n");
    } else if (strcmp(cmd, "reset_arm") == 0) {
        if (IS_PLAYBACK_DESTRUCTIVE()) {
            UART_SendString("ERROR: Not allowed during playback.\r\n");
        } else {
            Arm_Set_To_Initial_Position(); UART_SendString("OK: Arm reset.\r\n");
        }
    } else if (strcmp(cmd, "reset_system") == 0) {
        if (IS_PLAYBACK_DESTRUCTIVE()) {
            UART_SendString("ERROR: Not allowed during playback.\r\n");
        } else {
            Buzzer_Beep(100); delay_ms(50);
            Load_Default_Settings(); Flash_Write_Config(&g_config); Rebuild_Active_Servo_Map();
            Arm_Set_To_Initial_Position(); current_mode = MODE_MAIN_MENU;
            UART_SendString("OK: System reset.\r\n");
        }
    } else if (sscanf(cmd, "angle %d %d", &s, &a) == 2) {
        if (current_mode == MODE_PLAYBACK) {
            UART_SendString("ERROR: Not allowed during playback.\r\n");
        } else if (s >= 1 && s <= MAX_SERVOS && a >= 0 && a <= 180) {
            uint8_t channel = (uint8_t)(s - 1);
            uint8_t old_angle = servo_angles[channel];
            servo_angles[channel] = (uint8_t)a;
            if ((servo_active_mask >> channel) & 1) {
                PCA9685_SetServoAngleSmooth(channel, old_angle, (uint8_t)a, SMOOTH_SINGLE_DELAY_MS);
            }
            sprintf(response, "OK: Angle for S%d set to %d.\r\n", s, a); UART_SendString(response);
        } else { UART_SendString("ERROR: Invalid params.\r\n"); }
    } else if (strcmp(cmd, "rec_step") == 0) {
        if (IS_PLAYBACK_DESTRUCTIVE()) {
            UART_SendString("ERROR: Not allowed during playback.\r\n");
        } else if (recorded_step_count < MAX_ACTION_STEPS) {
            memcpy(recorded_sequence[recorded_step_count].angles, servo_angles, sizeof(servo_angles));
            recorded_sequence[recorded_step_count].delay_s = 1;
            recorded_step_count++; Flash_Write_Config(&g_config);
            sprintf(response, "OK: Step %d recorded.\r\n", recorded_step_count); UART_SendString(response);
        } else { UART_SendString("ERROR: Sequence full.\r\n"); }
    } else if (sscanf(cmd, "rec_delay %d", &d) == 1) {
        if (IS_PLAYBACK_DESTRUCTIVE()) {
            UART_SendString("ERROR: Not allowed during playback.\r\n");
        } else if (recorded_step_count > 0 && d >= 0 && d <= 600) {
            recorded_sequence[recorded_step_count - 1].delay_s = (uint16_t)d;
            Flash_Write_Config(&g_config);
            sprintf(response, "OK: Delay for last step set to %d.\r\n", d); UART_SendString(response);
        } else { UART_SendString("ERROR: No steps or invalid delay.\r\n"); }
    } else if (strcmp(cmd, "rec_clear") == 0) {
        if (IS_PLAYBACK_DESTRUCTIVE()) {
            UART_SendString("ERROR: Not allowed during playback.\r\n");
        } else {
            recorded_step_count = 0;
            is_editing_step = 0;
            current_editing_step = 0;
            Flash_Write_Config(&g_config);
            UART_SendString("OK: Sequence cleared.\r\n");
        }
    } else if (sscanf(cmd, "rec_insert %d", &d) == 1) {
        if (IS_PLAYBACK_DESTRUCTIVE()) {
            UART_SendString("ERROR: Not allowed during playback.\r\n");
        } else if (recorded_step_count >= MAX_ACTION_STEPS) {
            UART_SendString("ERROR: Sequence full.\r\n");
        } else if (d < 1 || d > (int)recorded_step_count + 1) {
            sprintf(response, "ERROR: Index must be 1..%d.\r\n", recorded_step_count + 1);
            UART_SendString(response);
        } else {
            uint8_t idx = (uint8_t)(d - 1);  /* 1-based to 0-based */
            uint8_t i;
            /* 步骤从 idx 起向后移动 */
            for (i = recorded_step_count; i > idx; i--) {
                recorded_sequence[i] = recorded_sequence[i - 1];
            }
            memcpy(recorded_sequence[idx].angles, servo_angles, sizeof(servo_angles));
            recorded_sequence[idx].delay_s = 1;
            recorded_step_count++;
            Flash_Write_Config(&g_config);
            sprintf(response, "OK: Inserted step at %d (total %d).\r\n", d, recorded_step_count);
            UART_SendString(response);
        }
    } else if (strcmp(cmd, "play") == 0) {
        if (recorded_step_count > 0) {
            current_mode = MODE_PLAYBACK; playback_status = PLAYBACK_RUNNING;
            playback_current_step = 0; playback_step_timer = g_sys_tick_count;
            Arm_Start_Smooth_Move(recorded_sequence[0].angles);
            UART_SendString("OK: Playback started.\r\n");
        }
        else { UART_SendString("ERROR: No sequence.\r\n"); }
    } else if (strcmp(cmd, "stop") == 0) {
        if (current_mode == MODE_PLAYBACK) {
            playback_status = PLAYBACK_STOPPED;
            s_smooth_active = 0;
            current_mode = MODE_MAIN_MENU;
            UART_SendString("OK: Playback stopped.\r\n");
        } else { UART_SendString("OK.\r\n"); }
    } else if (strcmp(cmd, "pause") == 0) {
        if (playback_status == PLAYBACK_RUNNING) { playback_status = PLAYBACK_PAUSED; UART_SendString("OK: Playback paused.\r\n"); }
    } else if (strcmp(cmd, "resume") == 0) {
        if (playback_status == PLAYBACK_PAUSED) { playback_status = PLAYBACK_RUNNING; playback_step_timer = g_sys_tick_count; UART_SendString("OK: Playback resumed.\r\n"); }
    } else if (sscanf(cmd, "set_thresh %d %d", &t, &h) == 2) {
        if (IS_PLAYBACK_DESTRUCTIVE()) {
            UART_SendString("ERROR: Not allowed during playback.\r\n");
        } else if (t >= 0 && t <= 100 && h >= 0 && h <= 100) {
            temp_threshold = (uint8_t)t; humi_threshold = (uint8_t)h;
            Flash_Write_Config(&g_config); UART_SendString("OK: Thresholds saved.\r\n");
        } else { UART_SendString("ERROR: Invalid values.\r\n"); }
    } else if (sscanf(cmd, "servo on %d", &s) == 1) {
        if (IS_PLAYBACK_DESTRUCTIVE()) {
            UART_SendString("ERROR: Not allowed during playback.\r\n");
        } else if(s >= 1 && s <= 16) {
            servo_active_mask |= (uint16_t)(1 << (s-1));
            Rebuild_Active_Servo_Map();
            /* 立即给新启用通道一次 PWM 输出, 避免舵机无脉冲 */
            Arm_Set_To_Pose_Smooth(servo_angles, SMOOTH_POSE_DELAY_MS);
            sprintf(response, "OK: S%d ON.\r\n", s); UART_SendString(response);
        }
        else { UART_SendString("ERROR: Invalid servo.\r\n"); }
    } else if (sscanf(cmd, "servo off %d", &s) == 1) {
        if (IS_PLAYBACK_DESTRUCTIVE()) {
            UART_SendString("ERROR: Not allowed during playback.\r\n");
        } else if(s >= 1 && s <= 16) {
            uint8_t ch = (uint8_t)(s - 1);
            servo_active_mask &= (uint16_t)~(1 << ch);
            /* 只清掩码不会停止 PCA9685 的脉冲输出, 该通道仍会保持力矩。
               写 FULL_OFF(off=0x1000) 真正关闭通道 */
            PCA9685_SetPWM(ch, 0, 0x1000);
            Rebuild_Active_Servo_Map();
            sprintf(response, "OK: S%d OFF.\r\n", s); UART_SendString(response);
        }
        else { UART_SendString("ERROR: Invalid servo.\r\n"); }
    } else if (strcmp(cmd, "servo_save") == 0) {
        if (IS_PLAYBACK_DESTRUCTIVE()) {
            UART_SendString("ERROR: Not allowed during playback.\r\n");
        } else {
            Flash_Write_Config(&g_config); UART_SendString("OK: Servo config saved.\r\n");
        }
    } else if (sscanf(cmd, "speed %d", &d) == 1) {
        /* d 取 5/10/20 表示 0.5x/1x/2x, 也允许 1/2 简写 */
        uint8_t idx;
        if (d == 5 || d == 0) idx = SPEED_INDEX_0_5X;
        else if (d == 10 || d == 1) idx = SPEED_INDEX_1X;
        else if (d == 20 || d == 2) idx = SPEED_INDEX_2X;
        else { UART_SendString("ERROR: Use 0.5/1/2 (or 5/10/20).\r\n"); return; }
        g_config.playback_speed_index = idx;
        Flash_Write_Config(&g_config);
        sprintf(response, "OK: Speed set to %d.%dx.\r\n", s_speed_x10[idx] / 10, s_speed_x10[idx] % 10);
        UART_SendString(response);
    } else if (strcmp(cmd, "key up") == 0) {
        s_serial_key_pending = KEY_UP;
        UART_SendString("OK: KEY_UP queued.\r\n");
    } else if (strcmp(cmd, "key down") == 0) {
        s_serial_key_pending = KEY_DOWN;
        UART_SendString("OK: KEY_DOWN queued.\r\n");
    } else if (strcmp(cmd, "key ok") == 0) {
        s_serial_key_pending = KEY_CONFIRM;
        UART_SendString("OK: KEY_CONFIRM queued.\r\n");
    } else if (strcmp(cmd, "key cancel") == 0) {
        s_serial_key_pending = KEY_CANCEL;
        UART_SendString("OK: KEY_CANCEL queued.\r\n");
    } else if (strcmp(cmd, "key info") == 0) {
        s_serial_key_pending = KEY_INFO;
        UART_SendString("OK: KEY_INFO queued.\r\n");
    } else if (strcmp(cmd, "key stop") == 0) {
        s_serial_key_pending = KEY_STOP;
        UART_SendString("OK: KEY_STOP queued.\r\n");
    } else if (strcmp(cmd, "key reset") == 0) {
        s_serial_key_pending = KEY_RESET;
        UART_SendString("OK: KEY_RESET queued.\r\n");
    } else if (strcmp(cmd, "key sysreset") == 0) {
        s_serial_key_pending = KEY_SYS_RESET;
        UART_SendString("OK: KEY_SYS_RESET queued.\r\n");
    } else if (sscanf(cmd, "enc %d", &d) == 1) {
        /* 虚拟编码器: +n 增大 / -n 减小, 单次限幅 ±100 格 */
        if (d > 100) d = 100;
        if (d < -100) d = -100;
        s_serial_enc_pending = (int16_t)(s_serial_enc_pending + (int16_t)d);
        if (s_serial_enc_pending > 200) s_serial_enc_pending = 200;
        if (s_serial_enc_pending < -200) s_serial_enc_pending = -200;
        sprintf(response, "OK: encoder %d step(s) queued.\r\n", d);
        UART_SendString(response);
    } else {
        UART_SendString("ERROR: Unknown command.\r\n");
    }
    if(sync_enabled) Send_Status_Update();
}

void handle_input(void)
{
    uint8_t key = Keypad_Scan();
    int16_t encoder_delta = (int16_t)(Encoder_Get_Count() / 2);
    uint8_t state_changed;

    /* 合并串口注入的虚拟按键/编码器 (真实按键优先, 编码器累加) */
    if (s_serial_key_pending != 0) {
        if (key == 0) key = s_serial_key_pending;
        s_serial_key_pending = 0;
    }
    if (s_serial_enc_pending != 0) {
        encoder_delta = (int16_t)(encoder_delta + s_serial_enc_pending);
        s_serial_enc_pending = 0;
    }

    state_changed = (uint8_t)((key != 0 || encoder_delta != 0) ? 1 : 0);

    if (key == KEY_RESET) { Arm_Set_To_Initial_Position(); if(sync_enabled) Send_Status_Update(); return; }
    if (key == KEY_SYS_RESET) {
        Buzzer_Beep(100); delay_ms(50);
        Load_Default_Settings();
        g_config.manual_draft_valid = 0;  /* 重置后丢弃手动草稿 */
        Flash_Write_Config(&g_config);
        Rebuild_Active_Servo_Map();
        Arm_Set_To_Initial_Position(); current_mode = MODE_MAIN_MENU;
        OLED_Clear(); OLED_Printf(10, 24, OLED_8X16, "System Reset!"); OLED_Update();
        delay_ms(1500); if(sync_enabled) Send_Status_Update();
        return;
    }

    if (current_mode == MODE_PLAYBACK) {
        if (key == KEY_STOP) { playback_status = PLAYBACK_STOPPED; s_smooth_active = 0; playback_detail_mode = 0; current_mode = MODE_MAIN_MENU; }
        else if (key == KEY_INFO) { playback_detail_mode ^= 1; playback_detail_page = 0; }
        else if (key == KEY_UP)   { if (playback_detail_mode) { playback_detail_page = 0; } }
        else if (key == KEY_DOWN) { if (playback_detail_mode) { playback_detail_page = 1; } }
        else if (key == KEY_CANCEL) {
            if (playback_status == PLAYBACK_RUNNING) {
                playback_status = PLAYBACK_PAUSED;
            }
            else if (playback_status == PLAYBACK_PAUSED) {
                playback_status = PLAYBACK_RUNNING;
            }
        }
        if(sync_enabled && key != 0) Send_Status_Update();
        return;
    }

    switch (current_mode) {
        case MODE_MAIN_MENU: handle_main_menu_input(key); break;
        case MODE_MANUAL_CONTROL: handle_manual_control_input(key, encoder_delta); break;
        case MODE_SET_INITIAL_ANGLES: handle_set_initial_angles_input(key, encoder_delta); break;
        case MODE_RECORDING_ANGLES: handle_recording_angles_input(key, encoder_delta); break;
        case MODE_RECORDING_DELAY: handle_recording_delay_input(key, encoder_delta); break;
        case MODE_MONITOR: handle_monitor_input(key); break;
        case MODE_SET_THRESHOLDS: handle_set_thresholds_input(key, encoder_delta); break;
        case MODE_SERVO_SETUP: handle_servo_setup_input(key); break;
        case MODE_EDIT_SEQUENCE: handle_edit_sequence_input(key, encoder_delta); break;
        case MODE_PLAY_SPEED: handle_play_speed_input(key, encoder_delta); break;
        default: break;
    }

    if (state_changed && sync_enabled) {
        Send_Status_Update();
    }
}

void handle_main_menu_input(uint8_t key)
{
    if (key == KEY_UP) {
        if (menu_cursor_pos > 0) menu_cursor_pos--;
        else menu_cursor_pos = (uint8_t)(main_menu_item_count - 1);
    } else if (key == KEY_DOWN) {
        if (menu_cursor_pos < main_menu_item_count - 1) menu_cursor_pos++;
        else menu_cursor_pos = 0;
    } else if (key == KEY_CONFIRM) {
        switch (menu_cursor_pos) {
            case 0: if (active_servo_count > 0) { current_mode = MODE_MANUAL_CONTROL; current_servo_map_index = 0; } break;
            case 1:
                if (active_servo_count > 0 && recorded_step_count < MAX_ACTION_STEPS) {
                    current_mode = MODE_RECORDING_ANGLES; current_editing_step = recorded_step_count;
                    current_servo_map_index = 0;
                    /* 进入录制时先把机械臂平滑移动到上一步的姿态(或初始姿态),
                       否则 servo_angles 与舵机实际位置不一致, 第一次编码器
                       微调会让舵机突然跳到新角度 */
                    if (current_editing_step > 0) {
                        Arm_Set_To_Pose_Smooth(recorded_sequence[current_editing_step-1].angles,
                                               SMOOTH_POSE_DELAY_MS);
                        memcpy(servo_angles, recorded_sequence[current_editing_step-1].angles,
                               sizeof(servo_angles));
                    } else {
                        Arm_Set_To_Pose_Smooth(initial_angles, SMOOTH_POSE_DELAY_MS);
                        memcpy(servo_angles, initial_angles, sizeof(servo_angles));
                    }
                } break;
            case 2:
                if (recorded_step_count > 0) {
                    current_mode = MODE_PLAYBACK;
                    playback_status = PLAYBACK_RUNNING;
                    playback_current_step = 0;
                    playback_step_timer = g_sys_tick_count;
                    Arm_Start_Smooth_Move(recorded_sequence[0].angles);
                } break;
            case 3:
                if (active_servo_count > 0) {
                    current_mode = MODE_SET_INITIAL_ANGLES;
                    current_servo_map_index = 0;
                    /* 从 initial_angles 开始, 确保 S12 取消能真正回到原值 */
                    memcpy(servo_angles, initial_angles, sizeof(servo_angles));
                } break;
            case 4: current_mode = MODE_MONITOR; break;
            case 5: current_mode = MODE_SET_THRESHOLDS; threshold_cursor_pos = 0; break;
            case 6: current_mode = MODE_SERVO_SETUP; servo_setup_cursor_pos = 0; temp_servo_mask = servo_active_mask; break;
            case 7:
                if (recorded_step_count > 0) {
                    current_mode = MODE_EDIT_SEQUENCE;
                    edit_seq_cursor = 0;
                    edit_seq_page = 0;
                } break;
            case 8: speed_before_edit = g_config.playback_speed_index; current_mode = MODE_PLAY_SPEED; break;
        }
    }
    current_page = (uint8_t)(menu_cursor_pos / MENU_ITEMS_PER_PAGE);
}

/* ─────────────────────────────────────────────────────
   三个舵机操作模式的公共逻辑：编码器调角度 + S7/S8切换舵机
   ──────────────────────────────────────────────────── */
static void process_servo_control(int16_t encoder_delta, uint8_t key)
{
    uint8_t channel;

    if (active_servo_count == 0) return;

    channel = active_servo_map[current_servo_map_index];

    /* 编码器调整角度 (使用 32 位运算, 避免 C251 16位int乘法溢出) */
    if (encoder_delta != 0) {
        int16_t temp_angle;
        uint8_t old_angle;

        temp_angle = (int16_t)((int32_t)servo_angles[channel] + (int32_t)encoder_delta * ANGLE_STEP);
        if (temp_angle > 180) temp_angle = 180;
        if (temp_angle < 0)  temp_angle = 0;
        old_angle = servo_angles[channel];
        servo_angles[channel] = (uint8_t)temp_angle;
        PCA9685_SetServoAngleSmooth(channel, old_angle, servo_angles[channel], SMOOTH_SINGLE_DELAY_MS);
    }

    /* S7/S8 切换舵机 */
    if (key == KEY_UP) {
        current_servo_map_index = (uint8_t)((current_servo_map_index + active_servo_count - 1) % active_servo_count);
    } else if (key == KEY_DOWN) {
        current_servo_map_index = (uint8_t)((current_servo_map_index + 1) % active_servo_count);
    }
}

void handle_manual_control_input(uint8_t key, int16_t encoder_delta)
{
    if (active_servo_count == 0) { current_mode = MODE_MAIN_MENU; return; }
    process_servo_control(encoder_delta, key);
    if (key == KEY_CANCEL) {
        /* 自动保存草稿, 断电后恢复 */
        memcpy(g_config.manual_draft_angles, servo_angles, sizeof(g_config.manual_draft_angles));
        g_config.manual_draft_valid = 1;
        Flash_Write_Config(&g_config);
        current_mode = MODE_MAIN_MENU;
    }
}

void handle_set_initial_angles_input(uint8_t key, int16_t encoder_delta)
{
    if (active_servo_count == 0) { current_mode = MODE_MAIN_MENU; return; }
    process_servo_control(encoder_delta, key);
    if (key == KEY_CONFIRM) {
        memcpy(initial_angles, servo_angles, sizeof(initial_angles));
        Flash_Write_Config(&g_config);
        current_mode = MODE_MAIN_MENU;
    } else if (key == KEY_CANCEL) {
        /* 先平滑回到初始角度 (函数内部会把 servo_angles 更新到目标值),
           再同步内存; 若先 memcpy 则当前位置=目标位置, 不会产生移动 */
        Arm_Set_To_Initial_Position();
        memcpy(servo_angles, initial_angles, sizeof(servo_angles));
        current_mode = MODE_MAIN_MENU;
    }
}

void handle_recording_angles_input(uint8_t key, int16_t encoder_delta)
{
    if (active_servo_count == 0) { current_mode = MODE_MAIN_MENU; return; }
    process_servo_control(encoder_delta, key);
    if (key == KEY_CONFIRM) {
        memcpy(recorded_sequence[current_editing_step].angles, servo_angles, sizeof(servo_angles));
        /* 仅追加新步骤时初始化 delay_s=1, 编辑已有步骤保留原值 */
        if (!is_editing_step) {
            recorded_sequence[current_editing_step].delay_s = 1;
        }
        current_mode = MODE_RECORDING_DELAY;
    } else if (key == KEY_CANCEL) {
        if (is_editing_step) {
            /* 恢复原始角度 (内存和物理)。必须先调用平滑移动, 再写回
               servo_angles: 该函数以 servo_angles 作为当前位置, 若先
               memcpy 则当前位置=目标位置, 物理舵机不会移动 */
            Arm_Set_To_Pose_Smooth(s_edit_backup_angles, SMOOTH_POSE_DELAY_MS);
            memcpy(servo_angles, s_edit_backup_angles, sizeof(servo_angles));
            current_mode = MODE_EDIT_SEQUENCE;
        } else {
            current_mode = MODE_MAIN_MENU;
        }
        is_editing_step = 0;
    }
}

void handle_recording_delay_input(uint8_t key, int16_t encoder_delta)
{
    if (encoder_delta != 0) {
        int32_t temp_delay = (int32_t)recorded_sequence[current_editing_step].delay_s + (int32_t)encoder_delta;
        if (temp_delay < 0) temp_delay = 0;
        if (temp_delay > 600) temp_delay = 600;
        recorded_sequence[current_editing_step].delay_s = (uint16_t)temp_delay;
    }
    if (key == KEY_CONFIRM) {
        if (!is_editing_step && current_editing_step == recorded_step_count) {
            recorded_step_count++;
        }
        Flash_Write_Config(&g_config);

        if (is_editing_step) {
            /* 编辑完成后回到步骤列表 */
            is_editing_step = 0;
            current_mode = MODE_EDIT_SEQUENCE;
        } else if (recorded_step_count < MAX_ACTION_STEPS) {
            /* 追加模式: 继续录下一步 */
            current_editing_step = recorded_step_count;
            current_mode = MODE_RECORDING_ANGLES;
        } else {
            current_mode = MODE_MAIN_MENU;
        }
    } else if (key == KEY_CANCEL) {
        /* 保持 is_editing_step 标志，以便继续编辑 */
        current_mode = MODE_RECORDING_ANGLES;
    }
}

void handle_playback_tick(void)
{
    uint32_t current_tick;
    uint32_t base_delay_ms;
    uint32_t step_delay_ms;

    if (playback_status != PLAYBACK_RUNNING) return;

    current_tick = g_sys_tick_count;
    base_delay_ms = (uint32_t)recorded_sequence[playback_current_step].delay_s * 1000UL;
    /* 应用播放速度倍率: 0.5x -> 延时×2, 2x -> 延时×0.5 */
    step_delay_ms = (base_delay_ms * 10UL) / s_speed_x10[g_config.playback_speed_index];
    if (step_delay_ms < 50) step_delay_ms = 50;  /* 下限保护 */

    if (current_tick - playback_step_timer >= step_delay_ms) {
        playback_step_timer = current_tick;
        playback_current_step = (uint8_t)((playback_current_step + 1) % recorded_step_count);

        Arm_Start_Smooth_Move(recorded_sequence[playback_current_step].angles);

        if (sync_enabled) Send_Status_Update();
    }
}

void handle_monitor_input(uint8_t key)
{
    if (key == KEY_CANCEL) { current_mode = MODE_MAIN_MENU; }
}

void handle_set_thresholds_input(uint8_t key, int16_t encoder_delta)
{
    if (key == KEY_UP || key == KEY_DOWN) {
        threshold_cursor_pos = (uint8_t)(threshold_cursor_pos ? 0 : 1);
    }
    if (encoder_delta != 0) {
        if (threshold_cursor_pos == 0) {
            int16_t temp = (int16_t)temp_threshold + encoder_delta;
            if (temp < 0) { temp = 0; } if (temp > 100) { temp = 100; }
            temp_threshold = (uint8_t)temp;
        } else {
            int16_t humi = (int16_t)humi_threshold + encoder_delta;
            if (humi < 0) { humi = 0; } if (humi > 100) { humi = 100; }
            humi_threshold = (uint8_t)humi;
        }
    }
    if (key == KEY_CONFIRM) {
        Flash_Write_Config(&g_config);
        current_mode = MODE_MAIN_MENU;
    } else if (key == KEY_CANCEL) {
        /* 只恢复阈值，不影响其他未保存的修改。
           若 EEPROM 内容无效(magic=0), 回退到默认值而不是垃圾数据 */
        SystemConfig_t xdata saved;
        Flash_Read_Config(&saved);
        if (saved.magic_number != 0) {
            temp_threshold = saved.temperature_threshold;
            humi_threshold = saved.humidity_threshold;
        } else {
            temp_threshold = DEFAULT_TEMP_THRESHOLD;
            humi_threshold = DEFAULT_HUMI_THRESHOLD;
        }
        current_mode = MODE_MAIN_MENU;
    }
}

void handle_servo_setup_input(uint8_t key)
{
    uint16_t old_mask;
    uint8_t i;

    if (key == KEY_UP) {
        if (servo_setup_cursor_pos > 0) servo_setup_cursor_pos--;
        else servo_setup_cursor_pos = (uint8_t)(servo_setup_item_count - 1);
    } else if (key == KEY_DOWN) {
        if (servo_setup_cursor_pos < servo_setup_item_count - 1) servo_setup_cursor_pos++;
        else servo_setup_cursor_pos = 0;
    }
    if (key == KEY_CONFIRM) {
        if (servo_setup_cursor_pos < MAX_SERVOS) {
            temp_servo_mask ^= (uint16_t)(1 << servo_setup_cursor_pos);
        } else {
            old_mask = servo_active_mask;
            servo_active_mask = temp_servo_mask;
            Flash_Write_Config(&g_config);
            Rebuild_Active_Servo_Map();
            /* 被关闭的舵机: 写 FULL_OFF 停止脉冲, 否则仍会保持力矩 */
            for (i = 0; i < MAX_SERVOS; i++) {
                if (((old_mask >> i) & 1) && !((servo_active_mask >> i) & 1)) {
                    PCA9685_SetPWM(i, 0, 0x1000);
                }
            }
            /* 新启用的舵机需要立即输出一次 PWM, 否则要等用户第一次
               调角度才动作 (PCA9685 未被写入的通道没有脉冲输出) */
            Arm_Set_To_Pose_Smooth(servo_angles, SMOOTH_POSE_DELAY_MS);
            current_mode = MODE_MAIN_MENU;
        }
    } else if (key == KEY_CANCEL) { current_mode = MODE_MAIN_MENU; }
    servo_setup_page = (uint8_t)(servo_setup_cursor_pos / SERVO_SETUP_ITEMS_PER_PAGE);
}

/* ─────────────────────────────────────────────────────
   编辑序列模式: 浏览/编辑/删除已录制步骤
   ──────────────────────────────────────────────────── */
void handle_edit_sequence_input(uint8_t key, int16_t encoder_delta)
{
    uint8_t total_items = recorded_step_count;

    /* 编辑界面打开期间, 串口 rec_clear 可能把步骤清空, 必须防止
       edit_seq_cursor 下溢成 255 后越界访问 recorded_sequence[255] */
    if (total_items == 0) {
        edit_seq_cursor = 0;
        current_mode = MODE_MAIN_MENU;
        return;
    }
    if (edit_seq_cursor >= total_items) {
        edit_seq_cursor = (uint8_t)(total_items - 1);
    }

    if (key == KEY_UP) {
        if (edit_seq_cursor > 0) edit_seq_cursor--;
        else edit_seq_cursor = (uint8_t)(total_items - 1);
    } else if (key == KEY_DOWN) {
        if (edit_seq_cursor < total_items - 1) edit_seq_cursor++;
        else edit_seq_cursor = 0;
    }

    /* 编码器选择预览哪个舵机的角度 */
    if (encoder_delta != 0 && active_servo_count > 0) {
        int16_t new_idx = (int16_t)edit_seq_preview_servo + encoder_delta;
        uint8_t count = active_servo_count;
        while (new_idx < 0) new_idx += (int16_t)count;
        while ((uint8_t)new_idx >= count) new_idx -= (int16_t)count;
        edit_seq_preview_servo = (uint8_t)new_idx;
    }

    if (key == KEY_CONFIRM) {
        /* 编辑选中步骤: 复用录制角度 UI */
        current_editing_step = edit_seq_cursor;
        is_editing_step = 1;
        /* 备份原始角度, S12 取消时恢复 */
        memcpy(s_edit_backup_angles,
               recorded_sequence[current_editing_step].angles,
               sizeof(s_edit_backup_angles));
        current_servo_map_index = 0;
        /* 平滑移动到当前编辑步骤的姿态。Arm_Set_To_Pose_Smooth 内部会把
           servo_angles 更新到目标值, 不能先把目标 memcpy 进 servo_angles,
           否则当前位置=目标位置, 物理舵机不会移动 */
        Arm_Set_To_Pose_Smooth(recorded_sequence[current_editing_step].angles,
                               SMOOTH_POSE_DELAY_MS);
        current_mode = MODE_RECORDING_ANGLES;
    } else if (key == KEY_INFO) {
        /* 在选中步骤前插入一个新步骤, 内容 = 当前 servo_angles */
        if (recorded_step_count < MAX_ACTION_STEPS) {
            uint8_t idx = edit_seq_cursor;
            uint8_t i;
            for (i = recorded_step_count; i > idx; i--) {
                recorded_sequence[i] = recorded_sequence[i - 1];
            }
            memcpy(recorded_sequence[idx].angles, servo_angles, sizeof(servo_angles));
            recorded_sequence[idx].delay_s = 1;
            recorded_step_count++;
            Flash_Write_Config(&g_config);
            Buzzer_Beep(30);
        }
    } else if (key == KEY_STOP) {
        /* 删除选中步骤 */
        if (recorded_step_count > 0) {
            uint8_t i;
            for (i = edit_seq_cursor; i < recorded_step_count - 1; i++) {
                recorded_sequence[i] = recorded_sequence[i + 1];
            }
            recorded_step_count--;
            Flash_Write_Config(&g_config);
            if (edit_seq_cursor >= recorded_step_count && recorded_step_count > 0) {
                edit_seq_cursor = (uint8_t)(recorded_step_count - 1);
            }
            if (recorded_step_count == 0) {
                current_mode = MODE_MAIN_MENU;
            }
        }
        Buzzer_Beep(30);
    } else if (key == KEY_CANCEL) {
        current_mode = MODE_MAIN_MENU;
    }
    edit_seq_page = (uint8_t)(edit_seq_cursor / MENU_ITEMS_PER_PAGE);
}

void draw_edit_sequence_ui(void)
{
    uint8_t total_items = recorded_step_count;
    uint8_t seq_total_pages;
    uint8_t start;
    uint8_t end;
    uint8_t i;

    OLED_Printf(10, 0, OLED_8X16, "EDIT SEQUENCE");
    OLED_DrawLine(0, 17, 127, 17);

    if (total_items == 0) {
        OLED_Printf(20, 30, OLED_6X8, "No steps recorded");
        OLED_Printf(0, 56, OLED_6X8, "S12 to exit");
        return;
    }

    seq_total_pages = (uint8_t)((total_items + EDIT_SEQ_ITEMS_PER_PAGE - 1) / EDIT_SEQ_ITEMS_PER_PAGE);
    if (seq_total_pages > 1) {
        OLED_Printf(100, 0, OLED_6X8, "%d/%d", edit_seq_page + 1, seq_total_pages);
    }

    start = (uint8_t)(edit_seq_page * EDIT_SEQ_ITEMS_PER_PAGE);
    end = (uint8_t)(start + EDIT_SEQ_ITEMS_PER_PAGE);
    if (end > total_items) end = total_items;

    for (i = start; i < end; i++) {
        uint8_t y = (uint8_t)(20 + (i % EDIT_SEQ_ITEMS_PER_PAGE) * 10);
        if (i == edit_seq_cursor) {
            OLED_Printf(0, y, OLED_6X8, ">");
        }
        if (active_servo_count > 0) {
            uint8_t preview_ch = active_servo_map[edit_seq_preview_servo];
            OLED_Printf(8, y, OLED_6X8, "S%d  S%d:%-3d  %ds",
                        i + 1, preview_ch + 1,
                        recorded_sequence[i].angles[preview_ch],
                        recorded_sequence[i].delay_s);
        } else {
            OLED_Printf(8, y, OLED_6X8, "Step %-2d  %ds", i + 1, recorded_sequence[i].delay_s);
        }
    }
    OLED_Printf(0, 56, OLED_6X8, "S3:Ins S11:Edit S5:Del");
}

/* ─────────────────────────────────────────────────────
   播放速度选择模式: 0.5x / 1x / 2x
   ──────────────────────────────────────────────────── */
void handle_play_speed_input(uint8_t key, int16_t encoder_delta)
{
    int16_t new_idx = (int16_t)g_config.playback_speed_index;
    uint8_t changed = 0;

    if (encoder_delta != 0) {
        new_idx += (int16_t)encoder_delta;
        if (new_idx < 0) new_idx = 0;
        if (new_idx >= SPEED_COUNT) new_idx = (int16_t)(SPEED_COUNT - 1);
        changed = 1;
    }
    if (key == KEY_UP)   { new_idx--; if (new_idx < 0) new_idx = (int16_t)(SPEED_COUNT - 1); changed = 1; }
    if (key == KEY_DOWN) { new_idx++; if (new_idx >= SPEED_COUNT) new_idx = 0; changed = 1; }

    if (changed) {
        g_config.playback_speed_index = (uint8_t)new_idx;
    }
    if (key == KEY_CONFIRM) {
        /* 只有确认时才写 EEPROM: 每格编码器都写会反复擦除 8 个扇区,
           既磨损 Flash 又会让界面卡顿 */
        Flash_Write_Config(&g_config);
        current_mode = MODE_MAIN_MENU;
    } else if (key == KEY_CANCEL) {
        g_config.playback_speed_index = speed_before_edit;
        current_mode = MODE_MAIN_MENU;
    }
}

void draw_play_speed_ui(void)
{
    uint8_t i;

    OLED_Printf(0, 0, OLED_8X16, "PLAY SPEED");
    OLED_DrawLine(0, 17, 127, 17);

    for (i = 0; i < SPEED_COUNT; i++) {
        uint8_t y = (uint8_t)(24 + i * 10);
        if (i == g_config.playback_speed_index) {
            OLED_Printf(0, y, OLED_6X8, ">%s", i == 0 ? "0.5x" : (i == 1 ? "1.0x" : "2.0x"));
        } else {
            OLED_Printf(8, y, OLED_6X8, " %s", i == 0 ? "0.5x" : (i == 1 ? "1.0x" : "2.0x"));
        }
    }
    OLED_Printf(0, 56, OLED_6X8, "S11:OK S12:Back");
}

// ====================================================================
//                   7. UI绘制函数
// ====================================================================
void draw_ui(void)
{
    OLED_Clear();

    /* 报警状态下覆盖显示报警界面，但不阻塞按键/串口/播放 */
    if (alarm_active) {
        draw_alarm_ui();
        OLED_Update();   /* 原 STM32 版本漏了这句, 报警画面只写入显存不上屏 */
        return;
    }

    switch (current_mode)
    {
        case MODE_MAIN_MENU:            draw_main_menu(); break;
        case MODE_MANUAL_CONTROL:       draw_manual_control_ui(); break;
        case MODE_SET_INITIAL_ANGLES:   draw_set_initial_angles_ui(); break;
        case MODE_RECORDING_ANGLES:     draw_recording_angles_ui(); break;
        case MODE_RECORDING_DELAY:      draw_recording_delay_ui(); break;
        case MODE_PLAYBACK:             draw_playback_ui(); break;
        case MODE_MONITOR:              draw_monitor_ui(); break;
        case MODE_SET_THRESHOLDS:       draw_set_thresholds_ui(); break;
        case MODE_SERVO_SETUP:          draw_servo_setup_ui(); break;
        case MODE_EDIT_SEQUENCE:        draw_edit_sequence_ui(); break;
        case MODE_PLAY_SPEED:           draw_play_speed_ui(); break;
    }
    OLED_Update();
}

void draw_main_menu(void)
{
    uint8_t start_index;
    uint8_t end_index;
    uint8_t i;

    OLED_Printf(15, 0, OLED_8X16, "MAIN MENU");
    if (total_pages > 1) OLED_Printf(100, 0, OLED_6X8, "%d/%d", current_page + 1, total_pages);
    OLED_DrawLine(0, 17, 127, 17);
    start_index = (uint8_t)(current_page * MENU_ITEMS_PER_PAGE);
    end_index = (uint8_t)(start_index + MENU_ITEMS_PER_PAGE);
    if (end_index > main_menu_item_count) end_index = main_menu_item_count;
    for (i = start_index; i < end_index; i++) {
        uint8_t y_pos = (uint8_t)(20 + (i % MENU_ITEMS_PER_PAGE) * 10);
        if (i == menu_cursor_pos) OLED_Printf(0, y_pos, OLED_6X8, "> %s", main_menu_items[i]);
        else OLED_Printf(8, y_pos, OLED_6X8, "%s", main_menu_items[i]);
    }
}

void draw_manual_control_ui(void)
{
    OLED_Printf(0, 0, OLED_6X8, "MANUAL (S12 to exit)");
    OLED_DrawLine(0, 10, 127, 10);
    if (active_servo_count > 0) {
        uint8_t channel = active_servo_map[current_servo_map_index];
        OLED_Printf(0, 16, OLED_8X16, "Servo %d", channel + 1);
        OLED_Printf(0, 32, OLED_8X16, "Angle: %-3d", servo_angles[channel]);
    } else { OLED_Printf(0, 24, OLED_6X8, "No active servos!"); }
    OLED_Printf(0, 56, OLED_6X8, "S7/S8 to switch");
}

void draw_set_initial_angles_ui(void)
{
    OLED_Printf(0, 0, OLED_6X8, "SET INITIAL (S11:OK)");
    OLED_DrawLine(0, 10, 127, 10);
    if (active_servo_count > 0) {
        uint8_t channel = active_servo_map[current_servo_map_index];
        OLED_Printf(0, 16, OLED_8X16, "Servo %d", channel + 1);
        OLED_Printf(0, 32, OLED_8X16, "Angle: %-3d", servo_angles[channel]);
    } else { OLED_Printf(0, 24, OLED_6X8, "No active servos!"); }
    OLED_Printf(0, 56, OLED_6X8, "S7/S8:Switch S12:Cancel");
}

void draw_recording_angles_ui(void)
{
    OLED_Printf(0, 0, OLED_6X8, "%s STEP %d/%d (S11:OK)",
                is_editing_step ? "EDIT" : "REC",
                current_editing_step + 1, MAX_ACTION_STEPS);
    OLED_DrawLine(0, 10, 127, 10);
    if (active_servo_count > 0) {
        uint8_t channel = active_servo_map[current_servo_map_index];
        OLED_Printf(0, 16, OLED_6X8, "Servo: %d", channel + 1);
        OLED_Printf(64, 16, OLED_6X8, "Angle: %-3d", servo_angles[channel]);
    } else { OLED_Printf(0, 24, OLED_6X8, "No active servos!"); }
    OLED_Printf(0, 32, OLED_8X16, "Set Angles...");
    OLED_Printf(0, 56, OLED_6X8, "S7/S8:Switch S12:Exit");
}

void draw_recording_delay_ui(void)
{
    OLED_Printf(0, 0, OLED_6X8, "%s STEP %d/%d (S11:OK)",
                is_editing_step ? "EDIT" : "REC",
                current_editing_step + 1, MAX_ACTION_STEPS);
    OLED_DrawLine(0, 10, 127, 10);
    OLED_Printf(0, 24, OLED_8X16, "Set Delay:");
    OLED_Printf(0, 40, OLED_8X16, "%d s", recorded_sequence[current_editing_step].delay_s);
    OLED_Printf(0, 56, OLED_6X8, "S12: Back to Angles");
}

void draw_playback_ui(void)
{
    const ActionStep_t* current_step = &recorded_sequence[playback_current_step];

    if (playback_status == PLAYBACK_PAUSED) {
        OLED_Printf(0, 0, OLED_8X16, "PAUSED");
    } else {
        OLED_Printf(0, 0, OLED_8X16, "PLAYING");
    }
    OLED_Printf(80, 0, OLED_6X8, "%s", playback_detail_mode ? "S3:Back" : "S3:Info");

    if (!playback_detail_mode) {
        /* ── 概览模式: 显示步骤和延时 ── */
        OLED_DrawLine(0, 17, 127, 17);
        OLED_Printf(0, 20, OLED_8X16, "Step %d/%d", playback_current_step + 1, recorded_step_count);
        OLED_Printf(0, 38, OLED_6X8, "Delay: %ds", current_step->delay_s);
        if (s_smooth_active) {
            OLED_Printf(0, 56, OLED_6X8, "Moving... S12 Pause");
        } else {
            OLED_Printf(0, 56, OLED_6X8, "S5:Stop S12:Pause");
        }
    } else {
        /* ── 详情模式: 分页显示舵机角度 (2列x4行) ── */
        int i;
        uint8_t base;
        OLED_DrawLine(0, 17, 127, 17);
        base = (uint8_t)(playback_detail_page * 8);
        for (i = 0; i < 4; i++) {
            uint8_t idx_left  = (uint8_t)(base + i);
            uint8_t idx_right = (uint8_t)(base + i + 4);
            uint8_t y = (uint8_t)(20 + i * 8);
            /* 左列 S(base+1) ~ S(base+4) */
            if ((servo_active_mask >> idx_left) & 1) {
                OLED_Printf(0, y, OLED_6X8, "S%d:%3d", idx_left + 1, current_step->angles[idx_left]);
            } else {
                OLED_Printf(0, y, OLED_6X8, "S%d: --", idx_left + 1);
            }
            /* 右列 S(base+5) ~ S(base+8) */
            if ((servo_active_mask >> idx_right) & 1) {
                OLED_Printf(64, y, OLED_6X8, "S%d:%3d", idx_right + 1, current_step->angles[idx_right]);
            } else {
                OLED_Printf(64, y, OLED_6X8, "S%d: --", idx_right + 1);
            }
        }
        OLED_Printf(108, 56, OLED_6X8, "%d/2", playback_detail_page + 1);
        OLED_Printf(0, 56, OLED_6X8, "S7:S1-8 S8:S9-16");
    }
}

void draw_monitor_ui(void)
{
    OLED_Printf(0, 0, OLED_8X16, "ENV. MONITOR");
    OLED_DrawLine(0, 17, 127, 17);
    OLED_Printf(10, 24, OLED_6X8, "Temp:");
    OLED_Printf(10, 36, OLED_8X16, "%d'C", g_temperature);
    OLED_Printf(74, 24, OLED_6X8, "Humi:");
    OLED_Printf(74, 36, OLED_8X16, "%d%%", g_humidity);
    OLED_DrawLine(0, 54, 127, 54);
    OLED_Printf(32, 56, OLED_6X8, "S12 to exit");
}

void draw_set_thresholds_ui(void)
{
    OLED_Printf(0, 0, OLED_8X16, "SET THRESHOLDS");
    OLED_DrawLine(0, 17, 127, 17);
    if (threshold_cursor_pos == 0) OLED_Printf(0, 24, OLED_6X8, "> Temp: %d 'C", temp_threshold);
    else OLED_Printf(8, 24, OLED_6X8, "Temp: %d 'C", temp_threshold);
    if (threshold_cursor_pos == 1) OLED_Printf(0, 40, OLED_6X8, "> Humi: %d %%", humi_threshold);
    else OLED_Printf(8, 40, OLED_6X8, "Humi: %d %%", humi_threshold);
    OLED_Printf(0, 56, OLED_6X8, "S11:Save S12:Cancel");
}

void draw_alarm_ui(void)
{
    OLED_Printf(30, 0, OLED_8X16, "ALARM!");
    OLED_DrawRectangle(28, 0, 70, 17, 0);
    if (g_temperature > temp_threshold) OLED_Printf(0, 24, OLED_8X16, "Temp: %d 'C (>%d)", g_temperature, temp_threshold);
    else OLED_Printf(0, 24, OLED_8X16, "Temp: %d 'C", g_temperature);
    if (g_humidity > humi_threshold) OLED_Printf(0, 44, OLED_8X16, "Humi: %d %% (>%d)", g_humidity, humi_threshold);
    else OLED_Printf(0, 44, OLED_8X16, "Humi: %d %%", g_humidity);
}

void draw_servo_setup_ui(void)
{
    uint8_t start_index;
    uint8_t end_index;
    uint8_t i;

    OLED_Printf(15, 0, OLED_8X16, "SERVO SETUP");
    if (servo_setup_total_pages > 1) {
        OLED_Printf(108, 0, OLED_6X8, "%d/%d", servo_setup_page + 1, servo_setup_total_pages);
    }
    OLED_DrawLine(0, 17, 127, 17);

    start_index = (uint8_t)(servo_setup_page * SERVO_SETUP_ITEMS_PER_PAGE);
    end_index = (uint8_t)(start_index + SERVO_SETUP_ITEMS_PER_PAGE);
    if (end_index > servo_setup_item_count) {
        end_index = servo_setup_item_count;
    }

    for (i = start_index; i < end_index; i++) {
        uint8_t y_pos = (uint8_t)(20 + (i % SERVO_SETUP_ITEMS_PER_PAGE) * 10);

        if (i == servo_setup_cursor_pos) {
            OLED_Printf(0, y_pos, OLED_6X8, ">");
        }

        if (i < MAX_SERVOS) {
            uint8_t is_active = (uint8_t)((temp_servo_mask >> i) & 1);
            OLED_Printf(8, y_pos, OLED_6X8, "Servo %-2d [%s]", i + 1, is_active ? "ON " : "OFF");
        } else {
            OLED_Printf(8, y_pos, OLED_6X8, "Save & Exit");
        }
    }
    OLED_Printf(0, 56, OLED_6X8, "S11:Toggle/OK S12:Cancel");
}

void Send_Status_Update(void)
{
    char buffer[256];
    int pos = 0;
    int i;

    pos = sprintf(buffer, "$MODE:%d;CUR_S_IDX:%d;MASK:%u;ANGS:",
                  current_mode, current_servo_map_index, (unsigned int)servo_active_mask);

    for (i = 0; i < MAX_SERVOS; i++) {
        pos += sprintf(buffer + pos, "%s%d",
                       (i > 0) ? "," : "", servo_angles[i]);
    }

    sprintf(buffer + pos,
            ";TEMP:%d;HUMI:%d;R_STEPS:%d;P_STAT:%d;#\r\n",
            g_temperature, g_humidity, recorded_step_count, playback_status);

    UART_SendString(buffer);
}

// ====================================================================
//                  8. TFPU 浮点协处理器初始化
// ====================================================================
/**
 * @brief  TFPU 时钟选择 PLL: PLL = MCLK/4 * 12 = 40/4*12 = 120MHz
 * @note   DMAIR 必须编译成 MOV DMAIR,#N 立即数指令才能触发 TFPU 命令,
 *         因此本函数必须使用优化等级 3 (等级 0 会生成 MOV A,#N / MOV DMAIR,A)
 */
#pragma OPTIMIZE(3)
void TFPU_Init(void)
{
    HSCLKDIV = 1;                       /* HSCLK = PLLCLK / 1 */
    TFPU_CLKDIV = 1;                    /* TFPU 时钟 = HSCLK / 1 = 120MHz */
    CLKSEL = 0x00;                      /* 主时钟仍为 HIRC 40MHz */
    CLKSEL |= 0x80;                     /* CKMS=1: PLL 倍频输出 144MHz 档 */
    USBCLK = (USBCLK & ~0x60) | 0x40;   /* PLL 输入 = MCLK/4 = 10MHz */
    USBCLK |= 0x80;                     /* 启动 PLL */
    delay_us(100);                      /* 等待 PLL 锁频 (>=50us) */
    CLKSEL |= 0x40;                     /* HSCLK 选择 PLL 时钟源 */
    DMAIR = 0x3F;                       /* TFPU 时钟选择 PLL */
    DMAIR = 0x31;                       /* 初始化协处理器 */
    DMAIR = 0x32;                       /* 清除初始化产生的异常 */
}
