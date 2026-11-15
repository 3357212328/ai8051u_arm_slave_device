#ifndef __FLASH_IF_H
#define __FLASH_IF_H

#include "def.h"     // uint8_t/uint16_t/uint32_t 类型定义

// ====================================================================
// 配置数据存储 (AI8051U 片内 IAP/EEPROM)
// --------------------------------------------------------------------
// 存储地址: EEPROM 起始地址 0x0000, 连续占用 8 个扇区(4KB)
// ⚠️ 必须在 STC-ISP 下载时把 "EEPROM大小" 设置为 ≥ 4K (建议 8K)。
// 使用 set_eeprom_base 底层接口 (set_eeprom.c 库), 不占用均衡磨损区。
// ====================================================================

#define CONFIG_EEPROM_START     0x0000UL
#define CONFIG_EEPROM_SECTORS   8   // 8 扇区 = 4KB (每扇区 512B)

#define MAX_SERVOS 16
#define MAX_ACTION_STEPS 200

typedef struct {
    uint8_t angles[MAX_SERVOS];
    uint16_t delay_s;
} ActionStep_t;

typedef struct {
    uint32_t magic_number;

    // --- Core System Settings ---
    uint16_t servo_active_mask;

    // 初始角度 (每个舵机的上电默认角度)
    uint8_t initial_angles[MAX_SERVOS];

    // --- Sequence Data ---
    uint8_t recorded_step_count;
    ActionStep_t recorded_sequence[MAX_ACTION_STEPS];

    // --- Environment Settings ---
    uint8_t temperature_threshold;
    uint8_t humidity_threshold;

    // --- Playback Settings ---
    uint8_t playback_speed_index;   /* 0=0.5x, 1=1x, 2=2x */

    // --- Manual Mode Draft (auto-saved on exit) ---
    uint8_t manual_draft_angles[MAX_SERVOS];
    uint8_t manual_draft_valid;     /* 1=draft 中存有有效数据 */

    uint32_t checksum;
} SystemConfig_t;

uint8_t Flash_Write_Config(SystemConfig_t* config);
void Flash_Read_Config(SystemConfig_t* config);
uint8_t Flash_Has_V1_Config(void);

#endif
