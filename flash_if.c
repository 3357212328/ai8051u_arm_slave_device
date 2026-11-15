#include "flash_if.h"
#include "set_eeprom.h"
#include <string.h>

// IAP_TPS 必须 = 系统主频(MHz), 否则EEPROM读写时序错误
// (set_eeprom.c 只在 set_eeprom_mode 中设置, 这里用 set_eeprom_base 需自行设置)
#define IAP_TPS_SET()   IAP_TPS = 40   /* 40MHz 主频 */

// 定义一个幻数，用于在读取时验证数据是否有效
// v2: 新增 playback_speed_index + manual_draft 字段
#define CONFIG_MAGIC_NUMBER     0xA5A5B4B5UL

// v1 旧版本 (initial_angles 之前): 用于向后兼容
#define CONFIG_MAGIC_NUMBER_V1  0xA5A5B4B4UL

/**
 * @brief  计算配置数据的校验和
 */
static uint32_t Calculate_Checksum(SystemConfig_t* config)
{
    uint32_t checksum = 0;
    uint8_t* p_data = (uint8_t*)config;
    uint32_t i;
    // 计算除checksum字段外的所有字节
    for (i = 0; i < (sizeof(SystemConfig_t) - sizeof(uint32_t)); i++)
    {
        checksum += p_data[i];
    }
    return checksum;
}

/**
 * @brief  将配置数据写入EEPROM (先擦除后写入)
 */
uint8_t Flash_Write_Config(SystemConfig_t* config)
{
    IAP_TPS_SET();
    config->magic_number = CONFIG_MAGIC_NUMBER;
    config->checksum = Calculate_Checksum(config);

    set_eeprom_base(Erase_Sectors, CONFIG_EEPROM_START, CONFIG_EEPROM_SECTORS);
    set_eeprom_base(Write_Buff, CONFIG_EEPROM_START, (char*)config, (int)sizeof(SystemConfig_t));

    return 0;
}

/**
 * @brief  从EEPROM中读取配置数据
 */
void Flash_Read_Config(SystemConfig_t* config)
{
    IAP_TPS_SET();
    set_eeprom_base(Read_Buff, CONFIG_EEPROM_START, (char*)config, (int)sizeof(SystemConfig_t));

    if (config->magic_number != CONFIG_MAGIC_NUMBER ||
        config->checksum != Calculate_Checksum(config))
    {
        config->magic_number = 0;
    }
}

/**
 * @brief  检测EEPROM中是否存有v1格式的旧配置
 * @return 1=是, 0=否
 * @note   用于首次从v1升级到v2时, 复用v1的设置并补齐新字段
 */
uint8_t Flash_Has_V1_Config(void)
{
    SystemConfig_t xdata tmp;
    IAP_TPS_SET();
    set_eeprom_base(Read_Buff, CONFIG_EEPROM_START, (char*)&tmp, (int)sizeof(SystemConfig_t));
    return (tmp.magic_number == CONFIG_MAGIC_NUMBER_V1);
}
