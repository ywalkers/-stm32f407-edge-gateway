#include "boot_param.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* ============================================================================
 *  CRC32：多项式 0xEDB88320（reflected，与 zlib 一致）
 *  逐位实现，不用查表，省 1KB RAM；464KB 固件校验约 0.2s，可以接受。
 * ========================================================================== */
uint32_t boot_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    uint32_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return crc;
}

uint32_t boot_crc32(const uint8_t *data, uint32_t len)
{
    return ~boot_crc32_update(0xFFFFFFFFU, data, len);
}

void boot_param_default(boot_param_t *p)
{
    memset(p, 0, sizeof(*p));

    p->magic        = BOOT_PARAM_MAGIC;
    p->ver          = BOOT_PARAM_VER;
    p->upgrade_flag = BOOT_FLAG_NONE;
    p->active_slot  = SLOT_A;
    p->target_slot  = SLOT_A;
    p->target_size  = 0;
    p->target_crc   = 0;
    p->boot_count   = 0;
    p->boot_max     = 3;            /* 新固件最多尝试启动 3 次 */
    p->fw_version   = 0x01000000U;  /* 1.0.0 */

    p->crc = boot_crc32((const uint8_t *)p, sizeof(*p) - sizeof(uint32_t));
}

uint8_t boot_param_read(boot_param_t *p)
{
    /* Flash 可以直接按地址读；用 memcpy 拷出来，
     * 避免结构体指针未对齐访问（Cortex-M4 允许，但编译器可能生成非最优代码） */
    memcpy(p, (const void *)PARAM_BASE_ADDR, sizeof(*p));

    /* 从来没写过：全 0xFF */
    if (p->magic != BOOT_PARAM_MAGIC) return 0;
    if (p->ver   != BOOT_PARAM_VER)   return 0;

    if (boot_crc32((const uint8_t *)p, sizeof(*p) - sizeof(uint32_t)) != p->crc)
        return 0;

    /* 防御性检查：slot 值只能是 0/1 */
    if (p->active_slot > SLOT_B || p->target_slot > SLOT_B) return 0;

    return 1;
}

void boot_param_mark_ok(void)
{
    boot_param_t p;

    if (!boot_param_read(&p)) return;

    /* 本来就是干净的（正常启动的常态），不要白擦一次 Flash */
    if (p.boot_count == 0U && p.upgrade_flag == BOOT_FLAG_NONE) return;

    p.boot_count   = 0;
    p.upgrade_flag = BOOT_FLAG_NONE;
    boot_param_write(&p);
}

uint8_t boot_param_erase(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_err = 0;
    uint8_t  ok;

    HAL_FLASH_Unlock();

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /* 2.7V ~ 3.6V */
    erase.Sector       = FLASH_SECTOR_2;          /* 0x08008000 */
    erase.NbSectors    = 1;

    ok = (HAL_FLASHEx_Erase(&erase, &sector_err) == HAL_OK) ? 1 : 0;

    HAL_FLASH_Lock();
    return ok;
}

uint8_t boot_param_write(const boot_param_t *p)
{
    boot_param_t tmp = *p;
    uint32_t i;

    /* 写之前先把 CRC 算好，保证存进去的是自洽的一份 */
    tmp.crc = boot_crc32((const uint8_t *)&tmp, sizeof(tmp) - sizeof(uint32_t));

    if (!boot_param_erase()) return 0;

    HAL_FLASH_Unlock();

    for (i = 0; i < sizeof(tmp) / 4U; i++)
    {
        uint32_t word;
        memcpy(&word, ((const uint8_t *)&tmp) + i * 4U, 4U);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              PARAM_BASE_ADDR + i * 4U, word) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0;
        }
    }

    HAL_FLASH_Lock();
    return 1;
}
