#ifndef __BOOT_PARAM_H__
#define __BOOT_PARAM_H__

#include <stdint.h>
#include "boot_partition.h"

/* ============================================================================
 *  参数区（Sector 2，0x08008000，16KB）
 *
 *  Bootloader 和 App 都要读写这块，用来跨复位传递状态：
 *    - 当前该启动哪个 slot
 *    - 是否有待验证的新固件
 *    - 新固件启动失败了几次（超阈值回滚）
 *
 *  写入流程：擦整个 Sector 2 -> 写结构体 -> 写 CRC。
 *  读的时候校验 magic + CRC，任一不对就当作"无参数"，用默认值。
 * ========================================================================== */

#define BOOT_PARAM_MAGIC   0x4F544131U   /* "OTA1" */
#define BOOT_PARAM_VER     1U

typedef struct
{
    uint32_t magic;          /* BOOT_PARAM_MAGIC */
    uint32_t ver;            /* 结构体版本 */

    uint32_t upgrade_flag;   /* BOOT_FLAG_NONE / BOOT_FLAG_PENDING */
    uint32_t active_slot;    /* 当前应该跑哪个 slot（SLOT_A / SLOT_B） */
    uint32_t target_slot;    /* 待验证的新固件所在 slot */
    uint32_t target_size;    /* 新固件字节数 */
    uint32_t target_crc;     /* 新固件 CRC32 */

    uint32_t boot_count;     /* 新固件已尝试启动次数 */
    uint32_t boot_max;       /* 超过这个次数就回滚 */

    uint32_t fw_version;     /* 当前固件版本号，上报给云端用 */

    uint32_t rsv[5];         /* 预留，保持结构体 64 字节对齐 */
    uint32_t crc;            /* 上面所有字段的 CRC32 */
} boot_param_t;

/* CRC32（多项式 0xEDB88320，与 zlib/标准 CRC32 一致） */
uint32_t boot_crc32(const uint8_t *data, uint32_t len);

/* 增量版本：分块喂数据时用。
 * 用法：crc = 0xFFFFFFFF；每块 crc = boot_crc32_update(crc, buf, n)；
 *       最后结果 = ~crc。OTA 边下载边算校验就靠它，不用整包缓存。 */
uint32_t boot_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len);

/* 填一套默认值：跑 App A、无升级任务 */
void boot_param_default(boot_param_t *p);

/* 从参数区读取并校验。返回 1 = 有效，0 = 无效/全 0xFF */
uint8_t boot_param_read(boot_param_t *p);

/* 擦除参数区扇区并写入。返回 1 = 成功 */
uint8_t boot_param_write(const boot_param_t *p);

/* 只擦除参数区 */
uint8_t boot_param_erase(void);

/* App 自检通过后调用：清启动计数、清升级标志，告诉 Bootloader "这次启动成功了"。
 * 内部先读再判断，无事可做时不擦 Flash —— 否则每次上电都擦一次，寿命吃不消。 */
void boot_param_mark_ok(void);

#endif /* __BOOT_PARAM_H__ */
