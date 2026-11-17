#ifndef __BOOT_PARTITION_H__
#define __BOOT_PARTITION_H__

#include <stdint.h>

/* ============================================================================
 *  F407VGT6（1MB Flash）OTA 分区表
 *
 *  F407 的 Flash 扇区大小是不规则的，擦除必须以「整个扇区」为单位，
 *  所以分区边界必须落在扇区边界上：
 *
 *      Sector 0~3 : 16KB × 4
 *      Sector 4   : 64KB
 *      Sector 5~11: 128KB × 7
 *
 *  0x08000000 +------------------+  Sector 0-1  (16K + 16K)
 *             |  Bootloader 32K  |
 *  0x08008000 +------------------+  Sector 2    (16K)
 *             |  参数区 16K       |
 *  0x0800C000 +------------------+  Sector 3-7  (16K + 64K + 128K×3)
 *             |  App A 464K      |
 *  0x08080000 +------------------+  Sector 8-11 (128K×4)
 *             |  App B 512K      |
 *  0x08100000 +------------------+
 *
 *  约束：App B 区必须 >= App A 区，否则新固件放不下。
 *
 *  ★ 两个 App 分区必须用各自的链接地址分别编译 ★
 *    App 是绝对地址镜像（向量表里的 MSP / Reset_Handler、字面量池、
 *    长跳转表都是链接时算死的），不能一份 bin 两处通用。
 *    Keil 工程里对应两个 target：
 *      f407   -> IROM 0x0800C000 -> f407_A.bin
 *      f407_B -> IROM 0x08080000 -> f407_B.bin
 *    下发配对规则：设备在 A 发 f407_B.bin，设备在 B 发 f407_A.bin。
 *    （踩坑记录：把 A 版 bin 烧到 B 区，跳过去实际执行的还是 A 区代码；
 *      两份内容相同时看起来能跑，一旦 A 被擦就静默 HardFault。）
 * ========================================================================== */

#define BOOT_BASE_ADDR       0x08000000U        /* Bootloader 起始地址 */
#define BOOT_SIZE            (32U  * 1024U)     /* 32KB，Sector 0-1 */

#define PARAM_BASE_ADDR      0x08008000U        /* 参数区起始地址 */
#define PARAM_SIZE           (16U  * 1024U)     /* 16KB，Sector 2 */

#define APP_A_BASE_ADDR      0x0800C000U        /* App A 起始地址 */
#define APP_A_SIZE           (464U * 1024U)     /* 464KB，Sector 3-7 */

#define APP_B_BASE_ADDR      0x08080000U        /* App B 起始地址 */
#define APP_B_SIZE           (512U * 1024U)     /* 512KB，Sector 8-11 */

/* 两个 slot 都放得下的最大固件尺寸（取小的那个） */
#define APP_MAX_SIZE         APP_A_SIZE

/* slot 编号 */
#define SLOT_A               0U
#define SLOT_B               1U

/* slot -> 地址 / 大小。用宏而不是 inline 函数，ARMCC v5 兼容性更稳。 */
#define BOOT_SLOT_ADDR(slot) ((slot) == SLOT_B ? APP_B_BASE_ADDR : APP_A_BASE_ADDR)
#define BOOT_SLOT_SIZE(slot) ((slot) == SLOT_B ? APP_B_SIZE      : APP_A_SIZE)

/* 参数区里的升级标志 */
#define BOOT_FLAG_NONE       0U   /* 无升级任务，直接启动 active_slot */
#define BOOT_FLAG_PENDING    1U   /* 新固件已写入，等待 Bootloader 校验并跳转 */

#endif /* __BOOT_PARTITION_H__ */
