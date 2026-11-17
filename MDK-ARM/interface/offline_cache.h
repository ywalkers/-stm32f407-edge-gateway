#ifndef __OFFLINE_CACHE_H__
#define __OFFLINE_CACHE_H__

#include <stdint.h>

/* ============================================================================
 *  W25Q16 日志式离线缓存（Phase 1：Flash 日志 + 掉电恢复）
 *
 *  背景：原来用 16KB RAM 环形缓冲，掉线期间的数据一切换复位就全丢。
 *        现在把缓存落到 W25Q16（2MB SPI NOR），掉电/复位后可继续补发。
 *
 *  分区（W25Q16 内部地址，不是 MCU 地址）：
 *
 *      0x000000 ~ 0x001000  OTA 头
 *      0x001000 ~ 0x071000  OTA 固件暂存（448KB）
 *      0x071000 ~ 0x0F1000  缓存数据区（512KB，128 个 4KB 扇区）
 *      0x0F1000 ~ 0x0F2000  meta 区 A（4KB）
 *      0x0F2000 ~ 0x0F3000  meta 区 B（4KB）
 *      其余预留
 *
 *  记录布局（每条记录不跨 4KB 扇区）：
 *      [16B 记录头][payload N 字节][2B payload CRC16][补齐到 4 字节]
 *      记录头：magic/ver/topic/len/seq/ts/hdr_crc
 *
 *  扇区头（每个 4KB 扇区前 16B）：
 *      magic/seq/used/crc，用于识别「这个扇区属于本轮日志」。
 *
 *  meta A/B：追加写 checkpoint（32B 一条），保存读写指针和条数。
 *      A/B 交替使用，一个区写满 128 条就擦另一个区切过去，掉电时
 *      取 seq 最大且 CRC 正确的一条恢复。
 *
 *  并发：Push/Peek/Commit 内部用互斥量保护 RAM 状态；
 *        SPI1 总线由 w25q16.c 的递归锁保护（与 OTA 共用）。
 *        抢总线超过 OC_BUS_TIMEOUT_MS 就丢帧计数，不阻塞采集。
 * ========================================================================== */

/* ---- Flash 分区 ---- */
#define OC_DATA_ADDR        0x00071000UL    /* 缓存数据区起始 */
#define OC_DATA_SIZE        0x00080000UL    /* 512KB */
#define OC_SECTOR_SIZE      4096UL
#define OC_SECTOR_COUNT     (OC_DATA_SIZE / OC_SECTOR_SIZE)   /* 128 */

#define OC_META_A_ADDR      0x000F1000UL
#define OC_META_B_ADDR      0x000F2000UL
#define OC_META_SIZE        0x00001000UL    /* 每个 meta 区 4KB */

/* ---- 魔数与版本 ---- */
#define OC_MAGIC_REC        0x3148434FU     /* "OCH1" 记录头 */
#define OC_MAGIC_SECTOR     0x314C4353U     /* "SCL1" 扇区头 */
#define OC_MAGIC_META       0x314D434FU     /* "OCM1" meta */
#define OC_VERSION          1U

/* ---- 记录尺寸 ---- */
#define OC_HDR_SIZE         16U             /* 记录头 */
#define OC_CRC_SIZE         2U              /* payload CRC16 */
#define OC_MAX_PAYLOAD      144U            /* 与 UPLOAD_ITEM.data 对齐 */
                                            /* Push 最多存 OC_MAX_PAYLOAD-1 字节， */
                                            /* 留 1 字节给读回时的 '\0' */

#define OC_META_ENTRY_SIZE  32U
#define OC_META_ENTRIES     (OC_META_SIZE / OC_META_ENTRY_SIZE)   /* 128 */

/* ---- 策略参数 ---- */
#define OC_BUS_TIMEOUT_MS   50U             /* 抢 SPI 总线最长等待，超时丢帧 */
#define OC_META_EVERY       16U             /* 每写这么多条落一次 meta */

/* 上电挂载：扫描 meta -> 校验读写指针 -> 修正掉电时写了一半的尾部。
 * 必须在调度器启动前调用一次（App_RTOS_Init 内）。 */
void Offline_Cache_Init(void);

/* 追加一条记录。topic 0=sensor 1=alarm 2=stats；data 是 '\0' 结尾 JSON。
 * 返回 1 成功，0 失败（缓存满 / 抢锁超时 / 参数非法），失败计入丢弃计数。 */
uint8_t Offline_Cache_Push(uint8_t topic, const char *data);

/* 读取当前读指针处的记录但不推进（Peek）。
 * topic/buf 可为 NULL（buf 不可为 NULL）；buf 可写 max_len 字节，
 * 函数会在 buf[len] 补 '\0'，所以调用方需要保证 buf 至少 max_len+1。
 * 返回 payload 长度，0 表示缓存空或读取失败。
 * 注意：Peek 之后必须调用 Commit 才会推进读指针。 */
uint16_t Offline_Cache_Peek(uint8_t *topic, char *buf, uint16_t max_len);

/* 提交上一次 Peek 的记录（PUBACK 之后调用），推进读指针。 */
void Offline_Cache_Commit(void);

/* Peek + Commit 的组合：读出来就立刻推进读指针。
 * ！！不要在“发出去等确认”的场景用！！记录一旦 Pop 出来而 PUBACK 没回来，
 * 这条数据就永久丢了。上行补发请用 Peek -> 发送 -> 收到 PUBACK 再 Commit
 * （见 app_rtos.c 的 Mqtt_Upload_Task）。保留它只是为了“读一条就作废”的场合。 */
uint16_t Offline_Cache_Pop(uint8_t *topic, char *buf, uint16_t max_len);

/* 当前待发送条数 / 累计丢弃条数 / 已写入总序号的低 16 位 */
uint32_t Offline_Cache_Count(void);
uint32_t Offline_Cache_DropCount(void);

/* 一次性取统计，便于上电打印 */
void Offline_Cache_Stats(uint32_t *count, uint32_t *drops, uint32_t *seq);

#endif /* __OFFLINE_CACHE_H__ */
