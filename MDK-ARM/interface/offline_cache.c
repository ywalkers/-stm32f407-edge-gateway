/* ============================================================================
 *  W25Q16 日志式离线缓存（Phase 1）
 *
 *  设计要点：
 *    1. 追加写：数据区按 4KB 扇区顺序使用，扇区内从 16 字节扇区头之后开始
 *       顺序追加记录，记录不跨扇区；写满一个扇区就擦下一个扇区继续。
 *    2. 环形使用：128 个扇区循环使用，写指针追上读指针时判满，丢新帧并计数。
 *    3. 掉电恢复：读写指针 + 条数定期写进 meta A/B（各 4KB，32B 一条追加写）。
 *       上电取 seq 最大且 CRC 正确的 meta，再从读指针向后扫描，修正掉电时
 *       写了一半的尾部记录。
 *    4. 与 OTA 共用 SPI1：所有 Flash 访问都走 w25q16.c 的递归锁；
 *       缓存侧先以 OC_BUS_TIMEOUT_MS 抢锁，抢不到就丢帧，绝不阻塞 DataProc。
 *
 *  记录格式：
 *      [16B hdr][payload][2B crc16][padding -> 4B 对齐]
 *      16B hdr = magic(4) ver(1) topic(1) len(2) seq(2) ts(4) hdr_crc(2)
 *
 *  Phase 1 范围：追加写 + 开机扫描恢复 + 基本 Peek/Commit 接口。
 *  跨重连「PUBACK 后才 Commit」的语义留给 Phase 2 在 Mqtt_Upload_Task 里接。
 * ========================================================================== */

#include "offline_cache.h"
#include "w25q16.h"
#include "time_sync.h"             /* Time_Sync_Now()：记录头的采集时间戳 */
#include "../boot/boot_param.h"    /* boot_crc32 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* 只在初始化（调度器启动前）使用，运行期不打印，避免与 Print_Mutex 嵌套 */
#define OC_LOG(fmt, ...)  printf("[CACHE] " fmt "\r\n", ##__VA_ARGS__)

/* ARM Compiler 5 只认 #pragma pack(n)，用 #pragma pack() 恢复默认 */
#pragma pack(1)
typedef struct
{
    uint32_t magic;      /* OC_MAGIC_REC */
    uint8_t  ver;        /* OC_VERSION */
    uint8_t  topic;      /* 0=sensor 1=alarm 2=stats */
    uint16_t len;        /* payload 长度 */
    uint16_t seq;        /* 记录序号（16 位回绕） */
    uint32_t ts;         /* 采集时刻的 Unix 秒（未对时为 0） */
    uint16_t hdr_crc;    /* 前 14 字节的 CRC16 */
} oc_hdr_t;

typedef struct
{
    uint32_t magic;      /* OC_MAGIC_SECTOR */
    uint32_t seq;        /* 扇区序号（写时的记录序号快照） */
    uint32_t used;       /* 本扇区已用字节数 */
    uint16_t crc;        /* 前 12 字节的 CRC16 */
    uint16_t rsv;
} oc_sector_hdr_t;

typedef struct
{
    uint32_t magic;      /* OC_MAGIC_META */
    uint32_t seq;        /* checkpoint 序号，越大越新 */
    uint32_t wr_sector;
    uint32_t wr_off;
    uint32_t rd_sector;
    uint32_t rd_off;
    uint16_t count;      /* 待发送条数：容量上限 128*204 = 26112，16 位够用 */
    uint16_t rec_seq;    /* 记录序号快照，重启后接着递增，避免 seq 倒退 */
    uint32_t crc;        /* 前 28 字节的 CRC32 */
} oc_meta_t;
#pragma pack()

/* 结构体尺寸自检：ARMCC 下 pack 生效后应严格等于 16/16/32 */
typedef char oc_hdr_size_check[(sizeof(oc_hdr_t) == OC_HDR_SIZE) ? 1 : -1];
typedef char oc_sector_size_check[(sizeof(oc_sector_hdr_t) == 16U) ? 1 : -1];
typedef char oc_meta_size_check[(sizeof(oc_meta_t) == OC_META_ENTRY_SIZE) ? 1 : -1];

/* ---------------------------- 状态 ---------------------------- */

static uint32_t s_wr_sector;      /* 当前写扇区 */
static uint32_t s_wr_off;         /* 当前写扇区内偏移（下一条记录的位置） */
static uint32_t s_rd_sector;      /* 当前读扇区 */
static uint32_t s_rd_off;         /* 当前读扇区内偏移 */
static uint32_t s_count;          /* 待发送记录数 */
static uint16_t s_seq;            /* 记录序号 */

static uint32_t s_meta_seq;       /* 最近一次 checkpoint 序号 */
static uint8_t  s_meta_zone;      /* 当前 checkpoint 写在哪个区：0=A 1=B */
static uint32_t s_meta_off;       /* 该区下一个空 slot 的偏移 */

static uint32_t s_dirty;          /* 距上次 checkpoint 又写了多少条 */
static uint32_t s_drops;          /* 累计丢弃条数 */

static uint16_t s_last_size;      /* 最近一次 Peek 的记录总长（供 Commit） */
static uint8_t  s_last_valid;

static SemaphoreHandle_t s_mutex = NULL;

/* ---------------------------- 小工具 ---------------------------- */

static uint16_t oc_crc16(const uint8_t *p, uint32_t n)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;

    while (n-- > 0U)
    {
        crc ^= (uint16_t)((uint16_t)(*p++) << 8);
        for (i = 0U; i < 8U; i++)
        {
            if (crc & 0x8000U)
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021U);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint32_t oc_sec_base(uint32_t sec)
{
    return OC_DATA_ADDR + sec * OC_SECTOR_SIZE;
}

static uint32_t oc_rec_addr(uint32_t sec, uint32_t off)
{
    return oc_sec_base(sec) + off;
}

static uint32_t oc_meta_base(uint8_t zone)
{
    return (zone == 0U) ? OC_META_A_ADDR : OC_META_B_ADDR;
}

static uint32_t oc_rec_size(uint16_t len)
{
    return ((uint32_t)OC_HDR_SIZE + (uint32_t)len + (uint32_t)OC_CRC_SIZE + 3U) & ~3U;
}

/* 校验一个 16 字节记录头，成功时回填结构体和整条记录长度 */
static int oc_hdr_valid(const uint8_t *raw, oc_hdr_t *h, uint16_t *rec)
{
    memcpy(h, raw, OC_HDR_SIZE);

    if (h->magic != OC_MAGIC_REC) return -1;
    if (h->ver != OC_VERSION)     return -1;
    if (h->len > OC_MAX_PAYLOAD)  return -1;
    if (oc_crc16(raw, 14U) != h->hdr_crc) return -1;

    *rec = (uint16_t)oc_rec_size(h->len);
    return 0;
}

/* 读一条完整记录（含 payload CRC 校验）。
 * max_len 表示 buf 可写的最大 payload 字节数（不含结尾 '\0'）。
 * 返回 0 成功；-1 头损坏；-2 长度超过 buf；-3 payload CRC 错。
 * 头校验通过后无论返回值如何，*rec 都会填上整条记录长度。 */
static int oc_read_rec(uint32_t sec, uint32_t off, oc_hdr_t *h,
                       uint8_t *payload, uint16_t max_len, uint16_t *rec)
{
    uint8_t raw[OC_HDR_SIZE];
    uint8_t crc_buf[2];
    uint16_t crc;

    w25q16_read(oc_rec_addr(sec, off), raw, OC_HDR_SIZE);
    if (oc_hdr_valid(raw, h, rec) != 0) return -1;

    if (h->len > max_len) return -2;

    w25q16_read(oc_rec_addr(sec, off) + OC_HDR_SIZE, payload, h->len);
    w25q16_read(oc_rec_addr(sec, off) + OC_HDR_SIZE + h->len, crc_buf, 2);

    crc = (uint16_t)((uint16_t)crc_buf[0] | ((uint16_t)crc_buf[1] << 8));
    if (oc_crc16(payload, h->len) != crc) return -3;

    return 0;
}

/* 只校验记录头（开机扫描用，不读 payload，快） */
static int oc_head_ok(uint32_t sec, uint32_t off, oc_hdr_t *h, uint16_t *rec)
{
    uint8_t raw[OC_HDR_SIZE];

    w25q16_read(oc_rec_addr(sec, off), raw, OC_HDR_SIZE);
    return oc_hdr_valid(raw, h, rec);
}

/* ---------------------------- 写入原语 ---------------------------- */

static int oc_write_sector_hdr(uint32_t sec)
{
    oc_sector_hdr_t sh;

    sh.magic = OC_MAGIC_SECTOR;
    sh.seq   = (uint32_t)s_seq;
    sh.used  = OC_HDR_SIZE;
    sh.crc   = oc_crc16((const uint8_t *)&sh, 12U);
    sh.rsv   = 0U;

    return w25q16_write(oc_sec_base(sec), &sh, sizeof(sh));
}

static int oc_write_rec(uint32_t sec, uint32_t off, uint8_t topic,
                        const uint8_t *payload, uint16_t len)
{
    uint8_t  buf[OC_HDR_SIZE + OC_MAX_PAYLOAD + OC_CRC_SIZE + 4U];
    oc_hdr_t h;
    uint16_t crc;
    uint32_t rec;

    memset(buf, 0xFF, sizeof(buf));     /* 对齐填充保持 0xFF，不额外擦 */

    h.magic   = OC_MAGIC_REC;
    h.ver     = OC_VERSION;
    h.topic   = topic;
    h.len     = len;
    h.seq     = s_seq;
    /* 采集时刻的时间戳：入缓存时就固定，补发时才是原始时间而不是发送时间 */
    h.ts      = Time_Sync_Now();
    h.hdr_crc = oc_crc16((const uint8_t *)&h, 14U);

    memcpy(buf, &h, OC_HDR_SIZE);
    memcpy(buf + OC_HDR_SIZE, payload, len);

    crc = oc_crc16(payload, len);
    buf[OC_HDR_SIZE + len]      = (uint8_t)(crc & 0xFFU);
    buf[OC_HDR_SIZE + len + 1U] = (uint8_t)(crc >> 8);

    rec = oc_rec_size(len);
    return w25q16_write(oc_rec_addr(sec, off), buf, rec);
}

/* ---------------------------- meta ---------------------------- */

/* 扫描一个 meta 区，返回最新一条（seq 最大且 CRC 有效） */
static uint8_t oc_meta_scan(uint8_t zone, oc_meta_t *best, uint32_t *best_idx)
{
    oc_meta_t m;
    uint8_t   found = 0U;
    uint32_t  i;
    uint32_t  base = oc_meta_base(zone);

    for (i = 0U; i < OC_META_ENTRIES; i++)
    {
        w25q16_read(base + i * OC_META_ENTRY_SIZE, &m, sizeof(m));

        if (m.magic != OC_MAGIC_META) continue;
        if (boot_crc32((const uint8_t *)&m, 28U) != m.crc) continue;

        if (!found || (int32_t)(m.seq - best->seq) > 0)
        {
            *best = m;
            *best_idx = i;
            found = 1U;
        }
    }
    return found;
}

/* 把当前读写状态落一条 checkpoint。调用者需已持有 s_mutex 和 SPI 锁。 */
static void oc_checkpoint(void)
{
    oc_meta_t m;
    uint32_t  base;

    memset(&m, 0, sizeof(m));
    m.magic     = OC_MAGIC_META;
    m.seq       = ++s_meta_seq;
    m.wr_sector = s_wr_sector;
    m.wr_off    = s_wr_off;
    m.rd_sector = s_rd_sector;
    m.rd_off    = s_rd_off;
    m.count     = (uint16_t)s_count;
    m.rec_seq   = s_seq;
    m.crc       = boot_crc32((const uint8_t *)&m, 28U);

    /* 当前区写满：擦另一个区，切过去从 0 开始写。
     * 另一个区是上一次的旧 checkpoint，擦掉不影响当前状态。 */
    if (s_meta_off + OC_META_ENTRY_SIZE > OC_META_SIZE)
    {
        uint8_t nz = (uint8_t)(s_meta_zone ^ 1U);

        if (w25q16_erase_sector(oc_meta_base(nz)) != 0)
        {
            /* 擦失败就原地重写旧的 slot，宁可丢 checkpoint 也不破坏 A/B 结构 */
            s_meta_off = OC_META_SIZE - OC_META_ENTRY_SIZE;
        }
        else
        {
            s_meta_zone = nz;
            s_meta_off  = 0U;
        }
    }

    base = oc_meta_base(s_meta_zone);
    w25q16_write(base + s_meta_off, &m, sizeof(m));
    s_meta_off += OC_META_ENTRY_SIZE;
}

/* ---------------------------- 格式化 / 恢复 ---------------------------- */

/* 首次使用或 meta 全坏：从扇区 0 重新开始 */
static void oc_format(void)
{
    w25q16_erase_sector(oc_sec_base(0U));
    w25q16_erase_sector(OC_META_A_ADDR);
    w25q16_erase_sector(OC_META_B_ADDR);

    s_wr_sector = 0U;
    s_wr_off    = OC_HDR_SIZE;
    s_rd_sector = 0U;
    s_rd_off    = OC_HDR_SIZE;
    s_count     = 0U;
    s_seq       = 0U;
    s_meta_seq  = 0U;
    s_meta_zone = 0U;
    s_meta_off  = 0U;

    oc_write_sector_hdr(0U);
    oc_checkpoint();
}

/* 从 meta 记录的写指针向后扫描，补回「已写入但没来得及落 checkpoint」的记录，
 * 并把写指针修正到第一条无效记录处。
 *
 * 扫描上限 = meta.count + OC_META_EVERY：
 * meta 最多落后 OC_META_EVERY 条，再加一圈环形覆盖的旧数据就可能被误读，
 * 用这个上限卡住，避免把上一轮的旧记录当成新数据。
 */
static void oc_recover_tail(void)
{
    uint32_t sec, off, cnt, limit, guard = 0U;
    uint8_t  raw[OC_HDR_SIZE];
    oc_hdr_t h;
    uint16_t rec;

    if (s_wr_off >= OC_SECTOR_SIZE)
    {
        s_wr_sector = (s_wr_sector + 1U) % OC_SECTOR_COUNT;
        s_wr_off    = OC_HDR_SIZE;
    }
    if (s_wr_off < OC_HDR_SIZE) s_wr_off = OC_HDR_SIZE;

    if (s_count == 0U)
    {
        /* 空缓存：写指针直接跟读指针对齐 */
        s_wr_sector = s_rd_sector;
        s_wr_off    = s_rd_off;
        return;
    }

    /* 读指针指向的记录头必须有效，否则视为缓存损坏，整段丢弃 */
    if (oc_head_ok(s_rd_sector, s_rd_off, &h, &rec) != 0)
    {
        OC_LOG("read cursor invalid, drop pending records");
        s_count     = 0U;
        s_wr_sector = s_rd_sector;
        s_wr_off    = s_rd_off;
        return;
    }

    sec   = s_rd_sector;
    off   = s_rd_off;
    cnt   = 0U;
    limit = s_count + OC_META_EVERY;

    /* 硬上限按容量算：最短记录 20B，单扇区最多 204 条，128 扇区 = 26112 条。
     * 取 32768 留余量，保证任何情况下都不会被 guard 提前截断。 */
    while (cnt < limit && guard++ < (OC_SECTOR_COUNT * 256U))
    {
        /* 环形绕回读指针：说明已经扫了一整圈，停 */
        if (cnt > 0U && sec == s_rd_sector && off == s_rd_off) break;

        w25q16_read(oc_rec_addr(sec, off), raw, OC_HDR_SIZE);
        if (oc_hdr_valid(raw, &h, &rec) != 0) break;    /* 未写区域或半写记录：停 */

        cnt++;
        off += rec;
        if (off >= OC_SECTOR_SIZE)
        {
            sec = (sec + 1U) % OC_SECTOR_COUNT;
            off = OC_HDR_SIZE;
        }
    }

    /* 停在半写记录上时，该位置不是 0xFF，不能直接覆盖（NOR 只能 1->0）。
     * 简单做法：放弃本扇区剩余空间，跳到下一扇区并擦好。 */
    if (cnt < limit && off < OC_SECTOR_SIZE)
    {
        uint8_t b = 0xFFU;

        w25q16_read(oc_rec_addr(sec, off), &b, 1);
        if (b != 0xFFU)
        {
            uint32_t ns = (sec + 1U) % OC_SECTOR_COUNT;

            if (s_count > 0U && ns == s_rd_sector)
            {
                /* 没有可用扇区了，整段丢弃 */
                s_count     = 0U;
                s_wr_sector = s_rd_sector;
                s_wr_off    = s_rd_off;
                return;
            }

            w25q16_erase_sector(oc_sec_base(ns));
            oc_write_sector_hdr(ns);

            s_wr_sector = ns;
            s_wr_off    = OC_HDR_SIZE;
            s_count     = cnt;
            return;
        }
    }

    s_count     = cnt;
    s_wr_sector = sec;
    s_wr_off    = off;
}

/* ---------------------------- 对外接口 ---------------------------- */

void Offline_Cache_Init(void)
{
    oc_meta_t ma, mb;
    oc_meta_t best;
    uint32_t  ia = 0U, ib = 0U;
    uint8_t   va, vb, have = 0U;

    if (s_mutex == NULL)
        s_mutex = xSemaphoreCreateMutex();

    w25q16_lock_init();

    if (s_mutex == NULL)
    {
        OC_LOG("mutex create failed");
        return;
    }

    if (!w25q16_lock_take(1000U))
    {
        OC_LOG("bus busy, mount skipped");
        return;
    }

    va = oc_meta_scan(0U, &ma, &ia);
    vb = oc_meta_scan(1U, &mb, &ib);

    if (va || vb)
    {
        have = 1U;

        if (va && vb)
        {
            if ((int32_t)(ma.seq - mb.seq) >= 0)
            {
                best = ma; s_meta_zone = 0U; s_meta_off = (ia + 1U) * OC_META_ENTRY_SIZE;
            }
            else
            {
                best = mb; s_meta_zone = 1U; s_meta_off = (ib + 1U) * OC_META_ENTRY_SIZE;
            }
        }
        else if (va)
        {
            best = ma; s_meta_zone = 0U; s_meta_off = (ia + 1U) * OC_META_ENTRY_SIZE;
        }
        else
        {
            best = mb; s_meta_zone = 1U; s_meta_off = (ib + 1U) * OC_META_ENTRY_SIZE;
        }

        s_meta_seq  = best.seq;
        s_wr_sector = best.wr_sector % OC_SECTOR_COUNT;
        s_wr_off    = best.wr_off;
        s_rd_sector = best.rd_sector % OC_SECTOR_COUNT;
        s_rd_off    = best.rd_off;
        s_count     = best.count;
        s_seq       = best.rec_seq;

        oc_recover_tail();
    }

    if (!have)
    {
        OC_LOG("no valid meta, format flash cache");
        oc_format();
    }
    else
    {
        /* 恢复后立即落一次 checkpoint，把修正过的指针固化下来 */
        oc_checkpoint();
    }

    w25q16_lock_give();

    OC_LOG("mounted: %lu pending, wr=%lu/%lu rd=%lu/%lu",
           (unsigned long)s_count,
           (unsigned long)s_wr_sector, (unsigned long)s_wr_off,
           (unsigned long)s_rd_sector, (unsigned long)s_rd_off);
}

uint8_t Offline_Cache_Push(uint8_t topic, const char *data)
{
    uint16_t len;
    uint32_t rec;
    uint8_t  ok = 0U;

    if (data == NULL || s_mutex == NULL) return 0U;

    len = (uint16_t)strlen(data);
    /* 收端缓冲区固定 OC_MAX_PAYLOAD 字节，还要留 '\0'，
     * 所以这里最多存 OC_MAX_PAYLOAD-1 字节，保证 Push 进去的一定能 Peek 出来。 */
    if (len == 0U || len >= OC_MAX_PAYLOAD) return 0U;

    rec = oc_rec_size(len);

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
    {
        s_drops++;
        return 0U;
    }

    /* 先抢 SPI 总线：OTA 正在擦写时最多等 OC_BUS_TIMEOUT_MS，超时就丢帧 */
    if (!w25q16_lock_take(OC_BUS_TIMEOUT_MS))
    {
        s_drops++;
        xSemaphoreGive(s_mutex);
        return 0U;
    }

    /* 本扇区剩余空间不够：换下一个扇区 */
    if (s_wr_off + rec > OC_SECTOR_SIZE)
    {
        uint32_t ns = (s_wr_sector + 1U) % OC_SECTOR_COUNT;

        if (s_count > 0U && ns == s_rd_sector)
        {
            /* 写指针追上读指针：缓存满，丢新帧 */
            s_drops++;
            w25q16_lock_give();
            xSemaphoreGive(s_mutex);
            return 0U;
        }

        if (w25q16_erase_sector(oc_sec_base(ns)) != 0 ||
            oc_write_sector_hdr(ns) != 0)
        {
            s_drops++;
            w25q16_lock_give();
            xSemaphoreGive(s_mutex);
            return 0U;
        }

        s_wr_sector = ns;
        s_wr_off    = OC_HDR_SIZE;
    }
    else if (s_wr_off == OC_HDR_SIZE)
    {
        /* 刚进入一个扇区：确认扇区头存在且有效（掉电恢复后可能出现头缺失） */
        oc_sector_hdr_t sh;

        w25q16_read(oc_sec_base(s_wr_sector), &sh, sizeof(sh));
        if (sh.magic != OC_MAGIC_SECTOR ||
            oc_crc16((const uint8_t *)&sh, 12U) != sh.crc)
        {
            if (w25q16_erase_sector(oc_sec_base(s_wr_sector)) != 0 ||
                oc_write_sector_hdr(s_wr_sector) != 0)
            {
                s_drops++;
                w25q16_lock_give();
                xSemaphoreGive(s_mutex);
                return 0U;
            }
        }
    }

    if (oc_write_rec(s_wr_sector, s_wr_off, topic,
                     (const uint8_t *)data, len) == 0)
    {
        s_wr_off += rec;
        s_count++;
        s_seq++;
        ok = 1U;

        if (++s_dirty >= OC_META_EVERY)
        {
            oc_checkpoint();
            s_dirty = 0U;
        }
    }
    else
    {
        s_drops++;
    }

    w25q16_lock_give();
    xSemaphoreGive(s_mutex);
    return ok;
}

uint16_t Offline_Cache_Peek(uint8_t *topic, char *buf, uint16_t max_len)
{
    oc_hdr_t h;
    uint16_t rec = 0U;
    uint16_t out = 0U;
    int      rc;
    uint32_t guard = 0U;

    if (s_mutex == NULL || buf == NULL || max_len == 0U) return 0U;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return 0U;
    if (!w25q16_lock_take(OC_BUS_TIMEOUT_MS))
    {
        xSemaphoreGive(s_mutex);
        return 0U;
    }

    while (s_count > 0U && guard++ < 32U)
    {
        rc = oc_read_rec(s_rd_sector, s_rd_off, &h,
                         (uint8_t *)buf, max_len, &rec);

        if (rc == 0)
        {
            if (topic != NULL) *topic = h.topic;
            buf[h.len] = '\0';
            s_last_size  = rec;
            s_last_valid = 1U;
            out = h.len;
            break;
        }

        if (rc == -1)
        {
            /* 记录头都坏了，无法定位下一条：整段缓存作废 */
            s_count     = 0U;
            s_wr_sector = s_rd_sector;
            s_wr_off    = s_rd_off;
            break;
        }

        /* payload CRC 坏或长度超缓冲：跳过这条（rec 已由头算出） */
        s_rd_off += rec;
        if (s_rd_off >= OC_SECTOR_SIZE)
        {
            s_rd_sector = (s_rd_sector + 1U) % OC_SECTOR_COUNT;
            s_rd_off    = OC_HDR_SIZE;
        }
        if (s_count > 0U) s_count--;
    }

    w25q16_lock_give();
    xSemaphoreGive(s_mutex);
    return out;
}

void Offline_Cache_Commit(void)
{
    if (s_mutex == NULL) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

    if (!s_last_valid)
    {
        xSemaphoreGive(s_mutex);
        return;
    }
    s_last_valid = 0U;

    if (!w25q16_lock_take(OC_BUS_TIMEOUT_MS))
    {
        xSemaphoreGive(s_mutex);
        return;
    }

    s_rd_off += s_last_size;
    if (s_rd_off >= OC_SECTOR_SIZE)
    {
        s_rd_sector = (s_rd_sector + 1U) % OC_SECTOR_COUNT;
        s_rd_off    = OC_HDR_SIZE;
    }
    if (s_count > 0U) s_count--;

    if (++s_dirty >= OC_META_EVERY)
    {
        oc_checkpoint();
        s_dirty = 0U;
    }

    w25q16_lock_give();
    xSemaphoreGive(s_mutex);
}

uint16_t Offline_Cache_Pop(uint8_t *topic, char *buf, uint16_t max_len)
{
    uint16_t n = Offline_Cache_Peek(topic, buf, max_len);

    if (n > 0U) Offline_Cache_Commit();
    return n;
}

uint32_t Offline_Cache_Count(void)
{
    return s_count;
}

uint32_t Offline_Cache_DropCount(void)
{
    return s_drops;
}

void Offline_Cache_Stats(uint32_t *count, uint32_t *drops, uint32_t *seq)
{
    if (count != NULL) *count = s_count;
    if (drops != NULL) *drops = s_drops;
    if (seq   != NULL) *seq   = (uint32_t)s_seq;
}
