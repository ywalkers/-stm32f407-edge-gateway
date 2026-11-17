/* ============================================================================
 *  OTA 固件下载：ESP8266 (AT) + HTTP/1.0 GET -> W25Q16 暂存区
 *
 *  数据流：
 *      USART3 中断  ->  esp_raw_buf 环形缓冲（esp8266.c）
 *                    -> 任务里 ESP8266_ReadRaw()
 *                    -> 本文件解析 "+IPD,<len>:<data>"
 *                    -> 跳过 HTTP 响应头
 *                    -> 4KB 一批写进 W25Q16，同时增量算 CRC32
 *
 *  为什么中间要绕一层环形缓冲：擦写 Flash 是毫秒级操作，而 115200 波特率下
 *  每 87us 就来一个字节，单字节中断接收在中断里写 Flash 必然丢数据。
 * ========================================================================== */

#include "ota.h"
#include "../upload/esp8266.h"
#include "../upload/wifi_config.h"
#include "../interface/w25q16.h"
#include "../boot/boot_param.h"    /* boot_crc32_update */
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* main.c 里提供的硬件看门狗喂狗 */
extern void IWDG_Feed(void);

/* 攒够这么多字节再写一次 Flash：太小则页编程次数多、太慢；太大则占 RAM */
#define OTA_FLUSH_SIZE      4096U

/* ---- 内部 Flash 扇区映射 ----
 * F407 1MB 的扇区大小不规则：
 *   Sector 0~3  16KB   （0x08000000 / 0x08004000 / 0x08008000 / 0x0800C000）
 *   Sector 4    64KB   （0x08010000）
 *   Sector 5~11 128KB  （0x08020000 起，每 0x20000 一颗）
 * 擦除必须整颗擦，所以升级时要按固件实际大小算"从哪颗擦到哪颗"。 */
static void flash_sector_of(uint32_t addr,
                            uint32_t *sector, uint32_t *base, uint32_t *size)
{
    if (addr < 0x08010000U)                       /* Sector 0~3：16KB */
    {
        uint32_t idx = (addr - 0x08000000U) / 0x4000U;
        *sector = idx;
        *base   = 0x08000000U + idx * 0x4000U;
        *size   = 0x4000U;
    }
    else if (addr < 0x08020000U)                  /* Sector 4：64KB */
    {
        *sector = FLASH_SECTOR_4;
        *base   = 0x08010000U;
        *size   = 0x10000U;
    }
    else                                          /* Sector 5~11：128KB */
    {
        uint32_t idx = (addr - 0x08020000U) / 0x20000U;
        *sector = FLASH_SECTOR_5 + idx;
        *base   = 0x08020000U + idx * 0x20000U;
        *size   = 0x20000U;
    }
}

/* +IPD 解析状态 */
enum { IPD_IDLE = 0, IPD_LEN, IPD_DATA };

typedef struct
{
    /* ---- +IPD,<len>: 解析 ---- */
    uint8_t  ipd_state;
    uint8_t  match_idx;          /* "+IPD," 已匹配到第几个字符 */
    uint32_t ipd_len;            /* 正在解析的长度数字 */
    uint32_t ipd_remain;         /* 本帧还剩多少数据字节 */

    /* ---- HTTP 响应头解析 ---- */
    uint8_t  hdr_done;           /* 已越过 \r\n\r\n */
    uint8_t  status_done;
    uint8_t  status_len;
    char     status[40];
    uint32_t hdr_tail;           /* 最近 4 个字节，用来识别 \r\n\r\n */
    char     hdr[512];           /* 完整响应头，用来取 Content-Length */
    uint16_t hdr_len;
    uint32_t body_limit;         /* 由 Content-Length 得出，收到这么多就停 */
    uint32_t expect_size;        /* 调用方给的期望值（0 = 完全以响应头为准） */

    /* ---- 固件数据 ---- */
    uint8_t  buf[OTA_FLUSH_SIZE];
    uint32_t buf_len;
    uint32_t total;
    uint32_t crc;                /* 0xFFFFFFFF 起算的中间值 */
    uint32_t flash_addr;
} ota_ctx_t;

/* 4KB 缓冲 + 状态，必须放静态区：
 * Mqtt_Upload_Task 的栈只有 512 字（2KB），放局部变量直接溢出 */
static ota_ctx_t s_ota;

/* ------------------------------------------------------------------ */
/*  固件数据落盘                                                       */
/* ------------------------------------------------------------------ */
static void ota_body_byte(ota_ctx_t *c, uint8_t b)
{
    c->buf[c->buf_len++] = b;
    c->total++;

    if (c->buf_len >= OTA_FLUSH_SIZE)
    {
        w25q16_write(c->flash_addr, c->buf, c->buf_len);
        c->crc = boot_crc32_update(c->crc, c->buf, c->buf_len);
        c->flash_addr += c->buf_len;
        c->buf_len = 0;
    }
}

/* ------------------------------------------------------------------ */
/*  逐字节状态机：AT 原始流 -> +IPD 帧 -> HTTP 头 -> 固件本体          */
/* ------------------------------------------------------------------ */
static void ota_feed(ota_ctx_t *c, uint8_t b)
{
    switch (c->ipd_state)
    {
    case IPD_IDLE:
    {
        /* 逐字符匹配 "+IPD,"，中途失配就重来。
         * 这段流里还夹着 "SEND OK"、"\r\n" 之类，靠这个状态机滤掉 */
        static const char pat[] = "+IPD,";

        if (b == (uint8_t)pat[c->match_idx])
        {
            if (++c->match_idx >= 5U)
            {
                c->match_idx = 0;
                c->ipd_len   = 0;
                c->ipd_state = IPD_LEN;
            }
        }
        else
        {
            c->match_idx = (b == (uint8_t)'+') ? 1U : 0U;
        }
        break;
    }

    case IPD_LEN:
        if (b >= (uint8_t)'0' && b <= (uint8_t)'9')
        {
            c->ipd_len = c->ipd_len * 10U + (uint32_t)(b - (uint8_t)'0');
            if (c->ipd_len > OTA_MAX_FW_SIZE)      /* 明显是误匹配 */
            {
                c->ipd_len   = 0;
                c->ipd_state = IPD_IDLE;
            }
        }
        else if (b == (uint8_t)':')
        {
            c->ipd_remain = c->ipd_len;
            c->ipd_len    = 0;
            c->ipd_state  = (c->ipd_remain > 0U) ? IPD_DATA : IPD_IDLE;
        }
        else
        {
            c->ipd_len   = 0;
            c->ipd_state = IPD_IDLE;
        }
        break;

    case IPD_DATA:
        if (!c->hdr_done)
        {
            /* 记一下状态行，方便判断是不是 404 / 500 */
            if (!c->status_done)
            {
                if (b == (uint8_t)'\n')      c->status_done = 1;
                else if (b != (uint8_t)'\r' && c->status_len < (uint8_t)(sizeof(c->status) - 1U))
                    c->status[c->status_len++] = (char)b;
            }

            /* 完整头也存一份，收完再解析 Content-Length。
             * 存不下就截断（正常响应头 200 字节以内，512 足够） */
            if (c->hdr_len < (uint16_t)(sizeof(c->hdr) - 1U))
                c->hdr[c->hdr_len++] = (char)b;

            /* 滑动窗口识别 "\r\n\r\n"：空行之后就是 body */
            c->hdr_tail = (c->hdr_tail << 8) | b;
            if (c->hdr_tail == 0x0D0A0D0AU)
            {
                const char *p;

                c->hdr_done       = 1;
                c->hdr[c->hdr_len] = '\0';
                c->status[c->status_len] = '\0';

                /* 有 Content-Length 就以它为准：这样不用把固件大小写死在代码里 */
                p = strstr(c->hdr, "Content-Length:");
                if (p == NULL) p = strstr(c->hdr, "content-length:");
                if (p != NULL)
                {
                    unsigned long v = strtoul(p + 15, NULL, 10);
                    if (v > 0UL && v <= (unsigned long)OTA_MAX_FW_SIZE)
                        c->body_limit = (uint32_t)v;
                }
                if (c->body_limit == 0U)
                    c->body_limit = c->expect_size;   /* 退回到调用方给的值 */
            }
        }
        else
        {
            ota_body_byte(c, b);
        }

        if (--c->ipd_remain == 0U)
            c->ipd_state = IPD_IDLE;
        break;

    default:
        c->ipd_state = IPD_IDLE;
        break;
    }
}

/* ------------------------------------------------------------------ */
/*  下载主流程                                                         */
/* ------------------------------------------------------------------ */
int ota_http_download(const char *host, uint16_t port, const char *path,
                      uint32_t expect_size, ota_result_t *res)
{
    char     req[192];
    char     cmd[64];
    int      req_len;
    uint8_t  tmp[256];
    uint32_t n;
    uint32_t t0;
    ota_ctx_t *c = &s_ota;

    if (host == NULL || path == NULL || res == NULL ||
        expect_size > OTA_MAX_FW_SIZE)
        return -1;

    memset(c, 0, sizeof(*c));
    c->ipd_state   = IPD_IDLE;
    c->crc         = 0xFFFFFFFFU;
    c->flash_addr  = OTA_STAGE_ADDR;
    c->expect_size = expect_size;

    printf("[OTA] GET %s  <- %s:%u\r\n", path, host, (unsigned)port);

    /* 1. 先把整个暂存区擦干净再连网。
     *    这一步不需要事先知道固件多大（大小后面从 Content-Length 取），
     *    也避免 TCP 连上之后再干等 1 秒擦除导致服务端超时。
     *    448KB = 7 个 64KB 块，约 1 秒。 */
    if (w25q16_erase_range(OTA_STAGE_ADDR, OTA_MAX_FW_SIZE) != 0)
    {
        printf("[OTA] erase staging area fail\r\n");
        return -4;
    }

    /* 2. 建 TCP 连接（内部会设 CIPMUX=0，单连接） */
    if (!ESP8266_OpenTCP(host, port))
    {
        printf("[OTA] TCP connect fail\r\n");
        return -2;
    }

    /* 3. 告知要发的字节数，等模块回 '>' */
    req_len = snprintf(req, sizeof(req),
                       "GET %s HTTP/1.0\r\nHost: %s\r\n\r\n", path, host);
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", req_len);

    if (!ESP8266_SendCmd(cmd, ">", 3000))
    {
        printf("[OTA] CIPSEND no '>'\r\n");
        ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", 2000);
        return -3;
    }

    /* 4. 进原始模式后发请求：这之后 USART3 收到的字节都进环形缓冲 */
    ESP8266_SetRawMode(1);
    ESP8266_Send((const uint8_t *)req, (uint16_t)req_len);

    /* 5. 收数据：边解析边写 Flash。
     *    结束条件 = 收满 Content-Length；收不到就等超时 */
    t0 = HAL_GetTick();
    while (c->body_limit == 0U || c->total < c->body_limit)
    {
        n = ESP8266_ReadRaw(tmp, sizeof(tmp), 500);

        if (n > 0U)
        {
            uint32_t i;
            for (i = 0; i < n; i++)
            {
                ota_feed(c, tmp[i]);
                if (c->body_limit != 0U && c->total >= c->body_limit) break;
                if (c->total >= OTA_MAX_FW_SIZE) break;   /* 防止无边界地写 */
            }
            t0 = HAL_GetTick();
        }
        else if ((HAL_GetTick() - t0) > OTA_RECV_TIMEOUT_MS)
        {
            printf("[OTA] recv timeout at %lu bytes\r\n", (unsigned long)c->total);
            break;
        }
    }

    ESP8266_SetRawMode(0);

    /* 6. 收尾：把缓冲里剩下的写掉 */
    if (c->buf_len > 0U)
    {
        w25q16_write(c->flash_addr, c->buf, c->buf_len);
        c->crc = boot_crc32_update(c->crc, c->buf, c->buf_len);
        c->flash_addr += c->buf_len;
        c->buf_len = 0;
    }

    ESP8266_SendCmd("AT+CIPCLOSE\r\n", "OK", 3000);

    res->size = c->total;
    res->crc  = ~c->crc;
    if (c->status_len < sizeof(res->http_status))
    {
        memcpy(res->http_status, c->status, c->status_len);
        res->http_status[c->status_len] = '\0';
    }
    else
    {
        res->http_status[0] = '\0';
    }

    printf("[OTA] recv %lu / %lu bytes (Content-Length), \"%s\", crc32=0x%08lX\r\n",
           (unsigned long)res->size, (unsigned long)c->body_limit,
           res->http_status, (unsigned long)res->crc);

    if (c->body_limit == 0U)
    {
        printf("[OTA] no Content-Length and no expected size\r\n");
        return -6;
    }
    return (res->size == c->body_limit) ? 0 : -5;
}

/* ------------------------------------------------------------------ */
/*  暂存区 -> 内部 App B                                               */
/* ------------------------------------------------------------------ */
int ota_apply(uint32_t size, uint32_t crc)
{
    uint32_t off, i, n, k, w;
    uint32_t calc;
    uint32_t nsect;
    uint32_t sect_err = 0;
    uint32_t target_slot, target_base, target_max;
    uint32_t first_sector, s_base, s_size, addr;
    FLASH_EraseInitTypeDef erase;
    boot_param_t p;
    uint8_t *buf = s_ota.buf;          /* 复用 4KB 下载缓冲，不再另占 RAM */

    /* ---- 0. 目标永远是"非活动分区" ----
     * 绝对不能往正在跑的这块写：擦除会把当前执行的代码一起抹掉，
     * 只能靠 ART I-Cache 里的残留指令侥幸跑完，参数一变就崩。
     * 跑在 A 就刷 B，跑在 B 就刷 A —— 这才是 A/B 双分区的意义。
     *
     * ★ 镜像必须按目标槽的链接地址分开编译 ★
     * App 是"绝对地址"镜像：向量表里的 MSP / Reset_Handler、字面量池、
     * 长跳转表存的都是链接时算好的地址。把链接在 0x0800C000 的 bin
     * 原样烧到 0x08080000，里面所有地址仍指向 A 区 —— 跳过去实际执行
     * 的还是 A 区代码（实机表现：A/B 内容相同时"看起来能跑"，一旦 A 被擦
     * 就立刻静默 HardFault）。
     * 所以 Keil 工程里做了两个 target：
     *     f407    -> IROM 0x0800C000，产出 f407_A.bin
     *     f407_B  -> IROM 0x08080000，产出 f407_B.bin
     * 云端下发时必须配对：跑在 A 用 URL 指 f407_B.bin，跑在 B 指 f407_A.bin。 */
    if (!boot_param_read(&p)) boot_param_default(&p);

    target_slot = (p.active_slot == SLOT_A) ? SLOT_B : SLOT_A;
    target_base = BOOT_SLOT_ADDR(target_slot);
    target_max  = BOOT_SLOT_SIZE(target_slot);

    printf("[OTA] target = slot %lu @ 0x%08lX (running slot %lu)\r\n",
           (unsigned long)target_slot, (unsigned long)target_base,
           (unsigned long)p.active_slot);

    if (size == 0U || size > target_max || crc == 0U) return -1;

    /* ---- 1. 回读暂存区重算 CRC ----
     * 下载时算的那次只证明"线上收到的字节对"，
     * 这次才证明"真正落进 W25Q16 的字节对"。 */
    printf("[OTA] verify staging ...\r\n");
    calc = 0xFFFFFFFFU;
    for (off = 0; off < size; )
    {
        n = size - off;
        if (n > OTA_FLUSH_SIZE) n = OTA_FLUSH_SIZE;

        w25q16_read(OTA_STAGE_ADDR + off, buf, n);
        calc = boot_crc32_update(calc, buf, n);
        off += n;
        IWDG_Feed();
    }
    calc = ~calc;

    if (calc != crc)
    {
        printf("[OTA] staging CRC mismatch: got 0x%08lX expect 0x%08lX\r\n",
               (unsigned long)calc, (unsigned long)crc);
        return -2;
    }
    printf("[OTA] staging CRC OK\r\n");

    /* ---- 2. 擦除目标分区里固件占用的扇区 ----
     * 整个 512KB 全擦要 4 颗扇区、约 4~8s，接近 IWDG 的 8.2s，而固件
     * 往往只有几十 KB。按实际大小算扇区数：既快，也少消耗擦写寿命。 */
    flash_sector_of(target_base, &first_sector, &s_base, &s_size);
    (void)s_base;

    nsect = 0U;
    addr  = target_base;
    while (addr < (target_base + size))
    {
        flash_sector_of(addr, &first_sector, &s_base, &s_size);
        (void)first_sector;
        addr = s_base + s_size;        /* 跳到下一颗扇区起点 */
        nsect++;
    }
    if (nsect == 0U) nsect = 1U;

    flash_sector_of(target_base, &first_sector, &s_base, &s_size);

    printf("[OTA] erase %lu sector(s) from sector %lu ...\r\n",
           (unsigned long)nsect, (unsigned long)first_sector);

    HAL_FLASH_Unlock();

    /* 一颗一颗擦，每颗之间喂狗并打点。
     * 一次调 HAL_FLASHEx_Erase 传 NbSectors>1 是能擦的，
     * 但中途出问题时看不出卡在哪颗；这里拆开便于定位。 */
    for (n = 0U; n < nsect; n++)
    {
        erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;    /* 2.7~3.6V */
        erase.Sector       = first_sector + n;
        erase.NbSectors    = 1U;

        printf("[OTA]   erase sector %lu ...\r\n",
               (unsigned long)(first_sector + n));

        if (HAL_FLASHEx_Erase(&erase, &sect_err) != HAL_OK)
        {
            HAL_FLASH_Lock();
            printf("[OTA] erase fail @sector %lu, err=0x%08lX\r\n",
                   (unsigned long)(first_sector + n), (unsigned long)sect_err);
            return -3;
        }
        IWDG_Feed();
    }

    /* ---- 3. 逐字编程 ----
     * 先整块从 SPI Flash 读进 4KB 缓冲，再按 32 位写内部 Flash。
     * 比逐字节 HAL_FLASH_Program 快一个数量级。 */
    printf("[OTA] program %lu bytes ...\r\n", (unsigned long)size);

    for (off = 0; off < size; )
    {
        n = size - off;
        if (n > OTA_FLUSH_SIZE) n = OTA_FLUSH_SIZE;

        w25q16_read(OTA_STAGE_ADDR + off, buf, n);

        for (i = 0; i < n; i += 4U)
        {
            /* 末尾不足 4 字节时高位补 0：多写的这几个字节
             * 仍在刚擦干净的扇区里，不影响 CRC（CRC 只算 size 之内） */
            w = 0U;
            for (k = 0U; k < 4U; k++)
                if (i + k < n) w |= ((uint32_t)buf[i + k]) << (k * 8U);

            /* ★ 必须用 target_base，不能硬编码 APP_B_BASE_ADDR ★
             * 从 B 区升级 A 时目标是 0x0800C000；这里写死 B 区地址的话，
             * 擦的是 A、写的却是 B —— B 区没擦除，数据会被"按位与"叠加上去
             * （Flash 编程只能 1→0），写几十字节后报错复位，B 区向量表被毁，
             * 表现就是复位后跳过去一个字都不打。 */
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                  target_base + off + i, w) != HAL_OK)
            {
                HAL_FLASH_Lock();
                printf("[OTA] program fail @0x%08lX\r\n",
                       (unsigned long)(target_base + off + i));
                return -4;
            }
        }
        off += n;
        IWDG_Feed();
    }

    HAL_FLASH_Lock();

    /* ---- 4. 回验内部 Flash ----
     * Bootloader 启动时还会再验一次，这里提前验是为了
     * 编程没写对时干脆别动参数区，直接当这次升级没发生过。 */
    if (boot_crc32((const uint8_t *)target_base, size) != crc)
    {
        printf("[OTA] internal flash verify FAIL\r\n");
        return -5;
    }
    printf("[OTA] internal flash verify OK\r\n");

    /* ---- 5. 写参数区，标记"新分区待验证" ----
     * boot_count 置 1：本次启动算第一次尝试。
     * App 跑到自检通过会调 boot_param_mark_ok() 清零；
     * 没跑到就复位（HardFault / 看门狗）的话，Bootloader 每次
     * 上电会继续累加，超过 boot_max 就自动回滚到 A。 */
    if (!boot_param_read(&p)) boot_param_default(&p);

    p.upgrade_flag = BOOT_FLAG_PENDING;
    p.target_slot  = target_slot;
    p.target_size  = size;
    p.target_crc   = crc;
    p.boot_count   = 1;

    if (!boot_param_write(&p))
    {
        printf("[OTA] write param fail\r\n");
        return -6;
    }

    printf("[OTA] param updated: PENDING -> slot %lu\r\n",
           (unsigned long)target_slot);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  链路准备：把 ESP8266 拉回 AT 模式并连上 WiFi                       */
/* ------------------------------------------------------------------ */
/* 这里必须先复位模块：
 *   1) 顺着 +++ 退出 MQTT 用的透传模式；
 *   2) ESP8266 是 CIPMUX=0 单连接，旧的 MQTT TCP 不关掉就建不了新连接。
 * 所以调用这个函数等于"主动把 MQTT 连接掐了"，调用方要做好重连准备。 */
static int ota_link_up(void)
{
    if (!ESP8266_Reset())
    {
        printf("[OTA] ESP8266 reset retry...\r\n");
        vTaskDelay(pdMS_TO_TICKS(1500));
        if (!ESP8266_Reset())
        {
            printf("[OTA] ESP8266 reset fail\r\n");
            return -1;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(2000));      /* 等模块启动稳定 */

    if (!ESP8266_SetMode(1))
    {
        printf("[OTA] SetMode fail\r\n");
        return -2;
    }
    if (!ESP8266_JoinAP(WIFI_SSID, WIFI_PASS))
    {
        printf("[OTA] JoinAP fail\r\n");
        return -3;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/*  按 URL 执行一次完整升级                                            */
/* ------------------------------------------------------------------ */
/* 解析结果放静态区：MqttProc 的栈只有 2KB，
 * 而 ota_http_download 内部已经要吃掉 500 多字节局部变量。 */
static char s_url_host[64];
static char s_url_path[96];

int ota_run_url(const char *url)
{
    ota_result_t res;
    const char *p, *q;
    char *end;
    char *dash;
    uint16_t port = 80;
    int n, rc;

    if (url == NULL) return -1;

    /* 只支持 http://。ESP8266-01S 可用堆只有 40KB 左右，
     * TLS 握手要把这吃光，实测必然 CIPSTART 失败或模块重启。 */
    if (strncmp(url, "http://", 7) != 0)
    {
        printf("[OTA] only http:// is supported\r\n");
        return -1;
    }

    /* ---- 拆 http://host:port/path ---- */
    p = url + 7;

    q = p;
    while (*q != '\0' && *q != ':' && *q != '/') q++;
    n = (int)(q - p);
    if (n <= 0 || n >= (int)sizeof(s_url_host)) return -1;
    memcpy(s_url_host, p, (size_t)n);
    s_url_host[n] = '\0';

    if (*q == ':')
    {
        unsigned long v = strtoul(q + 1, &end, 10);
        if (v > 0UL && v <= 65535UL) port = (uint16_t)v;
        q = end;
    }

    dash = s_url_path;
    snprintf(s_url_path, sizeof(s_url_path), "%s", (*q == '/') ? q : "/");

    printf("[OTA] url -> host=%s port=%u path=%s\r\n",
           s_url_host, (unsigned)port, dash);

    /* ---- 建链路 -> 下载 -> 刷 B 区 ---- */
    if (ota_link_up() != 0) return -2;

    rc = ota_http_download(s_url_host, port, s_url_path, 0U, &res);
    if (rc != 0)
    {
        printf("[OTA] download FAIL (rc=%d)\r\n", rc);
        return -10;
    }
    printf("[OTA] download OK: %lu bytes, crc32=0x%08lX\r\n",
           (unsigned long)res.size, (unsigned long)res.crc);

    rc = ota_apply(res.size, res.crc);
    if (rc != 0)
    {
        printf("[OTA] apply FAIL (rc=%d), stay on current slot\r\n", rc);
        return -20;
    }

    printf("[OTA] apply OK, reboot ...\r\n");
    vTaskDelay(pdMS_TO_TICKS(500));       /* 等串口把话说完 */
    NVIC_SystemReset();
    return 0;                             /* 走不到这里 */
}

/* ------------------------------------------------------------------ */
/*  调试用：上电自动下载一次                                           */
/* ------------------------------------------------------------------ */
void ota_auto_test(void)
{
    ota_result_t res;
    boot_param_t p;
    int rc;

    /* 只在 App A 上跑自检。
     * 否则升级成功、重启进 App B 之后又会再下载一遍、再刷一次 B，
     * 变成无限重启。跑到 B 说明这次升级已经验证过了。 */
    if (boot_param_read(&p) && p.active_slot != SLOT_A)
    {
        printf("[OTA] already running slot %lu, skip auto test\r\n",
               (unsigned long)p.active_slot);
        return;
    }

    printf("\r\n[OTA] === HTTP download selftest ===\r\n");

    /* ESP8266 冷启动时 ROM 会用 74880 波特率打印一段 boot 信息，
     * 115200 采样出来就是满屏乱码（实测把 1024 字节缓冲直接灌满），
     * 导致紧接着的 AT 命令匹配不上。先等它吐完。 */
    vTaskDelay(pdMS_TO_TICKS(2500));

    if (ota_link_up() != 0) return;

    /* 连通性诊断：板子现在连的是 PC 自己开的移动热点，
     * 热点网卡 192.168.137.1 就是板子的网关，ping 通说明二层直连没问题。
     * （之前走校园网时这一项和 CIPSTART 都是静默超时，
     *   因为校园网网关做代理 ARP 并禁止终端互访。） */
    ESP8266_SendCmd("AT+PING=\"" OTA_TEST_HOST "\"\r\n", "+PING:", 5000);
    vTaskDelay(pdMS_TO_TICKS(300));
    printf("[OTA] ping host : %s\r\n", ESP8266_LastResponse());

    ESP8266_SendCmd("AT+CIFSR\r\n", "OK", 3000);
    vTaskDelay(pdMS_TO_TICKS(300));
    printf("[OTA] CIFSR     : %s\r\n", ESP8266_LastResponse());

    rc = ota_http_download(OTA_TEST_HOST, OTA_TEST_PORT, OTA_TEST_PATH,
                           OTA_TEST_SIZE, &res);

    if (rc != 0)
    {
        printf("[OTA] selftest FAIL (rc=%d)\r\n", rc);
        return;
    }
    printf("[OTA] download PASS: %lu bytes, crc32=0x%08lX\r\n",
           (unsigned long)res.size, (unsigned long)res.crc);

    /* 下载成功就顺手做一次完整升级，验证"暂存 -> 内部 B -> 复位 -> 跳转"整条链 */
    rc = ota_apply(res.size, res.crc);
    if (rc != 0)
    {
        printf("[OTA] apply FAIL (rc=%d), stay on slot A\r\n", rc);
        return;
    }

    printf("[OTA] apply OK, reboot into App B ...\r\n");
    vTaskDelay(pdMS_TO_TICKS(500));       /* 等串口把话说完 */
    NVIC_SystemReset();
}
