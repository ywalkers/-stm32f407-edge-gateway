#include "esp8266.h"
#include "usart.h"
#include "mqtt.h"
#include "lwrb/lwrb.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"

extern UART_HandleTypeDef huart3;

/* 由 app_rtos.c 提供：AT 等待期间替 MQTT 任务打看门狗心跳，
 * 否则最长 15s 的 AT 等待会超过 6s 心跳阈值，导致 IWDG 复位 */
extern void App_MqttWdtAlive(void);

/* ---- USART3 接收 ---- */
#define ESP_RX_SIZE 1024
static uint8_t  esp_rx_byte;                              /* 中断接收的单个字节 */
static uint8_t  esp_rx[ESP_RX_SIZE];                      /* AT 阶段响应缓冲 */
static volatile uint16_t esp_rx_head = 0;                 /* 已接收字节数 */
static volatile uint8_t  esp_transparent = 0;             /* 1=已进入透传 */

/* ---- OTA 原始数据接收 ----
 * 用 lwrb 替代手写环形缓冲：
 *   - 写端：USART3 中断回调（单写者）
 *   - 读端：OTA 任务里的 ESP8266_ReadRaw（单读者）
 * 正好满足 lwrb 的单写单读无锁模型，不需要关中断 */
#define ESP_RAW_BUF_SIZE   4096U
static uint8_t  esp_raw_buf[ESP_RAW_BUF_SIZE];
static lwrb_t   esp_raw_rb;
static volatile uint8_t esp_raw_mode = 0;

/* 发送一行 AT 命令（已含 \r\n，调用方传入） */
static void esp_at_send(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 1000);
}

/* 发送 AT 命令，等待接收内容中出现 resp；cmd 为 NULL 则只等待 */
static uint8_t esp_expect(const char *cmd, const char *resp, uint32_t timeout_ms)
{
    esp_rx_head = 0;                          /* 清空响应缓冲 */
    if (cmd) esp_at_send(cmd);

    uint32_t start = HAL_GetTick();
    uint16_t rlen = (uint16_t)strlen(resp);

    while ((HAL_GetTick() - start) < timeout_ms) {
        uint16_t n = esp_rx_head;
        if (n >= rlen) {
            for (uint16_t i = 0; i + rlen <= n; i++) {
                if (memcmp(&esp_rx[i], resp, rlen) == 0)
                    return 1;
            }
        }

        /* 打看门狗心跳 + 主动让出 CPU：
         * 1. AT 等待最长 15s，不打心跳会触发 IWDG 复位；
         * 2. 纯忙等会饿死优先级更低的 Watchdog_Task，必须让出时间片。 */
        App_MqttWdtAlive();
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    /* 超时失败：打印收到的字节数和原始内容
     *   0 字节   -> 模块完全没说话（供电/接线/模块停在透传）
     *   有乱码   -> 收到了数据但解析不出，多半是波特率不匹配
     * 注意：数据里可能含 0x00，%s 会截断看起来像空串，
     * 所以额外打前 16 字节的十六进制，便于判断到底是"没数据"还是"乱码"。 */
    if (esp_rx_head >= ESP_RX_SIZE) esp_rx[ESP_RX_SIZE - 1] = '\0';
    else                             esp_rx[esp_rx_head]        = '\0';

    printf("[ESP] expect \"%s\" fail (%u B), got: %s\r\n",
           resp, (unsigned)esp_rx_head, (char *)esp_rx);

    {
        uint16_t n = (esp_rx_head > 16) ? 16 : esp_rx_head;
        printf("[ESP]   hex:");
        for (uint16_t i = 0; i < n; i++)
            printf(" %02X", esp_rx[i]);
        printf("\r\n");
    }
    return 0;
}

void ESP8266_UART_Start(void)
{
    /* 初始化 OTA 原始数据环形缓冲（必须在启动接收中断之前） */
    lwrb_init(&esp_raw_rb, esp_raw_buf, ESP_RAW_BUF_SIZE);
    HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
}

uint8_t ESP8266_Reset(void)
{
    /* 只有上次确实停在透传模式时才发 +++ 退出。
     * 若已在 AT 模式还发 +++，模块会把它和后面的 AT+RST 拼成
     * "+++AT+RST" 这条非法命令，回 ERROR —— 这是之前连不上的根因。 */
    if (esp_transparent)
    {
        HAL_Delay(1000);          /* +++ 前保护时间：至少 1s 静默 */
        esp_at_send("+++");
        HAL_Delay(1000);          /* +++ 后保护时间 */
        esp_transparent = 0;
    }

    esp_rx_head = 0;

    /* 先发裸 AT 探测：能回 OK 说明模块在线且波特率正确 */
    if (!esp_expect("AT\r\n", "OK", 3000))
    {
        /* 打印 USART3 状态，帮助区分三种情况：
         *   err=0 且 rxstate=BUSY_RX  -> 接收中断正常，是模块没说话
         *   err 含 ORE/FE/NE          -> 线路上有数据但采样错，多半是波特率不匹配
         *   rxstate=NULL/READY        -> 接收中断没挂上 */
        printf("[ESP] AT no reply: uart3 err=0x%08lX rxstate=%d\r\n",
               (unsigned long)huart3.ErrorCode, (int)huart3.RxState);

        /* 第二种可能：MCU 复位了但模块没断电，模块还停在透传模式，
         * 此时 AT 会被当成业务数据透过 TCP 发走，不会返回 OK。
         * esp_transparent 是 RAM 变量，复位后已清零，所以上面那段
         * 主动退出逻辑不会执行，必须在这里兜底：
         * 强制用 +++ 退出透传后再探测一次（+++ 前后各需 1s 以上静默）。 */
        printf("[ESP] try force exit transparent mode...\r\n");
        HAL_Delay(1200);
        esp_at_send("+++");
        HAL_Delay(1200);
        esp_rx_head = 0;

        if (!esp_expect("AT\r\n", "OK", 3000))
        {
            printf("[ESP] still no response, check power/baud\r\n");
            return 0;
        }
        printf("[ESP] recovered from transparent mode\r\n");
    }

    /* 再复位；模块重启要打印启动信息，最多等 10s（原 5s 偏紧） */
    if (!esp_expect("AT+RST\r\n", "ready", 10000))
        return 0;

    /* 关回显。AT+RST 后模块的 echo 会恢复成默认"开"，
     * 之后每条命令都会被原样吐回来，把日志和匹配缓冲搅成一团，
     * 出问题时很难看清模块到底回了什么。 */
    esp_expect("ATE0\r\n", "OK", 2000);
    return 1;
}

uint8_t ESP8266_SetMode(uint8_t mode)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "AT+CWMODE=%d\r\n", mode);
    return esp_expect(cmd, "OK", 3000);
}

uint8_t ESP8266_JoinAP(const char *ssid, const char *pwd)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
    return esp_expect(cmd, "OK", 15000);       /* 连 WiFi 较慢，超时给足 */
}

uint8_t ESP8266_OpenTCP(const char *ip, uint16_t port)
{
    char cmd[96];

    /* 单连接模式 */
    if (!esp_expect("AT+CIPMUX=0\r\n", "OK", 3000)) return 0;

#if ESP8266_USE_TLS
    /* TLS 需要预分配握手缓冲。ESP8266-01S 的 heap 只有 ~40KB，
     * 4096 字节的 SSL buffer 经常申请失败：模块要么回 ERROR，
     * 要么直接重启（表现就是只回显、不回 OK，3s 后超时）。
     * 所以失败时退一档再试。 */
    if (!esp_expect("AT+CIPSSLSIZE=4096\r\n", "OK", 3000))
    {
        printf("[ESP] CIPSSLSIZE=4096 fail (heap?), retry 1024\r\n");
        if (!esp_expect("AT+CIPSSLSIZE=1024\r\n", "OK", 3000))
        {
            printf("[ESP] TLS buffer alloc fail, 01S heap too small\r\n");
            return 0;
        }
    }

    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"SSL\",\"%s\",%d\r\n", ip, port);
    /* TLS 握手比 TCP 慢得多，超时给 20s */
    return esp_expect(cmd, "CONNECT", 20000);
#else
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", ip, port);
    return esp_expect(cmd, "CONNECT", 15000);
#endif
}

/* ============================================================================
 *  SNTP 对时（Phase 3）
 *
 *  ESP8266 的 AT 固件自带 SNTP 客户端：CIPSNTPCFG 配一次服务器，之后
 *  CIPSNTPTIME? 就能拿到模块已经同步好的网络时间。
 *  注意：必须在「非透传」模式下调用 —— 透传后 AT 命令会被当业务数据发走。
 * ========================================================================== */

/* 使能 SNTP：时区参数用 0（UTC）。
 * 为什么不用 8：Unix 时间戳本身与时区无关，统一按 UTC 换算最干净，
 * 东八区的本地时间交给云端/前端去显示，避免"存的是本地时间还要反算" */
uint8_t ESP8266_SNTP_Start(void)
{
    return esp_expect("AT+CIPSNTPCFG=1,0,\"ntp.aliyun.com\"\r\n", "OK", 3000);
}

/* 公历日期 → 距 1970-01-01 的天数（Howard Hinnant 算法）
 * 比查表 + 闰年判断短得多，也不依赖库函数 */
static int32_t esp_days_from_civil(int32_t y, int32_t m, int32_t d)
{
    int32_t  era;
    uint32_t yoe, doy, doe;

    y -= (m <= 2) ? 1 : 0;
    era  = (y >= 0 ? y : y - 399) / 400;
    yoe  = (uint32_t)(y - era * 400);                                  /* [0, 399] */
    doy  = (uint32_t)((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1); /* [0, 365] */
    doe  = yoe * 365U + yoe / 4U - yoe / 100U + doy;                   /* [0, 146096] */
    return era * 146097 + (int32_t)doe - 719468;
}

/* 在接收缓冲里找 "CIPSNTPTIME:" 之后 24 字节日期串的起始位置（0 = 还没到齐） */
static uint16_t esp_sntp_find_date(void)
{
    uint16_t n = esp_rx_head;
    uint16_t i;

    if (n > ESP_RX_SIZE) n = ESP_RX_SIZE;

    for (i = 0U; (uint32_t)i + 12U + 24U <= n; i++)
    {
        if (memcmp(&esp_rx[i], "CIPSNTPTIME:", 12U) == 0)
            return (uint16_t)(i + 12U);
    }
    return 0U;
}

/* 查询时间，成功返回 1 并写出 UTC 的 Unix 秒。
 * 模块响应：+CIPSNTPTIME:Thu Nov 11 20:51:13 2026
 * 还没同步完时返回的是 1970 年的时间，这里按年份过滤掉，让调用方重试 */
uint8_t ESP8266_QueryTime(uint32_t *unix_sec)
{
    static const char *MON = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char     date[25];
    uint16_t i0 = 0U;
    uint32_t t0, mon, day, hh, mm, ss, year;
    int32_t  days;

    if (unix_sec == NULL) return 0U;

    if (!esp_expect("AT+CIPSNTPTIME?\r\n", "CIPSNTPTIME:", 3000)) return 0U;

    /* esp_expect 是"看到前缀就返回"，日期串可能还在路上：最多再等 500ms 补齐 */
    t0 = HAL_GetTick();
    while (((HAL_GetTick() - t0) < 500U) && ((i0 = esp_sntp_find_date()) == 0U))
    {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (i0 == 0U) return 0U;

    memcpy(date, &esp_rx[i0], 24U);
    date[24] = '\0';

    /* "Thu Nov 11 20:51:13 2026"
     *   月份 [4..6]  日 [8..9]  时 [11..12]  分 [14..15]  秒 [17..18]  年 [20..23] */
    for (mon = 0U; mon < 12U; mon++)
    {
        if (memcmp(&date[4], &MON[mon * 3U], 3U) == 0) break;
    }
    if (mon >= 12U) return 0U;

    day  = (uint32_t)((date[8]  - '0') * 10 + (date[9]  - '0'));
    hh   = (uint32_t)((date[11] - '0') * 10 + (date[12] - '0'));
    mm   = (uint32_t)((date[14] - '0') * 10 + (date[15] - '0'));
    ss   = (uint32_t)((date[17] - '0') * 10 + (date[18] - '0'));
    year = (uint32_t)((date[20] - '0') * 1000 + (date[21] - '0') * 100
                    + (date[22] - '0') * 10   + (date[23] - '0'));

    /* 合理性校验：没同步时模块给的是 1970-01-01，这里直接判失败 */
    if (year < 2020U || day == 0U || day > 31U || hh > 23U || mm > 59U || ss > 60U)
        return 0U;

    days = esp_days_from_civil((int32_t)year, (int32_t)(mon + 1U), (int32_t)day);
    if (days < 0) return 0U;

    *unix_sec = (uint32_t)days * 86400UL + hh * 3600UL + mm * 60UL + ss;
    return 1U;
}

uint8_t ESP8266_EnterTransparent(void)
{
    if (!esp_expect("AT+CIPMODE=1\r\n", "OK", 3000)) return 0;
    if (!esp_expect("AT+CIPSEND\r\n", ">", 3000))     return 0;
    esp_transparent = 1;
    return 1;
}

/* 透传发送：作为 MQTT 层的 send 回调 */
void ESP8266_Send(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 1000);
}

/* ============================================================================
 *  原始数据模式（OTA 下载专用）
 *
 *  OTA 时链路是「非透传」的：模块用 +IPD,<len>:<data> 的格式把 TCP 数据
 *  吐出来。这里把中断收到的字节原样存进环形缓冲，解析工作交给 ota.c。
 *  中断里绝不能写 Flash —— 擦一个 4KB 扇区要 45ms，115200 波特率下
 *  期间会来 500 多个字节，单字节中断接收必丢数据。
 * ========================================================================== */

/* 缓冲区与句柄声明在文件顶部（ESP8266_UART_Start 要用） */

void ESP8266_SetRawMode(uint8_t on)
{
    lwrb_reset(&esp_raw_rb);
    esp_raw_mode = on;
}

uint32_t ESP8266_ReadRaw(uint8_t *buf, uint32_t len, uint32_t timeout_ms)
{
    uint32_t got = 0;
    uint32_t t0  = HAL_GetTick();

    while (got < len)
    {
        /* 一次能拷多少就拷多少（lwrb 内部会处理回绕，最多分两段 memcpy） */
        lwrb_sz_t n = lwrb_read(&esp_raw_rb, &buf[got], (lwrb_sz_t)(len - got));

        if (n > 0)
        {
            got += n;
            t0 = HAL_GetTick();          /* 有数据就续期，只有真正空等才计时 */
        }
        else if ((HAL_GetTick() - t0) >= timeout_ms)
        {
            break;
        }
        else
        {
            App_MqttWdtAlive();          /* 长下载期间别让 IWDG 复位 */
            vTaskDelay(1);               /* 让出 CPU，低优先级任务才能跑 */
        }
    }
    return got;
}

uint8_t ESP8266_SendCmd(const char *cmd, const char *resp, uint32_t timeout_ms)
{
    return esp_expect(cmd, resp, timeout_ms);
}

const char *ESP8266_LastResponse(void)
{
    uint16_t n = esp_rx_head;

    if (n >= ESP_RX_SIZE) n = ESP_RX_SIZE - 1U;
    esp_rx[n] = '\0';
    return (const char *)esp_rx;
}

/* ---- 中断回调：USART3 单字节接收 ---- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    if (esp_raw_mode) {
        /* OTA 阶段：整包塞环形缓冲，解析和写 Flash 都在任务里做。
         * lwrb 满了只会写入能容纳的部分（丢弃新字节），与旧逻辑一致。 */
        lwrb_write(&esp_raw_rb, &esp_rx_byte, 1);
    } else if (esp_transparent) {
        /* 透传阶段：broker 回的数据喂给 MQTT 解析（CONNACK/PINGRESP） */
        MQTT_FeedByte(&g_mqtt, esp_rx_byte);
    } else {
        /* AT 阶段：存进缓冲供 esp_expect 匹配 */
        if (esp_rx_head < ESP_RX_SIZE)
            esp_rx[esp_rx_head++] = esp_rx_byte;
    }
    HAL_UART_Receive_IT(huart, &esp_rx_byte, 1);   /* 继续收下一字节 */
}

/* app_rtos.c 提供：USART2(RS485) 出错后重新挂接收 DMA。
 * HAL 遇到 FE/ORE/NE 会把接收 DMA 关掉且不自恢复，不转发这一下，RS485 一次
 * 帧错误之后就永久收不到东西（2026-11-11 实测踩坑）。 */
extern void RS485_Uart_Error_Rearm(uint32_t err_code);

/* 接收出错也要恢复接收，避免串口停摆 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        HAL_UART_Receive_IT(huart, &esp_rx_byte, 1);
    }
    else if (huart->Instance == USART2) {
        RS485_Uart_Error_Rearm(huart->ErrorCode);
    }
}
