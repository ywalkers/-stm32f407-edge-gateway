#include "app_rtos.h"
#include "main.h"
#include "gpio.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "message_buffer.h"
#include "Int_can.h"
#include "modbus.h"
#include "edge.h"
#include "../upload/mqtt.h"
#include "../upload/esp8266.h"
#include "../ota/ota.h"
#include "offline_cache.h"   /* W25Q16 日志式离线缓存 */
#include "time_sync.h"       /* SNTP 对时后的绝对时间（Phase 3） */
extern CAN_HandleTypeDef hcan1;

/* ---- 全局对象 ---- */
QueueHandle_t        CAN_Msg_Queue      = NULL;
MessageBufferHandle_t RS485_MsgBuffer   = NULL;
SemaphoreHandle_t    RS485_Tx_Semaphore = NULL;
SemaphoreHandle_t    Print_Mutex        = NULL;
QueueHandle_t        Upload_Queue       = NULL;   /* 上行 JSON 队列：DataProc -> Mqtt 任务 */

/* ---- MQTT 主题 ---- */
#define TOPIC_SENSOR   "gw/node1/sensor"
#define TOPIC_ONLINE   "gw/node1/online"
#define TOPIC_CMD      "gw/node1/cmd"      /* 云端下发控制命令 */
#define TOPIC_ALARM    "gw/node1/alarm"    /* 边缘告警（高温/低温/恢复） */
#define TOPIC_STATS    "gw/node1/stats"    /* 分钟级统计 */
#define TOPIC_OTA      "gw/node1/ota/cmd"  /* 云端下发升级指令 {"ota":"http://..."} */
#define TOPIC_OTA_ST   "gw/node1/ota/status" /* 升级进度回执 */

/* ---- 上行队列元素：带上主题，让同一队列能承载不同类型消息 ---- */
typedef struct {
    uint8_t topic;          /* 0=sensor 1=alarm 2=stats */
    char    data[144];      /* JSON 字符串（stats 报文最长约 115 字节） */
} UPLOAD_ITEM;

static const char *Topic_Of(uint8_t id)
{
    switch (id) {
    case 1:  return TOPIC_ALARM;
    case 2:  return TOPIC_STATS;
    default: return TOPIC_SENSOR;
    }
}

/* ---- 云端下行命令：中断里解析置标志，任务里执行 ----
 * 不在中断里直接调 Gateway_Send_Cmd/printf：printf 用互斥量，中断中会死锁 */
static volatile uint8_t  cloud_cmd_pending = 0;
static volatile uint8_t  cloud_cmd_code    = 0;   /* 0x02 改周期 / 0x03 控 LED */
static volatile uint16_t cloud_cmd_param   = 0;

/* RS485 接收缓存：Ping-Pong 双缓冲，DMA 填一块、CPU 处理另一块，无缝切换不丢包。
 * 收满的一块以「一条完整消息」写入 MessageBuffer，任务侧不再解析长度头。 */
static uint8_t RS485_Rx_Buf[2][RS485_MAX_LEN];
static volatile uint8_t rs485_rx_sel = 0;   /* 当前 DMA 正在填充的缓冲编号 */
static volatile uint32_t rs485_overflow_cnt = 0;  /* MessageBuffer 满导致整帧丢弃的次数 */
/* USART2 的 FE/ORE/NE 错误计数：中断里只记账，打印留给任务，避免在 ISR 里拿打印锁 */
static volatile uint32_t rs485_uart_err_cnt  = 0;
static volatile uint32_t rs485_uart_err_code = 0;

/* RS485 原始帧调试开关：1 = 每收到一帧就按十六进制打出来。
 * 为什么需要它：只有 [Modbus] RX 那一行的话，"没收到"和"收到了但 CRC 不对
 * 被静默丢弃"看起来一模一样，排查接线时没法区分。
 * 接线验证完改回 0。 */
#define RS485_RX_DEBUG   1

/* 网关周期下发状态 */
static uint8_t gw_toggle_state = 0;

/* 线程安全打印：多任务并发 printf 会交错乱码，用互斥锁串行化。
 * 定义上移到此处，让 Offline_Cache_Push 等更靠前的函数也能用 LOG。 */
#define LOG(fmt, ...) do { \
    xSemaphoreTake(Print_Mutex, portMAX_DELAY); \
    printf(fmt, ##__VA_ARGS__); \
    xSemaphoreGive(Print_Mutex); \
} while(0)

/* ================= RS485 物理层自检 ================= */
/* 现象是「一个字节都收不到、DMA 计数器纹丝不动」，这时有两种完全不同的原因：
 *   A. 线上带电平，但 USART2/DMA 没在收（配置问题）
 *   B. 线上根本没有电平变化（接线/共地/收发器问题）
 * 光看收发结果区分不出来，所以下面两个工具直接从引脚和寄存器层面取证。
 * 排查完成后把 RS485_PIN_DEBUG 改回 0。 */
#define RS485_PIN_DEBUG          1   /* 1 = 每轮空转时采样 PA3 电平跳变并打印外设快照 */
#define RS485_LOOPBACK_SELFTEST  1   /* 1 = 开机跑一次 PA2->PA3 回环自检 */
#define RS485_TX_BEACON          1   /* 1 = 收不到数据时周期性往总线上发一段 ASCII，验证 A/B */

/**
 * @brief 用内部上拉/下拉判断 PA3 是被驱动还是悬空
 * @param pu 上拉后的电平，pd 下拉后的电平
 */
static void RS485_Pin_Drive_Test(uint32_t *pu, uint32_t *pd)
{
    uint32_t pupd_save = GPIOA->PUPDR;
    volatile uint32_t d;

    GPIOA->PUPDR = (pupd_save & ~(3UL << (3U * 2U))) | (1UL << (3U * 2U));
    for (d = 0U; d < 3000U; d++) { }
    *pu = (GPIOA->IDR & GPIO_PIN_3) ? 1UL : 0UL;

    GPIOA->PUPDR = (pupd_save & ~(3UL << (3U * 2U))) | (2UL << (3U * 2U));
    for (d = 0U; d < 3000U; d++) { }
    *pd = (GPIOA->IDR & GPIO_PIN_3) ? 1UL : 0UL;

    GPIOA->PUPDR = pupd_save;
}

#if RS485_TX_BEACON
/**
 * @brief 反向探针：主动往 RS485 总线上发一段 ASCII，看 PC 侧能不能收到
 *
 * 用途：PA3 已经被 RO 驱动起来（driven HIGH）却 edges=0 时，说明模块收不到
 *       A/B 上的差分信号。让板子往反方向发一次，就能判断 A/B 到底通不通：
 *         PC 收到 "GATEWAY-485-HELLO" -> A/B 是通的，问题在转换器的发送侧
 *         PC 什么都收不到            -> A/B 断线（或模块的 DI 没接到 PA2）
 *
 * 为什么不用 RS485_Send_Data：那里走 DMA + 等待 TC 信号量，一旦回调不来任务就
 * 永久挂死。诊断代码不能用会挂死的路径，所以这里全部裸寄存器轮询。
 */
static void RS485_Tx_Beacon(void)
{
    static const uint8_t msg[] = "GATEWAY-485-HELLO\r\n";
    uint16_t i;
    volatile uint32_t guard;

    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);   /* 切到发送态 */
    for (guard = 0U; guard < 30000UL; guard++) { }                       /* 等 DE 建立 */

    /* 发送态下判别 EN 到底通没通：
     * MAX3485 模块的 DE 和 /RE 通常并成一个 EN 脚，切到发送后 /RE=1，
     * RO 变成高阻 -> 这里应该读到 FLOATING。
     * 若仍读到 driven HIGH，说明模块压根没进发送态，也就是 PC4(EN) 没接到模块。 */
    {
        uint32_t pu, pd;
        RS485_Pin_Drive_Test(&pu, &pd);
        LOG("[RS485] beacon TXmode: PA3 PU=%lu PD=%lu -> %s\r\n",
            (unsigned long)pu, (unsigned long)pd,
            (pu == 0UL) ? "driven LOW" :
            (pd == 1UL) ? "still driven HIGH (EN not connected?)" :
                          "FLOATING (OK, module entered TX mode)");
    }

    for (i = 0U; i < (uint16_t)(sizeof(msg) - 1U); i++)
    {
        guard = 0U;
        while ((USART2->SR & USART_SR_TXE) == 0U) { if (++guard > 2000000UL) { break; } }
        USART2->DR = msg[i];
    }
    guard = 0U;
    while ((USART2->SR & USART_SR_TC) == 0U) { if (++guard > 2000000UL) { break; } }

    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET); /* 切回接收态 */
}
#endif

#if RS485_PIN_DEBUG
/**
 * @brief 绕开 USART/DMA，直接盯 PA3 这根线：数电平跳变，并打印收发器寄存器快照
 *
 * 原理：GPIOx->IDR 反映的是引脚的真实电平，跟它当前配成复用功能还是普通 IO 无关，
 *       所以 PA3 已经配成 AF7(USART2_RX) 也照样能读。
 *
 * 怎么读结果：
 *   edges > 0        线路上确实有电平变化 → 接线没问题，问题在 USART2/DMA 配置
 *   edges == 0       引脚上一直是个死电平 → 接线、共地、收发器供电/A-B 极性
 *   DMAR=0 或 DMA_EN=0  接收链路压根没启动（HAL_UARTEx_ReceiveToIdle_DMA 失败）
 *   SR 里 FE/NE/ORE 被置位  字节已经到了 USART，但帧格式/波特率不对
 */
static void RS485_Pin_Probe(uint32_t ms)
{
    uint32_t edges = 0U, last, now, t0;
    uint16_t i;
    uint32_t cr3, dma_cr, dma_ndtr, sr;

    last = (GPIOA->IDR & GPIO_PIN_3) ? 1U : 0U;
    t0   = HAL_GetTick();

    while ((HAL_GetTick() - t0) < ms)
    {
        /* 内层 400 次循环大约 3us，采样率足够看清 115200 下 8.7us 的一位 */
        for (i = 0U; i < 400U; i++)
        {
            now = (GPIOA->IDR & GPIO_PIN_3) ? 1U : 0U;
            if (now != last) { edges++; last = now; }
        }
    }

    /* ---- 浮空 / 被驱动 判别 ----
     * 依次把 PA3 配成「内部上拉」和「内部下拉」，各读一次电平：
     *   上拉读到 1、下拉读到 0  -> 线上什么都没有，是浮空的（杜邦线没接通）
     *   上拉也是 0              -> 有东西把它主动拉低（RO 输出低 / A-B 接反 / 对地短路）
     *   下拉也是 1              -> 有东西把它主动拉高（RO 正常空闲）
     * MAX3485 的 RO 是推挽输出，约 40k 的内部上下拉压不动它，所以这个判断可靠。
     * PUPDR 和 MODER 是两套独立寄存器，AF7 模式下照样能改上下拉。 */
    {
        uint32_t pu, pd;

        RS485_Pin_Drive_Test(&pu, &pd);

        /* 快照要在采样结束之后读，免得 SR 里的错误标志被提前清掉 */
        cr3      = USART2->CR3 & USART_CR3_DMAR;
        sr       = USART2->SR;
        dma_cr   = DMA1_Stream5->CR & DMA_SxCR_EN;
        dma_ndtr = DMA1_Stream5->NDTR;

        LOG("[RS485] PA3 level=%lu edges=%lu PU=%lu PD=%lu -> %s\r\n",
            (unsigned long)last, (unsigned long)edges,
            (unsigned long)pu, (unsigned long)pd,
            (pu == 0UL) ? "driven LOW" :
            (pd == 1UL) ? "driven HIGH" : "FLOATING (wire not connected?)");
        LOG("[RS485] RXstate=0x%02X DMAR=%lu DMA_EN=%lu NDTR=%lu SR=%04lX MODER=0x%08lX AFR0=0x%08lX\r\n",
            (unsigned)huart2.RxState,
            (unsigned long)(cr3 ? 1UL : 0UL),
            (unsigned long)(dma_cr ? 1UL : 0UL),
            (unsigned long)dma_ndtr,
            (unsigned long)sr,
            (unsigned long)GPIOA->MODER, (unsigned long)GPIOA->AFR[0]);
    }
}
#endif

/* ---- CAN 节点在线状态与数据合理性校验 ---- */
#define NODE_OFFLINE_TIMEOUT_MS  10000   /* 10s 无 CAN 数据判定节点掉线 */
#define DATA_TEMP_MIN   -40.0f          /* 温度合理范围下限 (°C) */
#define DATA_TEMP_MAX    85.0f
#define DATA_HUMI_MIN     0.0f          /* 湿度合理范围下限 (%) */
#define DATA_HUMI_MAX   100.0f

static TickType_t last_can_rx_tick = 0;  /* 最近一次收到合法 CAN 数据的时间 */
static uint8_t    node_online      = 0;  /* 0=未知/掉线 1=在线 */
static uint8_t    have_can_data    = 0;  /* 是否曾收到过数据（上电阶段不误报掉线） */
static uint32_t   bad_data_cnt     = 0;  /* 脏数据帧计数 */

/* ==================== 看门狗（任务心跳监控） ====================
 * 每个业务任务在循环里调用 WDT_Alive() 打心跳；
 * Watchdog_Task 周期性检查所有任务最近是否都存活，全部存活才喂 IWDG。
 * 任一任务卡死 → 停止喂狗 → 约 8s 后 IWDG 复位整机，实现自动恢复。 */
#define WDT_TASK_NUM     4
#define WDT_TIMEOUT_MS   6000   /* 任务超过 6s 无心跳判定卡死 */

typedef enum {
    WDT_DATA_PROC = 0,
    WDT_RS485_PROC,
    WDT_GW_CMD,
    WDT_MQTT_PROC,
} wdt_task_id_t;

static volatile TickType_t wdt_alive_tick[WDT_TASK_NUM] = {0};

static void WDT_Alive(wdt_task_id_t id)
{
    wdt_alive_tick[id] = xTaskGetTickCount();
}

/* 供 esp8266.c 在 AT 命令等待期间调用：
 * AT 等待最长达 15s，必须在此期间替 MQTT 任务打心跳，否则会被看门狗复位 */
void App_MqttWdtAlive(void)
{
    WDT_Alive(WDT_MQTT_PROC);
}

/* 看门狗监控任务：优先级最低，周期性检查所有业务任务心跳，
 * 全部存活才喂 IWDG；任一任务卡死则不喂，等待 IWDG 超时复位。 */
static void Watchdog_Task(void *pvParameters)
{
    (void)pvParameters;

    for(;;)
    {
        TickType_t now = xTaskGetTickCount();
        uint8_t all_alive = 1;

        for (uint8_t i = 0; i < WDT_TASK_NUM; i++)
        {
            if ((now - wdt_alive_tick[i]) >= pdMS_TO_TICKS(WDT_TIMEOUT_MS))
            {
                all_alive = 0;
                break;
            }
        }

        if (all_alive)
        {
            IWDG_Feed();   /* 全部存活才喂狗 */
        }

        vTaskDelay(pdMS_TO_TICKS(1000));   /* 1s 检查一次 */
    }
}

/* ==================== 掉线数据本地缓存（W25Q16 持久化） ====================
 * WiFi 断线时上行 JSON 无法 PUBLISH。原来用 16KB RAM 环形缓冲，掉电即丢；
 * 现在落到 W25Q16 日志式缓存，复位/断电后仍能补发。
 * 实现见 interface/offline_cache.c，这里只保留调用点。 */

/* 上报节点在线/掉线状态：复用 alarm 主题，走掉线缓存保证不丢 */
static void Report_Node_Status(uint8_t online)
{
    UPLOAD_ITEM it;
    it.topic = 1;   /* alarm 主题 */
    snprintf(it.data, sizeof(it.data), "{\"ts\":%lu,\"type\":\"%s\"}",
             (unsigned long)Time_Sync_Now(),
             online ? "node_online" : "node_offline");
    if (xQueueSend(Upload_Queue, &it, 0) != pdTRUE)
        Offline_Cache_Push(it.topic, it.data);
    LOG("[EDGE] Node %s\r\n", online ? "online" : "offline");
}

/**
 * @brief CAN 数据分发任务：消费中断入队的报文，按节点 ID 解析
 *        这是网关唯一的 CAN 接收路径（中断 -> 队列 -> 本任务）
 */
static void Data_Process_Task(void *pvParameters)
{
    CAN_REC_MSG msg;

    for(;;)
    {
        /* 1s 超时等待：即使无数据也周期性醒来打看门狗心跳 */
        if (xQueueReceive(CAN_Msg_Queue, &msg, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            switch (msg.RxHeader.StdId)
            {
                case 1: // F103 温湿度节点
                {
                    float raw_temp, raw_humi, ft, fh, alarm_val;
                    uint8_t atype;
                    UPLOAD_ITEM it;
                    edge_report_t rep;

                    memcpy(&raw_temp, &msg.data[0], 4);
                    raw_humi = (float)((msg.data[4] << 8) | msg.data[5]) / 10.0f;

                    /* 数据合理性校验：节点发来超范围脏数据时丢弃，
                     * 避免污染上报 JSON 和 Modbus 寄存器 */
                    if (raw_temp < DATA_TEMP_MIN || raw_temp > DATA_TEMP_MAX ||
                        raw_humi < DATA_HUMI_MIN || raw_humi > DATA_HUMI_MAX) {
                        bad_data_cnt++;
                        LOG("[GW RX] Invalid data dropped (temp=%.1f humi=%.1f, total=%lu)\r\n",
                            raw_temp, raw_humi, (unsigned long)bad_data_cnt);
                        break;
                    }

                    /* 收到合法数据：刷新在线时间戳；若之前判掉线则报恢复 */
                    last_can_rx_tick = xTaskGetTickCount();
                    have_can_data = 1;
                    if (!node_online) {
                        node_online = 1;
                        Report_Node_Status(1);
                    }

                    LOG("[GW RX] Node %d -> Temp: %.1f C, Humi: %.1f%%\r\n",
                        msg.RxHeader.StdId, raw_temp, raw_humi);

                    /* 边缘处理：滑动平均 + 死区过滤 + 阈值告警 */
                    rep = Edge_Process(raw_temp, raw_humi, &ft, &fh, &atype, &alarm_val);

                    /* Modbus 寄存器用滤波后的值，RS485 主站读到的是平滑数据 */
                    mb_hold_reg[MB_REG_TEMP] = (int16_t)(ft * 10.0f);
                    mb_hold_reg[MB_REG_HUMI] = (int16_t)(fh * 10.0f);

                    if (rep == EDGE_REPORT_DATA) {
                        it.topic = 0;
                        /* ts 是「采集时刻」而不是「发送时刻」：断网期间的数据
                         * 会先进 W25Q16 缓存，恢复后原样补发，时间语义才不会丢 */
                        snprintf(it.data, sizeof(it.data),
                                 "{\"ts\":%lu,\"node\":%lu,\"temp\":%.1f,\"humi\":%.1f}",
                                 (unsigned long)Time_Sync_Now(),
                                 (unsigned long)msg.RxHeader.StdId, ft, fh);
                        if (xQueueSend(Upload_Queue, &it, 0) != pdTRUE)
                            Offline_Cache_Push(it.topic, it.data);
                    } else if (rep == EDGE_REPORT_ALARM) {
                        const char *name = (atype == 1) ? "temp_high"
                                         : (atype == 2) ? "temp_low" : "recover";
                        it.topic = 1;
                        snprintf(it.data, sizeof(it.data),
                                 "{\"ts\":%lu,\"type\":\"%s\",\"temp\":%.1f}",
                                 (unsigned long)Time_Sync_Now(), name, alarm_val);
                        if (xQueueSend(Upload_Queue, &it, 0) != pdTRUE)
                            Offline_Cache_Push(it.topic, it.data);
                        LOG("[EDGE] Alarm: %s (%.1f C)\r\n", name, alarm_val);
                    }
                    /* EDGE_REPORT_NONE：死区内，静默丢弃，不占用上行带宽 */

                    /* 分钟级统计：周期到了就上报 avg/min/max，云端不用自己聚合 */
                    {
                        float at, tmin, tmax, ah, hmin, hmax;
                        uint32_t n;
                        if (Edge_Stats(&at, &tmin, &tmax, &ah, &hmin, &hmax, &n)) {
                            it.topic = 2;
                            snprintf(it.data, sizeof(it.data),
                                     "{\"ts\":%lu,\"temp_avg\":%.1f,\"temp_min\":%.1f,\"temp_max\":%.1f,"
                                     "\"humi_avg\":%.1f,\"humi_min\":%.1f,\"humi_max\":%.1f,\"n\":%lu}",
                                     (unsigned long)Time_Sync_Now(),
                                     at, tmin, tmax, ah, hmin, hmax, (unsigned long)n);
                            if (xQueueSend(Upload_Queue, &it, 0) != pdTRUE)
                                Offline_Cache_Push(it.topic, it.data);
                        }
                    }
                    break;
                }
                case 2: // 预留：未来第二个节点
                default:
                    break;
            }
        }
        else
        {
            /* 队列超时（1s 无数据）：检查数据源是否掉线 */
            if (have_can_data && node_online &&
                (xTaskGetTickCount() - last_can_rx_tick) >= pdMS_TO_TICKS(NODE_OFFLINE_TIMEOUT_MS)) {
                node_online = 0;
                Report_Node_Status(0);
            }
        }
        WDT_Alive(WDT_DATA_PROC);   /* 循环末尾打心跳（有数据/无数据都打） */
    }
}

/**
 * @brief RS485 数据处理任务：从 MessageBuffer 取「一条完整帧」并回显
 *        MessageBuffer 保留消息边界，溢出时整帧丢弃，不会出现半包错位
 */
static void RS485_Process_Task(void *pvParameters)
{
    uint8_t local_buf[RS485_MAX_LEN];
    uint8_t resp[RS485_MAX_LEN];
    size_t  len;
    uint32_t last_err_cnt = 0U;
    uint32_t beacon_div   = 0U;

    for(;;)
    {
        WDT_Alive(WDT_RS485_PROC);   /* 外循环打心跳 */

        /* 溢出计数：中断里只计数，打印放到任务上下文，避免中断里拿打印互斥量 */
        if (rs485_overflow_cnt)
        {
            uint32_t dropped;
            taskENTER_CRITICAL();
            dropped = rs485_overflow_cnt;
            rs485_overflow_cnt = 0;
            taskEXIT_CRITICAL();
            LOG("[RS485] MessageBuffer FULL, dropped %lu frame(s)\r\n",
                (unsigned long)dropped);
        }

        /* 一次收一条完整帧；超时返回 0，打心跳后继续等 */
        len = xMessageBufferReceive(RS485_MsgBuffer, local_buf,
                                    sizeof(local_buf), pdMS_TO_TICKS(1000));
        if (len == 0)
        {
            WDT_Alive(WDT_RS485_PROC);
#if RS485_RX_DEBUG
            /* 中断里只记账，这里统一打印：FE/ORE/NE 说明线上有字节但时序/电平不对 */
            if (rs485_uart_err_cnt != last_err_cnt)
            {
                last_err_cnt = rs485_uart_err_cnt;
                LOG("[RS485] UART error 0x%08lX (total %lu), RX re-armed\r\n",
                    (unsigned long)rs485_uart_err_code, (unsigned long)last_err_cnt);
            }
#endif
#if RS485_PIN_DEBUG
            /* 1 秒没凑出完整帧：用 900ms 贴近 100% 的占空比去数 PA3 的跳变，
             * 这样 PC 每发一帧基本都能落进采样窗口里。 */
            RS485_Pin_Probe(900U);
#endif
#if RS485_TX_BEACON
            /* 每 3 轮空转（约 6s）反向发一次，PC 那边收到就说明 A/B 是通的 */
            if (++beacon_div >= 3U)
            {
                beacon_div = 0U;
                RS485_Tx_Beacon();
                LOG("[RS485] TX beacon sent (A/B test)\r\n");
            }
#endif
            continue;
        }

#if RS485_RX_DEBUG
        {
            char     hex[128];
            uint16_t i, n = (uint16_t)((len < 32U) ? len : 32U);
            uint16_t pos = 0U;

            for (i = 0U; i < n; i++)
                pos += (uint16_t)snprintf(&hex[pos], sizeof(hex) - pos,
                                          " %02X", local_buf[i]);
            hex[pos] = '\0';
            LOG("[RS485] RX %u B:%s%s\r\n", (unsigned)len, hex,
                (len > 32U) ? " ..." : "");
        }
#endif

        /* Modbus RTU 从站：CRC 校验 + 生成响应 */
        uint16_t resp_len = Modbus_Process(local_buf, (uint16_t)len, resp);
        if (resp_len > 0)
        {
            RS485_Send_Data(resp, resp_len);
            LOG("[Modbus] RX %d B -> resp %d B (temp=%d, humi=%d)\r\n",
                (int)len, resp_len, mb_hold_reg[MB_REG_TEMP], mb_hold_reg[MB_REG_HUMI]);
        }
    }
}

/**
 * @brief 网关周期下发任务
 *        替代原来 main 里那段代码——vTaskStartScheduler() 不返回，
 *        main 的 while(1) 是死代码，周期下发必须放在任务里
 */
static void Gateway_Cmd_Task(void *pvParameters)
{
    for(;;)
    {
        WDT_Alive(WDT_GW_CMD);   /* 打心跳（周期 5s，小于 6s 判定阈值） */
        vTaskDelay(pdMS_TO_TICKS(5000));

        gw_toggle_state = !gw_toggle_state;

        Gateway_Send_Cmd(1, 0x03, gw_toggle_state);
        LOG("[GW TX] Sent LED CMD: %d\r\n", gw_toggle_state);

        uint16_t new_delay = gw_toggle_state ? 500 : 2000;
        Gateway_Send_Cmd(1, 0x02, new_delay);
        LOG("[GW TX] Sent Delay CMD: %d ms\r\n", new_delay);
    }
}

/* ---- OTA 请求：中断只管存 URL 和置标志，真正的下载在任务里做 ---- */
static char              ota_url_buf[128];
static volatile uint8_t  ota_req_pending = 0;

/* ---- 云端下行命令回调 ----
 * 由 MQTT_FeedByte 在 USART3 接收中断上下文调用：
 * 这里只做字符串解析并置标志，真正的 CAN 下发和 printf 交给 Mqtt_Upload_Task，
 * 因为 printf 走互斥量，在中断里调用会死锁。 */
static void Mqtt_OnPublish(const char *topic, const uint8_t *payload,
                           uint16_t len, uint8_t qos)
{
    static char buf[128];   /* 只在 USART3 中断里调用，不可重入，用静态省中断栈 */
    const char *p;
    const char *q;

    (void)qos;

    /* ---- 升级指令：{"ota":"http://192.168.137.1:8000/f407.bin"} ---- */
    /* 这里只把 URL 抠出来存着，下载/擦写/重启留给任务做：
     * 中断里既不能阻塞，也不能碰 Flash。 */
    if (strcmp(topic, TOPIC_OTA) == 0)
    {
        if (len >= sizeof(buf)) len = (uint16_t)(sizeof(buf) - 1u);
        memcpy(buf, payload, len);
        buf[len] = '\0';

        p = strstr(buf, "\"ota\"");
        if (p != NULL && (p = strchr(p, ':')) != NULL)
        {
            p++;                                        /* 跳过 ':' */
            while (*p == ' ' || *p == '"') p++;         /* 跳过空白和左引号 */

            q = strchr(p, '"');                         /* 到右引号为止 */
            if (q != NULL)
            {
                uint16_t n = (uint16_t)(q - p);
                if (n > 0U && n < (uint16_t)sizeof(ota_url_buf))
                {
                    memcpy(ota_url_buf, p, n);
                    ota_url_buf[n] = '\0';
                    ota_req_pending = 1;
                }
            }
        }
        return;
    }

    /* 只处理命令主题，其它主题直接忽略 */
    if (strcmp(topic, TOPIC_CMD) != 0) return;

    if (len >= sizeof(buf)) len = (uint16_t)(sizeof(buf) - 1u);
    memcpy(buf, payload, len);
    buf[len] = '\0';

    /* 极简 JSON 取值：{"led":1} / {"delay":500}
     * 报文很短，用字符串查找 + atoi 足够，不必引入 cJSON */
    if ((p = strstr(buf, "\"led\"")) != NULL && (p = strchr(p, ':')) != NULL) {
        cloud_cmd_code    = 0x03;
        cloud_cmd_param   = (atoi(p + 1) != 0) ? 1u : 0u;
        cloud_cmd_pending = 1;
    } else if ((p = strstr(buf, "\"delay\"")) != NULL && (p = strchr(p, ':')) != NULL) {
        int ms = atoi(p + 1);
        if (ms >= 100 && ms <= 60000) {          /* 参数范围保护 */
            cloud_cmd_code    = 0x02;
            cloud_cmd_param   = (uint16_t)ms;
            cloud_cmd_pending = 1;
        }
    }
}

/**
 * @brief SNTP 对时：连上 WiFi 后向模块要一次网络时间
 * @note  必须在进入透传之前调用。失败不致命——数据照发，只是 ts 字段为 0，
 *        下次重连会再试。
 * @retval 1 = 对时成功
 */
static uint8_t Mqtt_TimeSync(void)
{
    uint32_t unix_sec;
    uint32_t t0;

    if (!ESP8266_SNTP_Start()) return 0;

    /* 模块配好服务器后要等它完成一次同步，通常 1~3s，给 8s 余量 */
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 8000U)
    {
        if (ESP8266_QueryTime(&unix_sec))
        {
            Time_Sync_Set(unix_sec);
            return 1;
        }
        WDT_Alive(WDT_MQTT_PROC);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return 0;
}

/**
 * @brief 上行推送任务：ESP8266 WiFi 透传 + 自主封装 MQTT
 *        下行：订阅命令主题，处理云端下发的控制指令（QoS1，回 PUBACK）
 *        上行：QoS1 停等发布 + 超时重发，消费 Upload_Queue / 掉线缓存
 *        连接失败自动重试，周期发 PINGREQ 心跳保活
 */
static void Mqtt_Upload_Task(void *pvParameters)
{
    UPLOAD_ITEM it;
    static uint8_t mqtt_rx[256];   /* 下行缓冲：要能装下完整 PUBLISH 帧 */

    /* 缓存补发的「在途」标记（跨重连保留）：
     *   1 = 已从缓存 Peek 出来并发上路，但 PUBACK 还没回来，此时绝不能推进读指针。
     *   掉电/断线时这条记录原封不动留在 W25Q16 里，重连后会重新 Peek 出来再发一次
     *   ——「最坏重发一次」换「绝不丢失」，这就是 QoS1 至少一次在缓存层的落地。 */
    uint8_t cache_inflight = 0;

    /* 传输层 = ESP8266 透传；MQTT 报文在本 MCU 自主组帧 */
    MQTT_Init(&g_mqtt, ESP8266_Send, Mqtt_OnPublish, mqtt_rx, sizeof(mqtt_rx));

#if OTA_AUTO_TEST
    /* 调试：联网后先跑一次 HTTP 下载，把固件拉进 W25Q16 暂存区。
     * 必须在任务上下文跑 —— esp_expect 里用了 vTaskDelay。 */
    ota_auto_test();
#endif

    for(;;)
    {
        WDT_Alive(WDT_MQTT_PROC);   /* 外层重连循环也打心跳，防止长期连不上被误判卡死 */
        uint8_t ok = 1;
        LOG("[MQTT] Init ESP8266...\r\n");

        /* 每次重连都是新会话：清掉上一次残留的在途报文和解析状态 */
        MQTT_ResetSession(&g_mqtt);
        /* 上条缓存记录没等到 PUBACK 就断了，读指针尚未推进，记录还在 Flash 里，
         * 这里把标记清掉，重连后自然会重新 Peek 到同一条再发一次 */
        if (cache_inflight) {
            LOG("[MQTT] unacked cache record, will re-send after reconnect\r\n");
            cache_inflight = 0;
        }

        /* --- 链路：复位 -> 连 WiFi -> 建 TCP -> 透传 --- */
        if (!ESP8266_Reset()) {
            LOG("[MQTT] Reset fail\r\n"); ok = 0;
        } else {
            vTaskDelay(pdMS_TO_TICKS(2000));          /* 等模块稳定 */
            if      (!ESP8266_SetMode(1))            { LOG("[MQTT] SetMode fail\r\n");     ok = 0; }
            else if (!ESP8266_JoinAP(WIFI_SSID, WIFI_PASS)) { LOG("[MQTT] JoinAP fail\r\n"); ok = 0; }
            else {
                /* 对时：必须在 CIPMODE=1 + CIPSEND 之前做完，透传后 AT 命令会被当业务数据发走。
                 * 失败不影响链路，只是这批报文不带时间戳。 */
                if (Mqtt_TimeSync())
                    LOG("[TIME] SNTP synced: %lu\r\n", (unsigned long)Time_Sync_Now());
                else
                    LOG("[TIME] SNTP sync fail, ts disabled\r\n");

                if      (!ESP8266_OpenTCP(MQTT_BROKER, MQTT_PORT)) { LOG("[MQTT] OpenTCP fail\r\n"); ok = 0; }
                else if (!ESP8266_EnterTransparent())     { LOG("[MQTT] Transparent fail\r\n"); ok = 0; }
            }
        }

        /* --- MQTT 握手：CONNECT(带遗嘱) -> 等 CONNACK -> 订阅命令主题 --- */
        if (ok) {
            /* LWT：异常掉线时由 broker 代发 {"online":0}，与正常的 {"online":1} 配对 */
            /* 凭据在 wifi_config.h；公共 broker 匿名时 MQTT_USER/PASS 为 NULL */
            MQTT_Connect(&g_mqtt, "f407_gw", MQTT_USER, MQTT_PASS, 60,
                         TOPIC_ONLINE, "{\"online\":0}");
            LOG("[MQTT] CONNECT sent, waiting CONNACK...\r\n");

            uint32_t t0 = HAL_GetTick();
            while (!g_mqtt.connected && (HAL_GetTick() - t0) < 5000)
            {
                WDT_Alive(WDT_MQTT_PROC);   /* 等 CONNACK 期间也要打心跳 */
                vTaskDelay(pdMS_TO_TICKS(20));
            }

            if (!g_mqtt.connected) { LOG("[MQTT] CONNACK timeout\r\n"); ok = 0; }
            else {
                LOG("[MQTT] Connected!\r\n");
                /* 上线通知：retain=1，订阅者一订阅就能看到当前在线状态 */
                MQTT_Publish(&g_mqtt, TOPIC_ONLINE, "{\"online\":1}", 1);
                LOG("[MQTT] Online notify sent\r\n");

                /* 订阅云端命令主题（QoS1），用于接收下行控制 */
                MQTT_Subscribe(&g_mqtt, TOPIC_CMD, 1);
                LOG("[MQTT] SUBSCRIBE %s\r\n", TOPIC_CMD);

                /* 订阅升级指令主题：固件本体不走 MQTT（QoS1 停等 + 3s 超时
                 * 传几百 KB 要一小时），这里只收一条带 URL 的短指令 */
                vTaskDelay(pdMS_TO_TICKS(100));
                MQTT_Subscribe(&g_mqtt, TOPIC_OTA, 1);
                LOG("[MQTT] SUBSCRIBE %s\r\n", TOPIC_OTA);
            }
        }

        if (!ok) {
            /* 断线：把 Upload_Queue 里积压的数据搬进掉线缓存，避免占满队列后新数据被丢 */
            UPLOAD_ITEM tmp;
            while (xQueueReceive(Upload_Queue, &tmp, 0) == pdTRUE) {
                Offline_Cache_Push(tmp.topic, tmp.data);
            }
            LOG("[MQTT] Reconnect in 3s...\r\n");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        /* --- 发布主循环：执行下行命令 + 心跳保活 + QoS1 上行 --- */
        TickType_t last_ping = xTaskGetTickCount();
        uint8_t    ping_pending = 0;      /* 1=已发 PINGREQ 正在等 PINGRESP */
        TickType_t ping_sent_tick = 0;

        for(;;)
        {
            WDT_Alive(WDT_MQTT_PROC);   /* 内层循环打心跳（50ms 周期） */
            uint32_t now = HAL_GetTick();

            /* 0. 云端下发了升级指令：回执一条，然后把 ESP8266 整个抢过来下载固件。
             *    下载成功后 ota_run_url() 内部直接 NVIC_SystemReset()，不会再回来；
             *    能走到下面的 break 就说明升级失败，跳出内层交给外层重连 MQTT。 */
            if (ota_req_pending) {
                static char ota_url[128];   /* 静态：MqttProc 栈只有 2KB */
                int rc;

                taskENTER_CRITICAL();
                ota_req_pending = 0;
                strcpy(ota_url, ota_url_buf);
                taskEXIT_CRITICAL();

                LOG("[OTA] request: %s\r\n", ota_url);
                MQTT_Publish(&g_mqtt, TOPIC_OTA_ST, "{\"stage\":\"start\"}", 0);
                vTaskDelay(pdMS_TO_TICKS(200));   /* 让回执先发出去再掐链路 */

                rc = ota_run_url(ota_url);

                LOG("[OTA] FAIL rc=%d, resume MQTT\r\n", rc);
                MQTT_Publish(&g_mqtt, TOPIC_OTA_ST, "{\"stage\":\"failed\"}", 0);
                vTaskDelay(pdMS_TO_TICKS(200));
                break;
            }

            /* 1. 执行云端下行命令（中断只置标志，这里才做 CAN 下发和打印）
             *    取参数和清标志放临界区，避免中断在两步之间插入新命令导致丢指令 */
            if (cloud_cmd_pending) {
                uint8_t  code;
                uint16_t param;
                taskENTER_CRITICAL();
                code  = cloud_cmd_code;
                param = cloud_cmd_param;
                cloud_cmd_pending = 0;
                taskEXIT_CRITICAL();
                Gateway_Send_Cmd(1, code, param);
                LOG("[Cloud CMD] code=0x%02X param=%u\r\n", code, param);
            }

            /* 2. 回下行 QoS1 的 PUBACK（中断只置标志，这里真正发送） */
            if (g_mqtt.puback_reply_pending) {
                uint16_t id = g_mqtt.puback_reply_id;
                g_mqtt.puback_reply_pending = 0;
                MQTT_SendPuback(&g_mqtt, id);
            }

            /* 3. 心跳：每 30s 发一次 PINGREQ（keepalive=60s） */
            if ((xTaskGetTickCount() - last_ping) >= pdMS_TO_TICKS(30000)) {
                MQTT_Ping(&g_mqtt);                  /* 内部会清 pingresp_recv */
                last_ping      = xTaskGetTickCount();
                ping_sent_tick = xTaskGetTickCount();
                ping_pending   = 1;
            }

            /* 断线检测：发了心跳却超时未收到 PINGRESP，判定链路异常，跳出重连 */
            if (ping_pending) {
                if (g_mqtt.pingresp_recv) {
                    ping_pending = 0;                /* 收到应答，链路正常 */
                } else if ((xTaskGetTickCount() - ping_sent_tick) >= pdMS_TO_TICKS(5000)) {
                    LOG("[MQTT] PINGRESP timeout, link lost! Reconnect...\r\n");
                    break;                           /* 跳出内层 -> 外层重新初始化链路 */
                }
            }

            /* 4. QoS1 上行超时重发：重发次数耗尽说明链路已死，重连 */
            if (MQTT_Poll(&g_mqtt, now) == 2) {
                LOG("[MQTT] QoS1 retry exhausted, link lost! Reconnect...\r\n");
                break;
            }

            /* 5. 停等：没有在途报文时才发下一条；
             *    优先补发断线缓存（先旧后新），再发实时队列，保证时间顺序。
             *
             *    缓存这里必须 Peek 而不是 Pop：Pop 是「读出来 + 立刻推进指针」，
             *    一旦 PUBACK 没回来（掉电 / 断网 / 链路半死），这条记录就从
             *    Flash 里消失了，云端永远收不到。改用 Peek -> 发 -> 收到 PUBACK
             *    才 Commit，代价是极端情况下重复上报一次，换来的是不丢。
             *    云端按 (node, ts) 去重即可。 */
            if (!g_mqtt.puback_pending) {
                /* 上一条缓存记录已确认送达，这时才推进读指针 */
                if (cache_inflight) {
                    cache_inflight = 0;
                    Offline_Cache_Commit();
                }

                if (Offline_Cache_Peek(&it.topic, it.data, (uint16_t)(sizeof(it.data) - 1U)) > 0) {
                    if (MQTT_Publish_QoS1(&g_mqtt, Topic_Of(it.topic), it.data, 0, now)) {
                        cache_inflight = 1;      /* 发出去了，等 PUBACK 再提交 */
                        LOG("[MQTT] Re-send(QoS1): %s\r\n", it.data);
                    }
                    /* 发送失败不进 Commit：记录留在缓存里，下轮再试 */
                } else if (xQueueReceive(Upload_Queue, &it, 0) == pdTRUE) {
                    /* 实时数据直接来自队列，不存在「提交」的问题 */
                    MQTT_Publish_QoS1(&g_mqtt, Topic_Of(it.topic), it.data, 0, now);
                    LOG("[MQTT] Publish(QoS1): %s\r\n", it.data);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

/**
 * @brief 栈溢出钩子（configCHECK_FOR_STACK_OVERFLOW=2 时被内核调用）
 *        某个任务栈写穿时进入这里，停住方便用调试器定位
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for(;;)
    {
    }
}

#if RS485_LOOPBACK_SELFTEST
/**
 * @brief 开机跑一次 PA2(USART2_TX) -> PA3(USART2_RX) 硬回环自检
 *
 * 目的：把「MCU 侧 USART2 + DMA + IDLE 中断 + MessageBuffer」这整条链单独验一遍，
 *       和外部收发器、杜邦线彻底解耦。收不到数据时先看这一行，能一刀切分责任。
 *
 * 用法：把板子上的 PA2 和 PA3 用一根杜邦线直接短接，然后复位。
 *       ⚠ 短接前先把 MAX3485 模块上接到 PA3 的那根线（模块的 RX/RO）拔掉，
 *         否则收发器的 RO 输出会和 PA2 对顶。
 *
 * 结果：
 *   PASS —— MCU 侧链路完好，收不到就一定是外部接线 / 共地 / 收发器的问题
 *   FAIL —— 软件配置问题（回调没进、DMA 没启动、引脚被别的外设占着）
 *   没短接时打 FAIL 是正常的，不影响后面正常接收。
 */
static void RS485_Loopback_SelfTest(void)
{
    static const uint8_t tx[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
    uint8_t  rx[8];
    uint16_t i, got = 0U, pos = 0U;
    char     hex[40];
    volatile uint32_t guard;

    /* ！！这里刻意全部用寄存器裸操作！！不能调 HAL_UART_Transmit / HAL_Delay /
     * xMessageBufferReceive：
     *   1) 此时 vTaskStartScheduler() 还没调用，任何会 block 的 RTOS API 都没有调度器可用；
     *   2) 更不能让接收走 DMA + IDLE 中断 —— 短接 PA2/PA3 时发送会立刻触发
     *      HAL_UARTEx_RxEventCallback，里面的 portYIELD_FROM_ISR 在 pxCurrentTCB 还是
     *      NULL 的时候触发 PendSV，当场死机，日志就停在 "[CACHE] flash cache ready" 之后；
     *   3) HAL_Delay 依赖 HAL 时基，启动早期没验证过，tick 不走就是死等。
     * 裸轮询 + 计数上限则无论如何都会在有限步内返回。 */
    LOG("[RS485] loopback selftest start (PA2/PA3 jumpered?)\r\n");

    /* 边发边收：USART 的 DR 只有一级深（没开 DMA 也没有接收中断）。
     * 如果先把 8 个字节全发完再回头读，前面的字节早被后一个覆盖掉了，
     * 最后只剩下最后一个字节 + ORE —— 上一版就栽在这里，报了 "1 B 内容不符"。 */
    for (i = 0U; i < sizeof(tx); i++)
    {
        guard = 0U;
        while ((USART2->SR & USART_SR_TXE) == 0U)
        {
            if (++guard > 2000000UL) { goto loopback_done; }
        }
        USART2->DR = tx[i];

        /* 等这一字节绕回来：115200 下约 87us */
        guard = 0U;
        while ((USART2->SR & USART_SR_RXNE) == 0U)
        {
            if (++guard > 200000UL) { goto loopback_done; }
        }
        if (got < sizeof(rx))
        {
            rx[got++] = (uint8_t)(USART2->DR & 0xFFU);
        }
    }

loopback_done:
    for (i = 0U; i < got; i++)
    {
        pos += (uint16_t)snprintf(&hex[pos], sizeof(hex) - pos, " %02X", rx[i]);
    }
    hex[pos] = '\0';

    if (got == sizeof(tx) && memcmp(rx, tx, sizeof(tx)) == 0)
    {
        LOG("[RS485] loopback PASS (%u B echoed) - MCU side OK\r\n", (unsigned)got);
    }
    else
    {
        LOG("[RS485] loopback FAIL (%u B:%s) - PA2/PA3 not jumpered or MCU side broken\r\n",
            (unsigned)got, (got > 0U) ? hex : " none");
    }
}
#endif

void App_RTOS_Init(void)
{
    printf("--- System OS Initializing... ---\r\n");

    Edge_Init();                                 /* 边缘计算模块上电初始化 */

    Print_Mutex         = xSemaphoreCreateMutex();
    CAN_Msg_Queue      = xQueueCreate(10, sizeof(CAN_REC_MSG));
    RS485_MsgBuffer    = xMessageBufferCreate(RS485_MSGBUF_SIZE);
    RS485_Tx_Semaphore = xSemaphoreCreateBinary();
    Upload_Queue       = xQueueCreate(10, sizeof(UPLOAD_ITEM));  /* 元素带主题的 JSON */

    if (CAN_Msg_Queue == NULL || RS485_MsgBuffer == NULL ||
        RS485_Tx_Semaphore == NULL || Print_Mutex == NULL || Upload_Queue == NULL)
    {
        printf("--- RTOS Object Create FAILED! ---\r\n");
        Error_Handler();
    }

    /* 挂载 W25Q16 离线缓存：扫描 meta、修正掉电时写了一半的尾部。
     * 必须在任务启动前完成，之后 DataProc 才往 Flash 里追加记录。 */
    Offline_Cache_Init();

    {
        uint32_t cnt, drops, seq;
        Offline_Cache_Stats(&cnt, &drops, &seq);
        printf("[CACHE] flash cache ready: %lu pending, %lu dropped, seq=%lu\r\n",
               (unsigned long)cnt, (unsigned long)drops, (unsigned long)seq);
    }

    // CAN 接收统一走「中断 -> 队列」
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // RS485 初始化为接收态
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);

#if RS485_LOOPBACK_SELFTEST
    /* 必须在 arm 接收 DMA 之前跑：一旦 DMAR 置位，收到的字节就归 DMA 拿，
     * 这里的轮询读永远读不到。 */
    RS485_Loopback_SelfTest();
#endif

    // 启动「空闲分帧」DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RS485_Rx_Buf[0], RS485_MAX_LEN);

    // 启动 ESP8266(USART3) 单字节中断接收，供 AT 命令与 MQTT 解析使用
    ESP8266_UART_Start();

    // 创建任务（数字越大优先级越高）
    xTaskCreate(Data_Process_Task,  "DataProc",  512, NULL, 2, NULL);
    xTaskCreate(RS485_Process_Task, "RS485Proc", 512, NULL, 2, NULL);
    xTaskCreate(Gateway_Cmd_Task,   "GwCmd",     256, NULL, 1, NULL);
    xTaskCreate(Mqtt_Upload_Task,   "MqttProc",  512, NULL, 2, NULL);
    // 看门狗监控任务：优先级最低，所有业务任务存活才喂 IWDG
    xTaskCreate(Watchdog_Task,      "Watchdog",  256, NULL, 0, NULL);

    printf("--- RS485 MessageBuffer Engine Started! ---\r\n");
    printf("--- OS Scheduler Started! ---\r\n");
    vTaskStartScheduler();
}

/* ================= 中断回调 ================= */

/**
 * @brief CAN FIFO0 收到数据的中断回调：入队，由 Data_Process_Task 解析
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        CAN_REC_MSG rx_msg;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_msg.RxHeader, rx_msg.data) == HAL_OK)
        {
            xQueueSendFromISR(CAN_Msg_Queue, &rx_msg, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @brief RS485 接收完成回调（IDLE 分帧 / 满缓冲兜底）
 *        在中断上下文被 HAL 调用。把刚收满的一块缓冲区作为「一条完整消息」
 *        写入 MessageBuffer，然后重启另一块接收。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2 && Size > 0 && Size <= RS485_MAX_LEN)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        uint8_t cur = rs485_rx_sel;              /* 刚收满的缓冲 */
        rs485_rx_sel = 1 - rs485_rx_sel;         /* 切换到另一块 */

        /* 立即用另一块缓冲重启 DMA，实现 Ping-Pong 无缝切换 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RS485_Rx_Buf[rs485_rx_sel], RS485_MAX_LEN);

        /* 整帧原子写入：空间不够返回 0，整帧丢弃，绝不会半包 */
        if (xMessageBufferSendFromISR(RS485_MsgBuffer, RS485_Rx_Buf[cur], Size,
                                      &xHigherPriorityTaskWoken) == 0)
        {
            rs485_overflow_cnt++;
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief USART2 出错后的善后：记账 + 重新挂上 DMA 接收
 *
 * 由 esp8266.c 里那个唯一的 HAL_UART_ErrorCallback 转发过来（全工程只能有一份）。
 *
 * 为什么必须处理：HAL 一旦检测到 FE/ORE/NE，会在内部调用 UART_EndRxTransfer()，
 * 把 CR3 的 DMAR 清掉、把 DMA 流关掉、把 RxState 复位成 READY，**但它自己不会
 * 重新开接收**。不重新 arm，线上一旦出现一次帧错误，RS485 就永久失聪，
 * 表现就是「只收到过 1 个字节，之后全是 0」—— 2026-11-11 实测就踩了这个坑。
 *
 * 注意：本函数在中断上下文执行，不能调 LOG（里面有互斥量），只记账。
 */
void RS485_Uart_Error_Rearm(uint32_t err_code)
{
    rs485_uart_err_code = err_code;
    rs485_uart_err_cnt++;

    /* 重新挂上 DMA 接收，从 0 号缓冲重新开始 */
    rs485_rx_sel = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RS485_Rx_Buf[0], RS485_MAX_LEN);
}

/**
 * @brief 串口发送完成回调
 *        注意：HAL 对 DMA 发送会在「TC 标志置位（最后一字节真正移出）」后才回调
 *        本函数，所以在这里拉低 DE 是安全的。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(RS485_Tx_Semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief 线程安全的 RS485 发送
 *        DE 拉高 -> DMA 发送 -> 等 TC 回调 -> DE 拉低切回接收态
 */
void RS485_Send_Data(uint8_t *data, uint16_t len)
{
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET); // 进入发送态

    HAL_UART_Transmit_DMA(&huart2, data, len);

    // 阻塞等待 TC 回调（HAL 保证此时最后一字节已真正发完）
    xSemaphoreTake(RS485_Tx_Semaphore, portMAX_DELAY);

    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET); // 切回接收态
}
