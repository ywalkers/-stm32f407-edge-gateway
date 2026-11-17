#ifndef __MQTT_H__
#define __MQTT_H__

#include <stdint.h>

/* ============================================================
 *  MQTT 3.1.1 客户端（自主封装报文，不依赖第三方库）
 *  - 上行：CONNECT(带 LWT) / PUBLISH(QoS0/QoS1) / SUBSCRIBE / PINGREQ / PUBACK
 *  - 下行：CONNACK / PUBACK / SUBACK / PUBLISH / PINGRESP
 *  - 接收：固定头 -> 变长剩余长度 -> payload 的三态机，逐字节喂入
 * ============================================================ */

/* 底层字节发送函数原型：由传输层（ESP8266 透传）提供 */
typedef void (*mqtt_send_fn)(const uint8_t *data, uint16_t len);

/* 下行 PUBLISH 回调
 * 注意：运行在 USART3 接收中断上下文，禁止阻塞、禁止 printf/互斥量 */
typedef void (*mqtt_publish_fn)(const char *topic, const uint8_t *payload,
                                uint16_t len, uint8_t qos);

#define MQTT_ACK_TIMEOUT_MS   3000   /* QoS1 等待 PUBACK 的超时时间 */
#define MQTT_TX_RETRY_MAX     3      /* QoS1 最多重发次数 */
#define MQTT_TX_BUF_SIZE      256    /* 在途报文备份（停等，一次只存一条） */

/* 接收状态机状态 */
enum {
    MQTT_RX_FIXED = 0,   /* 等固定头 1 字节 */
    MQTT_RX_REMLEN,      /* 等变长剩余长度（1~4 字节） */
    MQTT_RX_PAYLOAD      /* 等 payload */
};

typedef struct {
    mqtt_send_fn    send;        /* 底层发送回调 */
    mqtt_publish_fn on_publish;  /* 下行 PUBLISH 回调（可空） */

    uint8_t  *rxbuf;             /* 接收缓冲（存 payload） */
    uint16_t  rxcap;             /* 接收缓冲容量 */
    uint16_t  keepalive;         /* 保活时间（秒） */

    /* ---- 接收解析状态（USART 中断上下文访问） ---- */
    uint8_t   rx_state;
    uint8_t   rx_hdr;            /* 固定头字节 */
    uint32_t  rx_mult;           /* 剩余长度乘数 1/128/16384/2097152（必须 32 位，否则溢出） */
    uint32_t  rx_rem;            /* 剩余长度目标值 */
    uint32_t  rx_got;            /* 已收 payload 字节数 */

    /* ---- 连接状态：中断写、任务读 ---- */
    volatile uint8_t connected;      /* 1=已收到 CONNACK(成功) */
    volatile uint8_t pingresp_recv;  /* 1=最近一次 PINGREQ 已收到 PINGRESP */
    volatile uint8_t suback_recv;    /* 1=已收到 SUBACK */

    /* ---- 下行 QoS1：中断只置标志，由任务真正发 PUBACK ---- */
    volatile uint8_t  puback_reply_pending;
    volatile uint16_t puback_reply_id;

    /* ---- 上行 QoS1（停等：同一时刻只有一条在途） ---- */
    uint16_t  packet_id;             /* 报文标识符自增 */
    volatile uint8_t  puback_pending;
    volatile uint16_t puback_id;
    volatile uint8_t  puback_retry;
    uint32_t  puback_tick;           /* 发出时刻（调用方传 HAL_GetTick） */
    uint8_t   txbuf[MQTT_TX_BUF_SIZE];  /* 在途报文备份，用于重发 */
    uint16_t  txlen;
} MQTT_Client_t;

/* 全局 MQTT 客户端实例（单例，esp8266 接收回调里会喂它） */
extern MQTT_Client_t g_mqtt;

/* 初始化客户端（rxbuf 需静态分配，rxcap 建议 >= 256） */
void MQTT_Init(MQTT_Client_t *c, mqtt_send_fn send, mqtt_publish_fn on_pub,
               uint8_t *rxbuf, uint16_t rxcap);

/* 构造并发送 CONNECT；will_topic/will_msg 非空时启用 LWT（遗嘱消息） */
uint8_t MQTT_Connect(MQTT_Client_t *c, const char *client_id,
                     const char *username, const char *password,
                     uint16_t keepalive,
                     const char *will_topic, const char *will_msg);

/* 发送 PUBLISH（QoS0） */
uint8_t MQTT_Publish(MQTT_Client_t *c, const char *topic,
                     const char *payload, uint8_t retain);

/* 发送 PUBLISH（QoS1，至少一次）：记录报文并等待 PUBACK
 * 返回 1=已发出；0=已有在途报文未确认（停等） */
uint8_t MQTT_Publish_QoS1(MQTT_Client_t *c, const char *topic,
                          const char *payload, uint8_t retain, uint32_t now);

/* 订阅主题（QoS1） */
uint8_t MQTT_Subscribe(MQTT_Client_t *c, const char *topic, uint8_t qos);

/* 回复下行 QoS1 的 PUBACK（在任务上下文调用） */
void MQTT_SendPuback(MQTT_Client_t *c, uint16_t packet_id);

/* 周期驱动（任务里调用）：QoS1 超时重发
 * 返回 0=无动作 1=已重发 2=重发次数耗尽，调用方应判定链路异常并重连 */
uint8_t MQTT_Poll(MQTT_Client_t *c, uint32_t now);

/* 发送 PINGREQ 心跳 */
void MQTT_Ping(MQTT_Client_t *c);

/* 断线重连前清空会话状态（保留 send/on_publish/rxbuf 配置） */
void MQTT_ResetSession(MQTT_Client_t *c);

/* 喂入一个接收字节，状态机自动解析并分发 */
void MQTT_FeedByte(MQTT_Client_t *c, uint8_t b);

#endif
