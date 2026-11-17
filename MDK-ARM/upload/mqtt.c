#include "mqtt.h"
#include "string.h"

/* 全局 MQTT 客户端实例（单例） */
MQTT_Client_t g_mqtt;

static uint8_t g_frame[512];   /* 组帧缓冲 */

/* ==================== 小工具 ==================== */

/* MQTT 剩余长度：变长编码（每字节低 7 位有效，最高位是续标志）
 * 例：321 = 0xC1 0x02。返回编码后占用的字节数。 */
static uint8_t encode_remaining_length(uint8_t *dst, uint32_t len)
{
    uint8_t n = 0;
    do {
        uint8_t d = (uint8_t)(len % 128);
        len /= 128;
        if (len) d |= 0x80;
        dst[n++] = d;
    } while (len);
    return n;
}

/* 写 16 位大端整数，返回 2 */
static uint8_t put_u16(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)(v >> 8);
    dst[1] = (uint8_t)(v & 0xFF);
    return 2;
}

/* 写「2 字节长度前缀 + 字符串」，返回写入总字节数 */
static uint16_t put_str(uint8_t *dst, const char *s)
{
    uint16_t n = (uint16_t)strlen(s);
    dst[0] = (uint8_t)(n >> 8);
    dst[1] = (uint8_t)(n & 0xFF);
    if (n) memcpy(&dst[2], s, n);
    return (uint16_t)(n + 2u);
}

/* ==================== 客户端生命周期 ==================== */

void MQTT_Init(MQTT_Client_t *c, mqtt_send_fn send, mqtt_publish_fn on_pub,
               uint8_t *rxbuf, uint16_t rxcap)
{
    memset(c, 0, sizeof(*c));
    c->send       = send;
    c->on_publish = on_pub;
    c->rxbuf      = rxbuf;
    c->rxcap      = rxcap;
    c->rx_state   = MQTT_RX_FIXED;
    c->rx_mult    = 1;
}

void MQTT_ResetSession(MQTT_Client_t *c)
{
    c->connected            = 0;
    c->pingresp_recv        = 0;
    c->suback_recv          = 0;
    c->puback_pending       = 0;
    c->puback_reply_pending = 0;
    c->rx_state             = MQTT_RX_FIXED;
    c->rx_rem               = 0;
    c->rx_got               = 0;
    c->rx_mult              = 1;
}

/* ==================== 上行报文 ==================== */

/**
 * CONNECT 报文：
 *  固定头 0x10 | 剩余长度
 *  可变头：协议名 "MQTT" + 级别 0x04 + 连接标志 + 保活(2B)
 *  payload：ClientID + [Will Topic + Will Message] + [用户名] + [密码]
 *
 * 连接标志位：
 *   bit7 用户名  bit6 密码  bit5 Will Retain  bit4:3 Will QoS
 *   bit2 Will Flag  bit1 Clean Session  bit0 保留
 */
uint8_t MQTT_Connect(MQTT_Client_t *c, const char *client_id,
                     const char *username, const char *password,
                     uint16_t keepalive,
                     const char *will_topic, const char *will_msg)
{
    uint8_t  vh[384];                       /* 可变头 + payload */
    uint16_t p = 0;
    uint8_t  has_will = (will_topic && will_msg) ? 1 : 0;

    /* ---- 可变头 ---- */
    p += put_str(vh, "MQTT");               /* 协议名，长度前缀 0x0004 */
    vh[p++] = 0x04;                         /* 协议级别 3.1.1 */

    uint8_t flags = 0x02;                   /* bit1: Clean Session */
    if (username) flags |= 0x80;            /* bit7: User Name */
    if (password) flags |= 0x40;            /* bit6: Password */
    if (has_will) {
        flags |= 0x04;                      /* bit2: Will Flag */
        flags |= (uint8_t)(1 << 3);         /* bit4:3: Will QoS = 1 */
        flags |= 0x20;                      /* bit5: Will Retain */
    }
    vh[p++] = flags;

    c->keepalive = keepalive;
    p += put_u16(&vh[p], keepalive);

    /* ---- payload ---- */
    p += put_str(&vh[p], client_id);
    if (has_will) {
        p += put_str(&vh[p], will_topic);
        p += put_str(&vh[p], will_msg);
    }
    if (username) p += put_str(&vh[p], username);
    if (password) p += put_str(&vh[p], password);

    /* ---- 组帧 ---- */
    uint16_t pos = 0;
    g_frame[pos++] = 0x10;
    pos += encode_remaining_length(&g_frame[pos], p);
    memcpy(&g_frame[pos], vh, p);
    pos += p;

    c->connected = 0;
    c->suback_recv = 0;
    c->send(g_frame, pos);
    return 1;
}

/**
 * PUBLISH（QoS0）：固定头 0x30|retain | 剩余长度 | Topic(2B 长度+串) | payload
 */
uint8_t MQTT_Publish(MQTT_Client_t *c, const char *topic,
                     const char *payload, uint8_t retain)
{
    uint16_t tl = (uint16_t)strlen(topic);
    uint16_t pl = (uint16_t)strlen(payload);
    uint16_t rl = (uint16_t)(2u + tl + pl);

    uint16_t pos = 0;
    g_frame[pos++] = (uint8_t)(0x30 | (retain ? 0x01 : 0x00));
    pos += encode_remaining_length(&g_frame[pos], rl);
    pos += put_str(&g_frame[pos], topic);
    if (pl) memcpy(&g_frame[pos], payload, pl);
    pos += pl;

    c->send(g_frame, pos);
    return 1;
}

/**
 * PUBLISH（QoS1，至少一次）：
 * 与 QoS0 的区别是固定头 0x32，且可变头在 topic 之后带 2 字节报文标识符。
 * 发送后进入「停等」：保存报文副本，等 PUBACK；超时由 MQTT_Poll 重发。
 */
uint8_t MQTT_Publish_QoS1(MQTT_Client_t *c, const char *topic,
                          const char *payload, uint8_t retain, uint32_t now)
{
    if (c->puback_pending) return 0;        /* 停等：上一条还没确认 */

    uint16_t tl  = (uint16_t)strlen(topic);
    uint16_t pl  = (uint16_t)strlen(payload);
    uint16_t rl  = (uint16_t)(2u + tl + 2u + pl);
    uint16_t pid = (uint16_t)(c->packet_id + 1);
    if (pid == 0) pid = 1;                  /* 0 不是合法报文标识符 */
    c->packet_id = pid;

    uint16_t pos = 0;
    g_frame[pos++] = (uint8_t)(0x32 | (retain ? 0x01 : 0x00));  /* QoS1 */
    pos += encode_remaining_length(&g_frame[pos], rl);
    pos += put_str(&g_frame[pos], topic);
    pos += put_u16(&g_frame[pos], pid);
    if (pl) memcpy(&g_frame[pos], payload, pl);
    pos += pl;

    if (pos > MQTT_TX_BUF_SIZE) return 0;   /* 备份缓冲放不下（实际不会发生） */

    memcpy(c->txbuf, g_frame, pos);         /* 备份整帧，重发时原样再发 */
    c->txlen          = pos;
    c->puback_id      = pid;
    c->puback_retry   = 0;
    c->puback_tick    = now;
    c->puback_pending = 1;
    c->send(g_frame, pos);
    return 1;
}

/**
 * SUBSCRIBE：
 *  固定头 0x82 | 剩余长度 | 报文标识符(2B) | Topic(2B 长度+串) | 请求 QoS(1B)
 */
uint8_t MQTT_Subscribe(MQTT_Client_t *c, const char *topic, uint8_t qos)
{
    uint16_t tl  = (uint16_t)strlen(topic);
    uint16_t rl  = (uint16_t)(2u + 2u + tl + 1u);
    uint16_t pid = (uint16_t)(c->packet_id + 1);
    if (pid == 0) pid = 1;
    c->packet_id = pid;

    uint16_t pos = 0;
    g_frame[pos++] = 0x82;
    pos += encode_remaining_length(&g_frame[pos], rl);
    pos += put_u16(&g_frame[pos], pid);
    pos += put_str(&g_frame[pos], topic);
    g_frame[pos++] = (uint8_t)(qos & 0x03);

    c->suback_recv = 0;
    c->send(g_frame, pos);
    return 1;
}

/* 回复 PUBACK：固定头 0x40 | 剩余长度 0x02 | 报文标识符 */
void MQTT_SendPuback(MQTT_Client_t *c, uint16_t packet_id)
{
    uint8_t p[4];
    p[0] = 0x40;
    p[1] = 0x02;
    p[2] = (uint8_t)(packet_id >> 8);
    p[3] = (uint8_t)(packet_id & 0xFF);
    c->send(p, 4);
}

void MQTT_Ping(MQTT_Client_t *c)
{
    uint8_t p[2] = { 0xC0, 0x00 };
    c->pingresp_recv = 0;                   /* 发心跳前清标志，开始等 PINGRESP */
    c->send(p, 2);
}

/* ==================== 超时驱动 ==================== */

/**
 * QoS1 停等超时处理：超时则原样重发备份报文；
 * 重发次数耗尽返回 2，由上层判定链路异常并重连。
 */
uint8_t MQTT_Poll(MQTT_Client_t *c, uint32_t now)
{
    if (!c->puback_pending) return 0;
    if ((uint32_t)(now - c->puback_tick) < MQTT_ACK_TIMEOUT_MS) return 0;

    if (c->puback_retry >= MQTT_TX_RETRY_MAX) {
        c->puback_pending = 0;
        return 2;
    }

    c->puback_retry++;
    c->puback_tick = now;
    c->send(c->txbuf, c->txlen);            /* 停等，备份帧直接重发即可 */
    return 1;
}

/* ==================== 下行解析 ==================== */

/* 一帧 payload 收齐后的分发 */
static void mqtt_dispatch(MQTT_Client_t *c)
{
    uint8_t  type = (uint8_t)(c->rx_hdr & 0xF0);
    uint8_t *p    = c->rxbuf;
    uint16_t rem  = (uint16_t)c->rx_rem;

    switch (type)
    {
    case 0x20:                                  /* CONNACK: 20 02 00 rc */
        if (rem >= 2u) c->connected = (p[1] == 0x00) ? 1 : 0;
        break;

    case 0x40:                                  /* PUBACK: 40 02 id_hi id_lo */
        if (rem >= 2u) {
            uint16_t pid = (uint16_t)((p[0] << 8) | p[1]);
            if (pid == c->puback_id) c->puback_pending = 0;
        }
        break;

    case 0x90:                                  /* SUBACK */
        if (rem >= 3u) c->suback_recv = 1;
        break;

    case 0xD0:                                  /* PINGRESP */
        c->pingresp_recv = 1;
        break;

    case 0x30:                                  /* PUBLISH（下行） */
    {
        if (rem < 2u) break;

        uint16_t tl = (uint16_t)((p[0] << 8) | p[1]);
        if ((uint32_t)tl + 2u > rem) break;     /* 畸形报文，丢弃 */

        static char topic[64];                  /* 中断上下文，用静态避免占栈 */
        uint16_t n = (tl < (uint16_t)(sizeof(topic) - 1u))
                   ? tl : (uint16_t)(sizeof(topic) - 1u);
        memcpy(topic, &p[2], n);
        topic[n] = '\0';

        uint8_t  qos = (uint8_t)((c->rx_hdr >> 1) & 0x03);
        uint32_t off = 2u + tl;
        uint16_t pid = 0;

        if (qos > 0) {                          /* QoS1/2 带报文标识符 */
            if (off + 2u > rem) break;
            pid  = (uint16_t)((p[off] << 8) | p[off + 1]);
            off += 2u;
        }
        if (off > rem) break;

        if (c->on_publish)
            c->on_publish(topic, &p[off], (uint16_t)(rem - off), qos);

        /* 中断里不直接写串口（会与任务的发送冲突），只置标志由任务回 */
        if (qos == 1u) {
            c->puback_reply_id      = pid;
            c->puback_reply_pending = 1;
        }
        break;
    }

    default:                                    /* 其余类型不关心 */
        break;
    }
}

/**
 * 逐字节喂入。这里假设 TCP 是字节流：一帧可能分多次到达、
 * 多帧也可能粘在一起，所以必须用状态机自己找边界，不能假设
 * 一次收到完整报文。
 */
void MQTT_FeedByte(MQTT_Client_t *c, uint8_t b)
{
    switch (c->rx_state)
    {
    case MQTT_RX_FIXED:                         /* 固定头：类型 + 标志 */
        c->rx_hdr   = b;
        c->rx_rem   = 0;
        c->rx_mult  = 1;
        c->rx_got   = 0;
        c->rx_state = MQTT_RX_REMLEN;
        break;

    case MQTT_RX_REMLEN:                        /* 变长剩余长度 */
    {
        c->rx_rem += (uint32_t)(b & 0x7F) * c->rx_mult;

        if (b & 0x80) {                         /* 续标志：还有字节 */
            c->rx_mult *= 128;
            if (c->rx_mult > 128u * 128u * 128u) {
                c->rx_state = MQTT_RX_FIXED;    /* 非法编码，复位重来 */
            }
            break;
        }

        if (c->rx_rem > c->rxcap) {             /* 超过缓冲：整包丢弃 */
            c->rx_state = MQTT_RX_FIXED;
        } else if (c->rx_rem == 0) {            /* 无 payload，直接分发 */
            c->rx_state = MQTT_RX_FIXED;
            mqtt_dispatch(c);
        } else {
            c->rx_state = MQTT_RX_PAYLOAD;
        }
        break;
    }

    case MQTT_RX_PAYLOAD:                       /* 累积 payload */
        c->rxbuf[c->rx_got++] = b;
        if (c->rx_got >= c->rx_rem) {
            c->rx_state = MQTT_RX_FIXED;
            mqtt_dispatch(c);
        }
        break;

    default:
        c->rx_state = MQTT_RX_FIXED;
        break;
    }
}
