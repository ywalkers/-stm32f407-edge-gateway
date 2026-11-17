#ifndef __ESP8266_H__
#define __ESP8266_H__

#include <stdint.h>

/* Wi-Fi 密码、MQTT 认证凭据都放在这个文件里，不进版本库 */
#include "wifi_config.h"

/* ============================================================
 *  ESP8266 AT 指令封装（WiFi 透传，对接 USART3）
 *  职责：连 WiFi -> 建 TCP/TLS(连 MQTT Broker) -> 进入透传
 *        透传后，MCU 发的字节原样走 TCP，broker 回的字节从 USART3 吐出
 * ============================================================ */

#define MQTT_BROKER  "broker.emqx.io"   /* 公共测试 broker */

/* ---- TLS 开关 ----
 * 1 = 走 8883 加密链路（推荐）；0 = 走 1883 明文。
 * ESP8266-01S 可用堆只有 40~50KB，TLS 握手要吃 30KB 左右，
 * 若出现 AT+CIPSTART 失败或模块重启，说明内存不够，
 * 可先把 AT+CIPSSLSIZE 调小，或改用 ESP32。 */
#define ESP8266_USE_TLS  0

#if ESP8266_USE_TLS
#define MQTT_PORT    8883
#else
#define MQTT_PORT    1883
#endif

/* 启动 USART3 单字节中断接收（USART3 初始化完成后调用一次） */
void ESP8266_UART_Start(void);

/* 复位并等待 ready */
uint8_t ESP8266_Reset(void);

/* 设为 Station 模式 */
uint8_t ESP8266_SetMode(uint8_t mode);

/* 连接 WiFi（返回 1 成功） */
uint8_t ESP8266_JoinAP(const char *ssid, const char *pwd);

/* 单连接 + 建立到 broker 的 TCP 连接 */
uint8_t ESP8266_OpenTCP(const char *ip, uint16_t port);

/* 进入透传模式（CIPMODE=1 + CIPSEND，等到 ">"） */
uint8_t ESP8266_EnterTransparent(void);

/* 透传发送：直接写 USART3（作为 MQTT 的 send 回调） */
void ESP8266_Send(const uint8_t *data, uint16_t len);

/* ---- SNTP 对时（Phase 3）----
 * 必须在进入透传之前调用（透传模式下 AT 命令会被当业务数据发走） */

/* 配置 SNTP 服务器（UTC）。返回 1 = 模块接受配置 */
uint8_t ESP8266_SNTP_Start(void);

/* 查询已同步的网络时间，成功返回 1 并写出 UTC 的 Unix 秒；
 * 模块还没同步完（返回 1970 年）时返回 0，调用方应稍后重试 */
uint8_t ESP8266_QueryTime(uint32_t *unix_sec);

/* ---- 以下给 OTA 用（非透传 / AT 命令模式下的原始收发） ---- */

/* 发 AT 命令并等待响应字符串出现，返回 1 成功 */
uint8_t ESP8266_SendCmd(const char *cmd, const char *resp, uint32_t timeout_ms);

/* 取最近一次 AT 响应的原始文本（已补 '\0'），调试用 */
const char *ESP8266_LastResponse(void);

/* 原始数据模式开关：
 *   开 = USART3 中断收到的每个字节压进环形缓冲，不再喂给 MQTT 解析；
 *   关 = 恢复原有行为。
 * OTA 下载时打开，由 ota.c 从环形缓冲里解析 +IPD 帧。
 * 不能在中断里直接写 Flash（擦一个扇区 45ms，串口早溢出了），
 * 所以这里只做搬运，真正的解析和写入在任务上下文完成。 */
void ESP8266_SetRawMode(uint8_t on);

/* 从环形缓冲读原始字节，最多等 timeout_ms（期间来数据会续期）。
 * 返回实际读到的字节数 */
uint32_t ESP8266_ReadRaw(uint8_t *buf, uint32_t len, uint32_t timeout_ms);

#endif
