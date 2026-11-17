/*
 * @Author: ywalkers 2078500453@qq.com
 * @Date: 2026-11-11 20:49:51
 * @LastEditors: ywalkers 2078500453@qq.com
 * @LastEditTime: 2026-11-17 15:36:48
 * @FilePath: \undefinedf:\STM32project\stm32 f407\f407\MDK-ARM\upload\wifi_config.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __WIFI_CONFIG_H__
#define __WIFI_CONFIG_H__

#include <stddef.h>   /* NULL */

/* ============================================================
 *  敏感凭据配置文件
 *
 *  ！！不要把本文件提交到公开仓库 ！！
 *  在仓库根目录的 .gitignore 里加入：
 *      MDK-ARM/upload/wifi_config.h
 *
 *  仓库里只保留 wifi_config.example.h 作为模板。
 * ============================================================ */

/* ---- Wi-Fi ---- */
/* 手机热点（也曾用于 PC 共享出来的热点，SSID 同名）：
 * 板子连热点后走蜂窝网络出网，MQTT 连公共 broker 不受影响。
 *
 * 注意：ESP8266 只支持 2.4GHz。手机热点如果开了 5GHz（部分安卓默认 5G），
 * 模块根本扫不到，AT+CWJAP 会一直失败。 */
#define WIFI_SSID        "YOUR_WIFI_SSID"
#define WIFI_PASS        "YOUR_WIFI_PASSWORD"

/* ---- MQTT 认证 ----
 * 公共 broker.emqx.io 通常允许匿名，这里置 0；
 * 换成自建 EMQX/Mosquitto 后置 1，并填上用户名密码。 */
#define MQTT_AUTH_ENABLE 0

#if MQTT_AUTH_ENABLE
#define MQTT_USER        "f407_gw"
#define MQTT_PASS        "change_me"
#else
#define MQTT_USER        NULL
#define MQTT_PASS        "YOUR_MQTT_PASSWORD"
#endif

#endif
