#ifndef __OTA_H__
#define __OTA_H__

#include <stdint.h>

/* ============================================================================
 *  OTA 升级：HTTP 下载固件到 W25Q16 暂存区
 *
 *  为什么先存外部 Flash、而不是直接写内部 App B：
 *    1. 网络随时可能断。直接写 B 区等于先把可用固件擦掉，
 *       断一次就没有可回滚的镜像了；暂存区写坏了不影响正在跑的固件。
 *    2. 内部 Flash 的 App B 扇区是 128KB 一颗，擦一颗 CPU 会停摆 1~2s，
 *       期间代码执行、串口接收全断；W25Q16 是 4KB/64KB 擦除，快得多。
 *  下载完校验 CRC 通过后，再由 ota_apply() 搬进内部 App B（下一步做）。
 *
 *  链路：MQTT 只发指令，固件本体走 HTTP GET —— 控制面与数据面分离。
 *  这次传输必须独占 ESP8266（CIPMUX=0），所以下载期间 MQTT 连接是断开的。
 * ========================================================================== */

/* 暂存区布局（W25Q16，2MB） */
#define OTA_STAGE_ADDR      0x00001000UL   /* 固件暂存起始，前 4KB 留给 OTA 头 */
#define OTA_MAX_FW_SIZE     0x00070000UL   /* 448KB，不超过 App B 的 512KB */

/* 接收超时：连续这么久没有新数据就判失败 */
#define OTA_RECV_TIMEOUT_MS 30000UL

/* ---- 调试用自动测试（验证完改成 0） ----
 * 现在改成 0：改由 MQTT 指令 gw/node1/ota/cmd 触发，
 * 上电不再自己下载，也不会出现"每次复位都升一次级"的噪声。 */
#define OTA_AUTO_TEST       0
#define OTA_TEST_HOST       "192.168.137.1"  /* PC 移动热点网卡的固定地址 */
#define OTA_TEST_PORT       8000
#define OTA_TEST_PATH       "/f407.bin"
#define OTA_TEST_SIZE       0UL            /* 0 = 用响应头里的 Content-Length */

typedef struct
{
    uint32_t size;                      /* 实际下载到的字节数 */
    uint32_t crc;                       /* 整包 CRC32，和 PC 上算的对一下就知对不对 */
    char     http_status[40];           /* 服务端状态行，比如 "HTTP/1.0 200 OK" */
} ota_result_t;

/* HTTP GET 下载固件到 W25Q16 暂存区。
 * 调用前提：ESP8266 已在 AT 命令模式（非透传），且已连上 WiFi。
 * 返回：0 成功
 *      -1 参数非法      -2 TCP 连接失败     -3 CIPSEND 失败
 *      -4 Flash 擦除失败 -5 收到的字节数不足 */
int ota_http_download(const char *host, uint16_t port, const char *path,
                      uint32_t expect_size, ota_result_t *res);

/* 把 W25Q16 暂存区的固件搬进内部 App B，并在参数区标记"待验证"。
 *
 * 流程：回读暂存区重算 CRC -> 只擦固件真正占用的扇区 -> 按字编程
 *       -> 内部 Flash 回验 CRC -> 写参数区（upgrade_flag=PENDING, target_slot=B）。
 * 成功返回 0（此时还没复位，由调用方决定什么时候重启）；
 * 任何一步失败都返回负值且参数区保持不动 —— 参数区不动就等于"没升级"，
 * 下次启动还是老老实实跑 App A，这是不变砖的关键。
 *
 * 返回：-1 参数非法      -2 暂存区 CRC 不符   -3 擦除失败
 *       -4 编程失败      -5 内部 Flash 回验失败 -6 参数区写失败 */
int ota_apply(uint32_t size, uint32_t crc);

/* 远程触发入口：按 URL 执行一次完整升级
 *
 *   url 形如 "http://192.168.137.1:8000/f407_B.bin"（端口缺省 80）
 *
 * ★ URL 指向的镜像必须匹配"目标槽"的链接地址 ★
 *   跑在 A（active_slot=0）时目标槽是 B，要下发 f407_B.bin（链接 0x08080000）；
 *   跑在 B（active_slot=1）时目标槽是 A，要下发 f407_A.bin（链接 0x0800C000）。
 *   Keil 工程里 f407 / f407_B 两个 target 分别产出这两份 bin。
 *   发错版本不会立即炸，但重启后跳过去执行的是别人的代码，症状是静默死机。
 *
 * 内部流程：复位 ESP8266（顺带退出透传、断开 MQTT 那条 TCP，
 * 因为模块是 CIPMUX=0 单连接，下载必须独占）-> 连 WiFi
 * -> HTTP 下载到 W25Q16 -> 校验 -> 刷"非活动槽" -> NVIC_SystemReset。
 *
 * 成功不返回（直接重启）；失败返回负值，调用方可以重连 MQTT 上报错误。 */
int ota_run_url(const char *url);

/* 上电自检用：自己复位 ESP -> 连 WiFi -> 下载一次并打印结果 */
void ota_auto_test(void);

#endif /* __OTA_H__ */
