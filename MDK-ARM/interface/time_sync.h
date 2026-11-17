#ifndef __TIME_SYNC_H__
#define __TIME_SYNC_H__

#include <stdint.h>

/* ============================================================================
 *  系统绝对时间（Phase 3：SNTP 对时）
 *
 *  设计：
 *    1. 只在连上 WiFi 后向 ESP8266 要一次网络时间（AT+CIPSNTPTIME?），
 *       记下 {unix_base, tick_base} 两个标量。
 *    2. 之后 Time_Sync_Now() = unix_base + (HAL_GetTick() - tick_base)/1000，
 *       靠本地 1ms 时基续走，不需要每帧都去问模块。
 *    3. 时间戳统一用 UTC 的 Unix 秒（不带时区），本地时间由云端/前端换算。
 *
 *  为什么必须"采集时刻"打时间戳：
 *    断网期间的数据会先落 W25Q16 缓存，恢复后补发。如果在发送时才取当前
 *    时间，10 分钟前的数据和刚采的数据会变成同一个时刻，时序就全丢了。
 *    所以时间戳在数据进入队列/缓存的那一刻就固定下来。
 * ========================================================================== */

/* 对时成功：unix_sec 为 UTC 的 Unix 秒 */
void Time_Sync_Set(uint32_t unix_sec);

/* 是否已经对过时（1 = 时间是可信的） */
uint8_t Time_Sync_Valid(void);

/* 当前 Unix 秒；未对时返回 0（调用方据此决定是否省略 ts 字段） */
uint32_t Time_Sync_Now(void);

#endif
