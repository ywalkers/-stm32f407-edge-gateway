/* ============================================================================
 *  系统绝对时间（Phase 3：SNTP 对时）
 *  实现见 time_sync.h 的说明。
 * ========================================================================== */

#include "time_sync.h"
#include "main.h"      /* HAL_GetTick */

static volatile uint32_t s_base_unix = 0U;   /* 对时那一刻的 Unix 秒 */
static volatile uint32_t s_base_tick = 0U;   /* 对时那一刻的 HAL 毫秒计数 */
static volatile uint8_t  s_valid     = 0U;   /* 1 = 已对时 */

void Time_Sync_Set(uint32_t unix_sec)
{
    /* 先取 tick 再存 unix：两条语句之间有极小的计时误差，
     * 先取 tick 会让误差偏向"时间略快"，可以接受（不影响数据时序） */
    s_base_tick = HAL_GetTick();
    s_base_unix = unix_sec;
    s_valid     = 1U;
}

uint8_t Time_Sync_Valid(void)
{
    return s_valid;
}

uint32_t Time_Sync_Now(void)
{
    if (s_valid == 0U) return 0U;

    /* HAL_GetTick 是 32 位毫秒，减法天然处理回绕；
     * 除 1000 丢掉毫秒——上行数据的精度到秒就够，JSON 也更短 */
    return s_base_unix + ((HAL_GetTick() - s_base_tick) / 1000U);
}
