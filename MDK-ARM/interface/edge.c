#include "edge.h"
#include <string.h>

/* ============================================================
 *  边缘计算：滑动平均滤波 + 死区过滤 + 阈值告警 + 分钟级统计
 *
 *  目的：把「原始采集」和「上行上报」解耦。
 *    - 滤波：抑制传感器抖动，上报的是平滑值
 *    - 死区：变化小于阈值不上报，显著降低上行流量
 *    - 告警：阈值越界立即上报，带滞回避免在阈值附近反复触发
 *    - 统计：每分钟输出 min/max/avg，云端不用自己做聚合
 * ============================================================ */

#define EDGE_WIN          8       /* 滑动平均窗口（8 次采样） */
#define EDGE_DEAD_T       0.2f    /* 温度死区：变化 < 0.2°C 不上报 */
#define EDGE_DEAD_H       1.0f    /* 湿度死区：变化 < 1.0% 不上报 */
#define EDGE_T_HIGH       30.0f   /* 高温告警阈值 */
#define EDGE_T_LOW        5.0f    /* 低温告警阈值 */
#define EDGE_HYST         1.0f    /* 告警滞回带，防抖 */
#define EDGE_STATS_N      30      /* 每 30 次采样出一份统计（2s x 30 = 60s） */

static float   t_buf[EDGE_WIN];
static float   h_buf[EDGE_WIN];
static uint8_t win_idx;
static uint8_t win_cnt;

static float   last_rep_t, last_rep_h;   /* 上次上报的滤波值 */
static uint8_t have_last_rep;

static uint8_t alarm_on;                 /* 0=正常 1=告警中 */
static uint8_t alarm_type;               /* 1=高温 2=低温 */

/* 分钟统计 */
static float    stat_t_min, stat_t_max, stat_t_sum;
static float    stat_h_min, stat_h_max, stat_h_sum;
static uint32_t stat_cnt;

/* 不引入 math.h，自己算绝对值，避免链接浮点库 */
static float fabs_f(float x)
{
    return (x < 0.0f) ? -x : x;
}

void Edge_Init(void)
{
    memset(t_buf, 0, sizeof(t_buf));
    memset(h_buf, 0, sizeof(h_buf));
    win_idx = 0;
    win_cnt = 0;
    last_rep_t = last_rep_h = 0.0f;
    have_last_rep = 0;
    alarm_on = 0;
    alarm_type = 0;
    stat_t_min = stat_t_max = stat_t_sum = 0.0f;
    stat_h_min = stat_h_max = stat_h_sum = 0.0f;
    stat_cnt = 0;
}

static void win_push(float t, float h)
{
    t_buf[win_idx] = t;
    h_buf[win_idx] = h;
    win_idx = (uint8_t)((win_idx + 1) % EDGE_WIN);
    if (win_cnt < EDGE_WIN) win_cnt++;
}

static float win_avg(const float *b)
{
    float s = 0.0f;
    for (uint8_t i = 0; i < win_cnt; i++) s += b[i];
    return (win_cnt > 0) ? (s / (float)win_cnt) : 0.0f;
}

edge_report_t Edge_Process(float raw_t, float raw_h,
                           float *out_t, float *out_h,
                           uint8_t *atype, float *alarm_val)
{
    float t, h;

    /* 1. 滑动平均 */
    win_push(raw_t, raw_h);
    t = win_avg(t_buf);
    h = win_avg(h_buf);

    *out_t    = t;
    *out_h    = h;
    *atype    = 0;
    *alarm_val = t;

    /* 2. 累积分钟统计（用原始值，统计的是真实数据） */
    if (stat_cnt == 0) {
        stat_t_min = stat_t_max = raw_t;
        stat_h_min = stat_h_max = raw_h;
    } else {
        if (raw_t < stat_t_min) stat_t_min = raw_t;
        if (raw_t > stat_t_max) stat_t_max = raw_t;
        if (raw_h < stat_h_min) stat_h_min = raw_h;
        if (raw_h > stat_h_max) stat_h_max = raw_h;
    }
    stat_t_sum += raw_t;
    stat_h_sum += raw_h;
    stat_cnt++;

    /* 3. 阈值告警（带滞回）：只在「进入」和「退出」告警时各报一次 */
    if (!alarm_on) {
        if (t > EDGE_T_HIGH)      { alarm_on = 1; alarm_type = 1; }
        else if (t < EDGE_T_LOW)  { alarm_on = 1; alarm_type = 2; }
        if (alarm_on) {
            *atype     = alarm_type;
            *alarm_val = t;
            have_last_rep = 1;
            last_rep_t = t;
            last_rep_h = h;
            return EDGE_REPORT_ALARM;
        }
    } else {
        uint8_t recover = 0;

        if (alarm_type == 1 && t < EDGE_T_HIGH - EDGE_HYST) recover = 1;
        if (alarm_type == 2 && t > EDGE_T_LOW + EDGE_HYST)  recover = 1;

        if (recover) {
            alarm_on   = 0;
            *atype     = 3;              /* 3 = 恢复 */
            *alarm_val = t;
            have_last_rep = 1;
            last_rep_t = t;
            last_rep_h = h;
            return EDGE_REPORT_ALARM;
        }

        /* 告警持续期间：按死区正常上报数据，但不再重复发告警 */
    }

    /* 4. 死区过滤：与上次上报值比较 */
    if (!have_last_rep ||
        fabs_f(t - last_rep_t) >= EDGE_DEAD_T ||
        fabs_f(h - last_rep_h) >= EDGE_DEAD_H) {
        have_last_rep = 1;
        last_rep_t = t;
        last_rep_h = h;
        return EDGE_REPORT_DATA;
    }

    return EDGE_REPORT_NONE;
}

uint8_t Edge_Stats(float *avg_t, float *min_t, float *max_t,
                   float *avg_h, float *min_h, float *max_h,
                   uint32_t *count)
{
    if (stat_cnt < EDGE_STATS_N) return 0;

    *avg_t = stat_t_sum / (float)stat_cnt;
    *min_t = stat_t_min;
    *max_t = stat_t_max;
    *avg_h = stat_h_sum / (float)stat_cnt;
    *min_h = stat_h_min;
    *max_h = stat_h_max;
    *count = stat_cnt;

    /* 清空，开始下一个周期 */
    stat_cnt   = 0;
    stat_t_sum = 0.0f;
    stat_h_sum = 0.0f;

    return 1;
}
