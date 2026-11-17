#ifndef __EDGE_H__
#define __EDGE_H__

#include <stdint.h>

/* ============================================================
 *  边缘计算模块
 *  输入一次原始采样，输出滤波值 + 上报决策；
 *  另提供分钟级 min/max/avg 统计。
 * ============================================================ */

typedef enum {
    EDGE_REPORT_NONE = 0,   /* 死区内，不上报 */
    EDGE_REPORT_DATA,       /* 正常数据上报 */
    EDGE_REPORT_ALARM       /* 告警或恢复，需要单独上报 */
} edge_report_t;

/* 初始化（上电调用一次） */
void Edge_Init(void);

/* 处理一次采样
 *   raw_t/raw_h : 原始温湿度
 *   out_t/out_h : 滤波后的温湿度（供上报和 Modbus 寄存器）
 *   atype       : 告警类型 0=无 1=高温 2=低温 3=恢复正常
 *   alarm_val   : 告警发生时的温度值
 * 返回值说明是否应该上报、走哪个主题 */
edge_report_t Edge_Process(float raw_t, float raw_h,
                           float *out_t, float *out_h,
                           uint8_t *atype, float *alarm_val);

/* 分钟级统计：采样次数够一个周期时返回 1，并回填统计值 */
uint8_t Edge_Stats(float *avg_t, float *min_t, float *max_t,
                   float *avg_h, float *min_h, float *max_h,
                   uint32_t *count);

#endif
