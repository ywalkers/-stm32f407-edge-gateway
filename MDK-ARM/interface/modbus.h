#ifndef __MODBUS_H__
#define __MODBUS_H__

#include <stdint.h>

/* ============================================================
 *  Modbus RTU 从站（RS485 侧）
 *  网关把 CAN 采集到的温湿度写入保持寄存器，
 *  供 RS485 总线上的主站（上位机/PLC）用功能码 0x03 读取。
 * ============================================================ */

#define MODBUS_SLAVE_ADDR   1        /* 本从站地址 */

/* 保持寄存器地址定义 */
#define MB_REG_TEMP         0        /* 温度，单位 0.1°C（26.0 -> 260） */
#define MB_REG_HUMI         1        /* 湿度，单位 0.1%（66.5 -> 665） */
#define MB_REG_NUM          2

/* 保持寄存器（DataProc 写入，Modbus 从站读取） */
extern volatile int16_t mb_hold_reg[MB_REG_NUM];

/* CRC16-Modbus（多项式 0xA001，初值 0xFFFF） */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len);

/* 处理一帧 Modbus 请求，生成响应；返回响应字节数，0=不响应（地址不符/CRC错/功能码不支持） */
uint16_t Modbus_Process(const uint8_t *req, uint16_t req_len, uint8_t *resp);

#endif
