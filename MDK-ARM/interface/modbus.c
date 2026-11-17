#include "modbus.h"

volatile int16_t mb_hold_reg[MB_REG_NUM] = { 0, 0 };

uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

uint16_t Modbus_Process(const uint8_t *req, uint16_t req_len, uint8_t *resp)
{
    /* 从站只响应发给自己的帧（此处不含广播处理） */
    if (req_len < 8) return 0;
    if (req[0] != MODBUS_SLAVE_ADDR) return 0;

    /* CRC 校验：帧尾 2 字节，低字节在前 */
    uint16_t crc_rx = ((uint16_t)req[req_len - 1] << 8) | req[req_len - 2];
    if (Modbus_CRC16(req, req_len - 2) != crc_rx) return 0;

    uint8_t func = req[1];

    switch (func) {
    case 0x03: {   /* 读保持寄存器 */
        uint16_t start = ((uint16_t)req[2] << 8) | req[3];
        uint16_t num   = ((uint16_t)req[4] << 8) | req[5];
        if (num == 0 || num > 2 || (start + num) > MB_REG_NUM) return 0;

        resp[0] = MODBUS_SLAVE_ADDR;
        resp[1] = func;
        resp[2] = (uint8_t)(num * 2);            /* 字节数 = 寄存器数 * 2 */
        for (uint16_t i = 0; i < num; i++) {
            int16_t v = mb_hold_reg[start + i];
            resp[3 + i * 2] = (uint8_t)(v >> 8);     /* 高字节在前 */
            resp[4 + i * 2] = (uint8_t)(v & 0xFF);
        }

        uint16_t pos = 3 + num * 2;
        uint16_t crc = Modbus_CRC16(resp, pos);
        resp[pos]     = (uint8_t)(crc & 0xFF);       /* CRC 低字节在前 */
        resp[pos + 1] = (uint8_t)(crc >> 8);
        return pos + 2;
    }
    default:
        /* 不支持的功能码：从站静默（可扩展返回异常帧 0x80|func） */
        return 0;
    }
}
