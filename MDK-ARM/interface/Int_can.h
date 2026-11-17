#ifndef __INT_CAN_H
#define __INT_CAN_H

#include "can.h"
#include "string.h"

/* CAN 接收消息结构体：报文头 + 8 字节数据 */
typedef struct
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8];
} CAN_REC_MSG;

/**
 * @brief 配置 CAN 过滤器并启动 CAN1
 */
void Int_can_init(void);

/**
 * @brief 通过 CAN 发送一帧数据
 * @param id   标准帧 ID
 * @param data 数据指针
 * @param len  数据长度 (<=8)
 */
void Int_can_send(uint16_t id, uint8_t *data, uint8_t len);

/**
 * @brief 网关下发控制指令
 * @param node_id 目标节点 ID
 * @param cmd     指令码 (0x02 修改上报周期, 0x03 控制 LED)
 * @param param   指令参数
 */
void Gateway_Send_Cmd(uint16_t node_id, uint8_t cmd, uint16_t param);

#endif
