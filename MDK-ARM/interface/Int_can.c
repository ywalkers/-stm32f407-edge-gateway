#include "Int_can.h"
#include "string.h"
#include "stdio.h"

void Int_can_init(void)
{
    //配置过滤器
    CAN_FilterTypeDef filterconfig = {0};
    filterconfig.FilterBank = 0;  //过滤器编号
    filterconfig.FilterMode = CAN_FILTERMODE_IDMASK;  //选择使用掩码模式（模糊匹配）
    filterconfig.FilterScale = CAN_FILTERSCALE_32BIT;  //选择使用位数  32位
    //填写寄存器 只要id为1的数据
    filterconfig.FilterIdHigh = 0x0000;//stid[10:3] 8 位  stid[2:0] 3位
    filterconfig.FilterIdLow = 0x0000;
    //只要掩码都是0，表示不需要匹配上，哪一位是1表示哪一位需要匹配
    filterconfig.FilterMaskIdHigh = 0x0000;
    filterconfig.FilterMaskIdLow = 0x0000;
    //使用哪一个接收队列
    filterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    filterconfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan1, &filterconfig) != HAL_OK) {
        printf("--- CAN Filter Config FAILED! ---\r\n");
    }

    // 探针 2：检查 CAN 硬件是否成功启动
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        printf("--- CAN Start FAILED! ---\r\n");
    } else {
        printf("--- CAN Start SUCCESS! ---\r\n");
    }
}


void Int_can_send(uint16_t id, uint8_t *data, uint8_t len)
{
    uint32_t timeout=0;
    //等待发送邮箱空闲
    //函数返回空闲邮箱个数
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) ==0 ){
        timeout++;
        if(timeout>0x3FFFF)
        {
            //超时退出，放弃发送
            printf("CAN TX Mailbox FULL!Drop msg.\r\n");
            return;
        }
    }
    //将需要发送的数据添加到发送邮箱
    CAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.StdId = id; //标准消息ID
    TxHeader.IDE = CAN_ID_STD; //标准格式
    TxHeader.RTR = CAN_RTR_DATA; //数据帧
    TxHeader.DLC = len; //数据长度
    uint32_t mailbox = 0;
    //句柄  can消息头消息 数据 邮箱编号
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox);
}


/**
 * @brief 网关下发控制指令
 * @param node_id 目标节点ID
 * @param cmd     指令码 (0x02修改周期，0x03控制LED)
 * @param param   指令参数
 */
void Gateway_Send_Cmd(uint16_t node_id, uint8_t cmd, uint16_t param)
{
    uint8_t tx_data[8] = {0};
    tx_data[0] = cmd; // 第0字节：指令码

    if (cmd == 0x02) {
        // 组装周期数据：高8位和低8位
        tx_data[1] = (param >> 8) & 0xFF;
        tx_data[2] = param & 0xFF;
        Int_can_send(node_id, tx_data, 3); // 发送3字节
    }
    else if (cmd == 0x03) {
        // 组装开关数据
        tx_data[1] = param & 0xFF;
        Int_can_send(node_id, tx_data, 2); // 发送2字节
    }
}
