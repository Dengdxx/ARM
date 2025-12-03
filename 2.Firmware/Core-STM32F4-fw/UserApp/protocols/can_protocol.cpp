#include "common_inc.h"

/**
 * @file can_protocol.cpp
 * @brief CAN 通信协议实现。
 *
 * 此文件包含处理接收到的 CAN 消息的回调函数，包括电机位置更新和联合状态更新。
 */

// Used for response CAN message.
static CAN_TxHeaderTypeDef txHeader =
    {
        .StdId = 0,
        .ExtId = 0,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    };

extern DummyRobot dummy;

/**
 * @brief 处理接收到的 CAN 消息。
 *
 * 此函数由 CAN 中断处理程序调用，用于解析和处理传入的 CAN 帧。
 * 它根据 CAN ID 和命令代码识别消息类型，并更新相应的电机或机器人状态。
 *
 * @param canCtx CAN 上下文指针。
 * @param rxHeader 指向接收到的 CAN 帧头的指针。
 * @param data 指向接收到的数据缓冲区的指针。
 */
void OnCanMessage(CAN_context* canCtx, CAN_RxHeaderTypeDef* rxHeader, uint8_t* data)
{
    // Common CAN message callback, uses ID 32~0x7FF.
    if (canCtx->handle->Instance == CAN1)
    {
        uint8_t id = rxHeader->StdId >> 7; // 4Bits ID & 7Bits Msg
        uint8_t cmd = rxHeader->StdId & 0x7F; // 4Bits ID & 7Bits Msg

        /*----------------------- ↓ Add Your CAN1 Packet Protocol Here ↓ ------------------------*/
        switch (cmd)
        {
            case 0x23:
                dummy.motorJ[id]->UpdateAngleCallback(*(float*) (data), data[4]);
                break;
            default:
                break;
        }

        dummy.UpdateJointAnglesCallback();

    } else if (canCtx->handle->Instance == CAN2)
    {
        /*----------------------- ↓ Add Your CAN2 Packet Protocol Here ↓ ------------------------*/
    }
    /*----------------------- ↑ Add Your Packet Protocol Here ↑ ------------------------*/
}