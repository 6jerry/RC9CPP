#include "netswitch.h"

rcnode *rcnode::MAC_2_NODE[MAX_NODES] = {nullptr};

uint8_t rcnode::local_ip = LOCAL_RCIP;

/**
 * @brief 同步发送数据到指定的节点
 * @param rcnIP_ 目标节点的 IP 地址
 * @param rcnMAC_ 目标节点的 MAC 地址
 * @param rcnID_ 数据的 ID
 * @param data 数据指针
 * @return 发送结果，成功返回1，失败返回0
 */
uint8_t
rcnode::ppsend_Syn(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, const void *data)
{
    if (rcnIP_ == local_ip && MAC_2_NODE[rcnMAC_] != nullptr && rcnMAC_ < MAX_NODES)
    {
        // 调用目标节点的 msgin 函数发送数据，并返回结果
        return MAC_2_NODE[rcnMAC_]->msgin(rcnID_, data);
    }
    else
    {
    }
}

uint8_t rcnode::ppget(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, void *output)
{
    if (rcnIP_ == local_ip && MAC_2_NODE[rcnMAC_] != nullptr && rcnMAC_ < MAX_NODES)
    {
        return MAC_2_NODE[rcnMAC_]->msgout(rcnID_, output);
    }
    else
    {
    }
}

bool rcnode::rcninit(uint8_t RCMAC_, uint8_t normalQueueLength)
{
    if (RCMAC_ >= MAX_NODES || MAC_2_NODE[RCMAC_] != nullptr)
    {
        if_registed = false;
        return false; // 检查 MAC 地址是否有效
    }

    overwriteQueue = osMessageQueueNew(1, sizeof(rcn_msg_), nullptr);
    if (overwriteQueue == nullptr)
    {
        return false;
    }

    normalQueue = osMessageQueueNew(normalQueueLength, sizeof(rcn_msg_), nullptr);
    if (normalQueue == nullptr)
    {
        osMessageQueueDelete(normalQueue);
        return false;
    }

    MAC_2_NODE[RCMAC_] = this;
    if_registed = true;
    rcmac = RCMAC_;
    return true;
}
uint8_t rcnode::ppsend_Asyn(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, const void *data)
{
    if (rcnIP_ == local_ip && MAC_2_NODE[rcnMAC_] != nullptr && rcnMAC_ < MAX_NODES)
    {
        rcn_msg_ message = {rcnIP_, rcnMAC_, rcnID_, data};
        if (osMessageQueuePut(MAC_2_NODE[rcnMAC_]->normalQueue, &message, 0, 0) == osOK)
        {
            return 1; // 发送成功
        }

        return 0;
    }
    else
    {
        return 0;
    }
}
uint8_t rcnode::ppget_Asyn()
{

    if (osMessageQueueGet(normalQueue, &rcn_msg, nullptr, 0) == osOK)
    {

        return msgin(rcn_msg.rcnID, rcn_msg.data);
    }
    return 0;
}
uint8_t rcnode::ppsend_AsynOverwrite(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, const void *data)
{
    if (rcnIP_ == local_ip && MAC_2_NODE[rcnMAC_] != nullptr && rcnMAC_ < MAX_NODES)
    {
        rcnode *targetNode = MAC_2_NODE[rcnMAC_];
        rcn_msg_ message = {rcnIP_, rcnMAC_, rcnID_, data};

        // 尝试放入队列，如果队列满则覆盖
        if (osMessageQueuePut(targetNode->overwriteQueue, &message, 0, 0) == osOK)
        {
            return 1; // 发送成功
        }
        else
        {
            // 队列已满，移除旧消息并重试
            osMessageQueueGet(targetNode->overwriteQueue, nullptr, nullptr, 0); // 移除旧消息
            if (osMessageQueuePut(targetNode->overwriteQueue, &message, 0, 0) == osOK)
            {
                return 1; // 覆盖成功
            }
        }
    }
    return 0; // 发送失败
}
uint8_t rcnode::ppget_AsynOverwrite()
{
    rcn_msg_ message;

    // 从覆盖队列中尝试获取消息（非阻塞）
    if (osMessageQueueGet(overwriteQueue, &message, nullptr, 0) == osOK)
    {
        return msgin(message.rcnID, message.data); // 调用用户实现的 `msgin` 方法处理消息
    }

    return 0; // 队列中无消息
}
