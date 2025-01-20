#include "rcncore.h"

// 静态变量初始化
topicinfo_ rcncore::topicTable[MAX_TOPICS] = {};
uint8_t rcncore::topicCount = 0;

int mystrcmp(const char *str1, const char *str2)
{
    while (*str1 && (*str1 == *str2))
    {
        str1++;
        str2++;
    }
    return *(unsigned char *)str1 - *(unsigned char *)str2;
}

/**
 * @brief 查找或创建一个主题
 * @param topicName 主题名称
 * @return 主题ID，如果没有找到或创建有效的主题，返回0xFF
 */
uint8_t rcncore::findOrCreateTopic(const char *topicName)
{
    // 遍历已注册的主题列表
    for (uint8_t i = 0; i < topicCount; ++i)
    {
        // 比较当前主题名称与给定的主题名称是否相同
        if (mystrcmp(topicTable[i].topicName, topicName) == 0)
        {
            return topicTable[i].topicID; // 话题已存在，返回对应的 ID
        }
    }

    // 如果主题数量已经达到最大值，则返回0xFF表示无法创建新主题
    if (topicCount >= MAX_TOPICS)
        return 0xFF; // 超过最大话题数量

    topicTable[topicCount].topicName = topicName; // 新建话题
    topicTable[topicCount].topicID = topicCount;
    // 返回新主题的ID，并将主题数量加一
    return topicCount++;
}

/**
 * @brief 注册一个发布者到指定的主题
 * @param topicName 主题名称
 * @param node 发布者节点
 * @return 注册结果，成功返回主题ID，失败返回0xFF
 */
uint8_t rcncore::registerPublisher(const char *topicName, rcnode *node)
{
    // 查找或创建主题，并获取主题ID
    uint8_t topicID = findOrCreateTopic(topicName);
    // 如果主题ID为0xFF，表示没有找到或创建有效的主题，返回0xFF
    if (topicID == 0xFF)
        return 0xFF;

    topicTable[topicID].publisherNode = node;
    return topicID;
}

/**
 * @brief 注册一个订阅者到指定的主题
 * @param topicName 主题名称
 * @param node 订阅者节点
 * @return 注册结果，成功返回 true，失败返回 false
 */
bool rcncore::registerSubscriber(const char *topicName, rcnode *node)
{
    // 查找或创建主题，并获取主题ID
    uint8_t topicID = findOrCreateTopic(topicName);
    // 如果主题ID为0xFF，表示没有找到或创建有效的主题，返回 false
    if (topicID == 0xFF)
        return false;

    // 获取指定主题ID的主题信息
    topicinfo_ &topic = topicTable[topicID];
    // 如果该主题的订阅者数量已经达到最大值，则返回 false
    if (topic.subscriberCount >= MAX_SUBSCRIBERS_PER_TOPIC)
        return false;

    // 将订阅者节点添加到该主题的订阅者列表中
    topic.subscribers[topic.subscriberCount++] = node;
    // 返回注册结果，成功返回 true
    return true;
}

/**
 * @brief 发布数据到指定的主题
 * @param topicID 主题ID
 * @param dataID 数据ID
 * @param data 数据指针
 * @param pptype 发布类型
 */
void rcncore::publish(uint8_t topicID, uint8_t dataID, const void *data, pptype_ pptype)
{
    // 如果主题ID大于等于已注册的主题数量，则返回
    if (topicID >= topicCount)
        return; // 无效话题 ID

    // 获取指定主题ID的主题信息
    topicinfo_ &topic = topicTable[topicID];
    // 获取该主题的发布者节点
    rcnode *publisher = topic.publisherNode;
    // 如果没有发布者，则返回
    if (publisher == nullptr)
        return; // 没有发布者

    // 遍历该主题的所有订阅者
    for (uint8_t i = 0; i < topic.subscriberCount; ++i)
    {
        // 获取订阅者节点
        rcnode *subscriber = topic.subscribers[i];
        // 根据发布类型选择相应的发送方法
        switch (pptype)
        {
        case SYN:
            // 同步发送数据
            publisher->ppsend_Syn(topicID, subscriber->rcmac, dataID, data);
            break;
        case ASYN:
            // 异步发送数据
            publisher->ppsend_Asyn(topicID, subscriber->rcmac, dataID, data);
            break;
        case ASYNOVERWRITE:
            // 异步覆盖发送数据
            publisher->ppsend_AsynOverwrite(topicID, subscriber->rcmac, dataID, data);
            break;
        }
    }
}

// 发布者实现

/**
 * @brief 初始化发布者
 * @param topicName_ 主题名称
 * @param pptypeselect 发布类型
 * @param node_ 节点指针
 */
void publisher::init(const char *topicName_, pptype_ pptypeselect, rcnode *node_)
{
    // 初始化主题名称
    topicName = topicName_;
    // 初始化发布类型
    pptype = pptypeselect;
    // 初始化节点指针
    node = node_;
    // 注册发布者，并获取主题ID
    topicID = rcncore::registerPublisher(topicName, node);
}

/**
 * @brief 发布数据到指定的主题
 * @param dataID 数据ID
 * @param data 数据指针
 * @return 发布结果，成功返回1，失败返回0
 */
uint8_t publisher::publish(uint8_t dataID, const void *data)
{
    // 如果主题ID为0xFF，表示没有找到或创建有效的主题，返回0
    if (topicID == 0xFF)
        return 0;

    // 调用rcncore::publish函数将数据发布到指定的主题
    rcncore::publish(topicID, dataID, data, pptype);

    // 返回发布结果，成功返回1
    return 1;
}

/**
 * @brief 初始化订阅者
 * @param topicName_ 主题名称
 * @param pptypeselect 发布类型
 * @param node_ 节点指针
 */
void subscriber::init(const char *topicName_, pptype_ pptypeselect, rcnode *node_)
{
    // 初始化主题名称
    topicName = topicName_;
    // 初始化发布类型
    pptype = pptypeselect;
    // 初始化节点指针
    node = node_;
    // 查找或创建主题，并获取主题ID
    topicID = rcncore::findOrCreateTopic(topicName);
    // 注册订阅者
    rcncore::registerSubscriber(topicName, node);
}

/**
 * @brief 从主题中接收消息
 * @return 接收到的消息数据ID，如果没有接收到消息则返回0
 */
uint8_t subscriber::hearfromtopic()
{
    // 如果主题ID为0xFF，表示没有找到或创建有效的主题，返回0
    if (topicID == 0xFF)
        return 0;

    // 根据发布类型选择相应的接收方法
    switch (pptype)
    {
    case ASYN:
        return node->ppget_Asyn(); // 异步接收消息
        break;
    case ASYNOVERWRITE:
        return node->ppget_AsynOverwrite(); // 异步接收消息
        break;
    }
}