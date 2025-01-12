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

uint8_t rcncore::findOrCreateTopic(const char *topicName)
{
    for (uint8_t i = 0; i < topicCount; ++i)
    {
        if (mystrcmp(topicTable[i].topicName, topicName) == 0)
        {
            return topicTable[i].topicID; // 话题已存在，返回对应的 ID
        }
    }

    if (topicCount >= MAX_TOPICS)
        return 0xFF; // 超过最大话题数量

    topicTable[topicCount].topicName = topicName; // 新建话题
    topicTable[topicCount].topicID = topicCount;
    return topicCount++;
}

uint8_t rcncore::registerPublisher(const char *topicName, rcnode *node)
{
    uint8_t topicID = findOrCreateTopic(topicName);
    if (topicID == 0xFF)
        return 0xFF;

    topicTable[topicID].publisherNode = node;
    return topicID;
}

bool rcncore::registerSubscriber(const char *topicName, rcnode *node)
{
    uint8_t topicID = findOrCreateTopic(topicName);
    if (topicID == 0xFF)
        return false;

    topicinfo_ &topic = topicTable[topicID];
    if (topic.subscriberCount >= MAX_SUBSCRIBERS_PER_TOPIC)
        return false;

    topic.subscribers[topic.subscriberCount++] = node;
    return true;
}

void rcncore::publish(uint8_t topicID, uint8_t dataID, const void *data, pptype_ pptype)
{
    if (topicID >= topicCount)
        return; // 无效话题 ID

    topicinfo_ &topic = topicTable[topicID];
    rcnode *publisher = topic.publisherNode;
    if (publisher == nullptr)
        return; // 没有发布者

    for (uint8_t i = 0; i < topic.subscriberCount; ++i)
    {
        rcnode *subscriber = topic.subscribers[i];
        switch (pptype)
        {
        case SYN:
            publisher->ppsend_Syn(topicID, subscriber->rcmac, dataID, data);
            break;
        case ASYN:
            publisher->ppsend_Asyn(topicID, subscriber->rcmac, dataID, data);
            break;
        case ASYNOVERWRITE:
            publisher->ppsend_AsynOverwrite(topicID, subscriber->rcmac, dataID, data);
            break;
        }
    }
}

// 发布者实现

void publisher::init(const char *topicName_, pptype_ pptypeselect, rcnode *node_)
{
    topicName = topicName_;
    pptype = pptypeselect;
    node = node_;
    topicID = rcncore::registerPublisher(topicName, node);
}

uint8_t publisher::publish(uint8_t dataID, const void *data)
{
    if (topicID == 0xFF)
        return 0;

    rcncore::publish(topicID, dataID, data, pptype);
    return 1;
}


void subscriber::init(const char *topicName_, pptype_ pptypeselect, rcnode *node_)
{
    topicName = topicName_;
    pptype = pptypeselect;
    node = node_;
    topicID = rcncore::findOrCreateTopic(topicName);
    rcncore::registerSubscriber(topicName, node);
}
uint8_t subscriber::hearfromtopic()
{
    if (topicID == 0xFF)
        return 0;

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
