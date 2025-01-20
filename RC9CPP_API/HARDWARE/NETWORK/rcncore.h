#ifndef RCNCORE_H
#define RCNCORE_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "netswitch.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define MAX_SUBSCRIBERS_PER_TOPIC 5 // 每个话题的最大订阅者数量
#define MAX_TOPICS 10               // 最大话题数量

enum pptype_
{
    SYN,          // 同步发送
    ASYN,         // 异步发送
    ASYNOVERWRITE // 异步覆盖发送
};

class publisher
{
private:
    const char *topicName = nullptr;
    uint8_t topicID = 0;
    pptype_ pptype = SYN; // 默认使用同步发送
    rcnode *node;         // 发布者绑定的 rcnode 节点

public:
    uint8_t publish(uint8_t dataID, const void *data); // 发布消息

    void init(const char *topicName_, pptype_ pptypeselect, rcnode *node_);
};

class subscriber
{
private:
    const char *topicName = nullptr;
    uint8_t topicID = 0;
    rcnode *node;          // 订阅者绑定的 rcnode 节点
    pptype_ pptype = ASYN; // 默认使用同步发送

public:
    uint8_t hearfromtopic(); // 异步获取消息
    void init(const char *topicName_, pptype_ pptypeselect, rcnode *node_);
};

typedef struct topicinfo_
{
    const char *topicName = nullptr;                            // 话题名称
    uint8_t topicID = 0;                                        // 话题 ID
    rcnode *publisherNode = nullptr;                            // 发布者节点
    rcnode *subscribers[MAX_SUBSCRIBERS_PER_TOPIC] = {nullptr}; // 订阅者节点列表
    uint8_t subscriberCount = 0;                                // 当前订阅者数量
};

class rcncore
{
public:
    static uint8_t findOrCreateTopic(const char *topicName); // 查找或创建话题

public:
    static topicinfo_ topicTable[MAX_TOPICS]; // 静态话题表
    static uint8_t topicCount;                // 当前话题数量

    static uint8_t registerPublisher(const char *topicName, rcnode *node);
    static bool registerSubscriber(const char *topicName, rcnode *node);

    static void publish(uint8_t topicID, uint8_t dataID, const void *data, pptype_ pptype); // 发布消息
};

#endif
#endif
