#ifndef NETSWITCH_H
#define NETSWITCH_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "EncodingStateMachine.h"
#include <cmsis_os2.h>
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// rc9net中的交换机类，负责局域网中的通讯，含有mac地址的映射表

#define MAX_NODES 64
#define LOCAL_RCIP 1

typedef struct rcn_msg_
{
    uint8_t rcnIP = 0;          // 目标 IP
    uint8_t rcnMAC = 0;         // 目标 MAC
    uint8_t rcnID = 0;          // 消息 ID
    const void *data = nullptr; // 数据指针
};

class rcnode // rcn网络节点类，每一个想要使用rcn网络的模块或者设备都要继承该类
{
public:
    static rcnode *MAC_2_NODE[MAX_NODES]; // MAC映射表
    static uint8_t local_ip;
    uint8_t rcmac = 0;
    bool if_registed = false;
    bool net_port = false;

    rcn_msg_ rcn_msg;

   
    osMessageQueueId_t normalQueue;

    osMessageQueueId_t overwriteQueue; // 覆盖式队列。长度强制为1

public:
    // pp(point-to-point)协议的API
    uint8_t ppsend_Syn(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, const void *data);
    uint8_t ppget(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, void *output); // pp同步通讯，运行ppget可以主动获取其他模块的某些东西，运行send可以立刻将任意数据传输到任意模块

    uint8_t ppsend_Asyn(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, const void *data);
    uint8_t ppget_Asyn(); // pp异步通讯，依托于freertos队列，需要调用ppget_Asyn去接收，当然这个收发都是非阻塞的

    uint8_t ppsend_AsynOverwrite(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, const void *data);
    uint8_t ppget_AsynOverwrite(); // pp异步通讯覆写版，采用单队列，只接受最新数据

    // pp协议主要是用来寻址，数据的接收解析函数需要用户提供
    bool rcninit(uint8_t RCMAC_, uint8_t normalQueueLength = 5);
    virtual uint8_t msgin(uint8_t rcnID_, const void *data) = 0; // 外面的传进来
    virtual uint8_t msgout(uint8_t rcnID_, void *output) = 0;    // 外面的拿里面的
};

#endif
#endif