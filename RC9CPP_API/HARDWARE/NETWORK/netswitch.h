#ifndef NETSWITCH_H
#define NETSWITCH_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "EncodingStateMachine.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// rc9net中的交换机类，负责局域网中的通讯，含有mac地址的映射表

#define MAX_NODES 64
#define LOCAL_RCIP 1
class rcnode
{
public:
    static rcnode *MAC_2_NODE[MAX_NODES]; // MAC映射表
    static uint8_t local_ip;
    uint8_t rcmac = 0;
    bool if_registed = false;
    bool net_port = false;

public:


    // GS(Get-Send)协议的API
    uint8_t rcn_send(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, const void *data);
    uint8_t rcn_get(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, void *output);
    // GS协议主要是用来寻址
    bool rcninit(uint8_t RCMAC_, bool if_port_);
    virtual uint8_t update(uint8_t rcnID_, const void *data) = 0; // 外面的传进来
    virtual uint8_t output(uint8_t rcnID_, void *output) = 0;     // 外面的拿里面的
}; // rcn网络节点类，每一个想要使用rcn网络的模块或者设备都要继承该类

#endif
#endif