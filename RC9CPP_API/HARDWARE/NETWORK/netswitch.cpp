#include "netswitch.h"

rcnode *rcnode::MAC_2_NODE[MAX_NODES] = {nullptr};

uint8_t rcnode::local_ip = LOCAL_RCIP;

uint8_t
rcnode::rcn_send(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, const void *data)
{
    if (rcnIP_ == local_ip && MAC_2_NODE[rcnMAC_] != nullptr)
    {
        return MAC_2_NODE[rcnMAC_]->update(rcnID_, data);
    }
    else
    {
    }
}

uint8_t rcnode::rcn_get(uint8_t rcnIP_, uint8_t rcnMAC_, uint8_t rcnID_, void *output)
{
    if (rcnIP_ == local_ip && MAC_2_NODE[rcnMAC_] != nullptr)
    {
        return MAC_2_NODE[rcnMAC_]->output(rcnID_, output);
    }
    else
    {
    }
}



bool rcnode::rcninit(uint8_t RCMAC_, bool if_port_)
{
    if (MAC_2_NODE[RCMAC_] == nullptr)
    {
        MAC_2_NODE[RCMAC_] = this;
        if_registed = true;
        rcmac = RCMAC_;
        return true;
    }
    else
    {
        if_registed = false;
        return false;
    }
}