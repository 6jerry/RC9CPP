#ifndef SERIAL_STUDIO_DEBUG_H
#define SERIAL_STUDIO_DEBUG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "RC9Protocol.h"
#ifdef __cplusplus

    /**
     *  serial studio
     *  Author： yanqy
     *  Todo ： 基于DataReceivedCallback实现一套接收数据并处理（可能会做）
     */
}
#endif
#ifdef __cplusplus

class serial_studio : public RC9subscriber
{
public:

    void send_float_debuginfo(uint8_t id, float *data, uint8_t length);
    void add_IO(RC9Protocol *port_);

    float temp_param[6] = {0.0f}; // 暂存上位机传过来的参数
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;

private:
};

#endif
#endif