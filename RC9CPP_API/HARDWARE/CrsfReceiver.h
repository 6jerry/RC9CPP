#ifndef CRSF_RECEIVER_H
#define CRSF_RECEIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "Serial_device.h"         // 包含您的 SerialDevice 基类
#include "crsf_protocol_defines.h" // CRSF 协议定义

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define BTN_OFF 191
#define BTN_ON 1792

#define BTN_1 191
#define BTN_2 997
#define BTN_3 1792

typedef struct
{
    uint8_t device_addr;
    uint8_t frame_size; // size after this byte, so it's type + payload + crc
    uint8_t type;
    crsf_channels_t channels; // 16 channels, 22 bytes
    uint8_t crc;
} PACKED CrsfRcChannelsFrame_t;

class CrsfReceiver : public SerialDevice
{
public:
    CrsfReceiver(UART_HandleTypeDef *huart);

    // 重写基类的处理数据方法
    void handleReceiveData(uint8_t byte) override;

    // 获取单个通道数据的方法 (传入通道编号1-16)
    int getChannel(uint8_t channel_num) const;

    // 获取所有通道数据数组的指针 (如果需要，但更推荐使用 getChannel)
    const int *getAllChannels() const
    {
        return channels_;
    }

private:
    // 存储解析后的通道值
    int channels_[CRSF_NUM_CHANNELS];

    // 存储链路统计信息 (仍然保留，但实际上不会被更新，因为只处理 RC Channels 帧)
    CrsfLinkStatistics_t link_statistics_;

    // 用于接收 CRSF RC Channels 帧的临时结构体
    CrsfRcChannelsFrame_t current_rc_frame_;

    uint8_t packet_byte_index_; // 当前接收字节在 current_rc_frame_ 中的索引
    uint8_t *payload_ptr_;      // 用于指向 channels 部分的成员变量

    // CRSF 协议状态机状态
    enum CrsfRxState
    {
        CRSF_WAITING_FOR_ADDRESS,
        CRSF_WAITING_FOR_FRAMESIZE,
        CRSF_WAITING_FOR_TYPE,
        CRSF_WAITING_FOR_PAYLOAD_CHANNELS, // 专门为 RC Channels Payload 状态
        CRSF_WAITING_FOR_CRC_BYTE,
        CRSF_PACKET_COMPLETE
    } rx_state_;

    // 内部函数：处理接收到的完整 RC Channels 数据包
    void processRcChannelsPacket();

    

    void map_value_compute();
    void flag_set();

public:
    virtual void SAR_ON() {}; // 右侧自锁开关.ch8
    virtual void SAR_OFF() {};
    virtual void SAL_ON() {}; // 左侧自锁开关ch4
    virtual void SAL_OFF() {};

    virtual void BTN_L_CALLBACK() {}; // 左侧按键ch6
    virtual void R_1() {};
    virtual void R_2() {}; // ch7
    virtual void R_3() {};

    virtual void L_1() {}; // 左侧档位ch5
    virtual void L_2() {};
    virtual void L_3() {};

    float left_H_map = 0.0f, left_V_map = 0.0f, right_H_map = 0.0f, right_V_map = 0.0f, roll_map = 0.0f; // 映射值

    uint8_t sar_flag = 0, sal_flag = 0, r_flag = 0, l_flag = 0, btn_r_flag = 0, last_btn_r_flag = 0;
};

#endif // __cplusplus

#endif // CRSF_RECEIVER_H
