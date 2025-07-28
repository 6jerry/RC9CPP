#ifndef CRSF_RECEIVER_H
#define CRSF_RECEIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "Serial_device.h"         // 包含您的 SerialDevice 基类
#include "crsf_protocol_defines.h" // CRSF 协议定义
#include "crc.h"
#include <string.h> // For memcpy
#include "time_counter.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define BTN_OFF 191
#define BTN_ON 1792

#define BTN_1 191
#define BTN_2 1004
#define BTN_3 1792

typedef struct
{
    uint8_t device_addr;
    uint8_t frame_size; // size after this byte, so it's type + payload + crc
    uint8_t type;
    crsf_channels_t channels; // 16 channels, 22 bytes
    uint8_t crc;
} PACKED CrsfRcChannelsFrame_t;

typedef struct
{
    int16_t pitch; // 弧度 * 10000
    int16_t roll;  // 弧度 * 10000
    int16_t yaw;   // 弧度 * 10000
} PACKED CrsfAttitudePayload_t;

typedef struct
{
    uint8_t device_addr; // 0xC8
    uint8_t frame_size;  // type + payload + crc
    uint8_t type;        // 0x1E
    CrsfAttitudePayload_t payload;
    uint8_t crc;
} PACKED CrsfAttitudeFrame_t;

// 定义电池状态数据结构
typedef struct
{
    uint16_t voltage;    // mV * 100
    uint16_t current;    // mA * 100
    uint8_t capacity[3]; // mAh (24位)
    uint8_t remaining;   // %
} PACKED CrsfBatteryPayload_t;

typedef struct
{
    uint8_t device_addr; // 0xC8
    uint8_t frame_size;  // type + payload + crc
    uint8_t type;        // 0x08
    CrsfBatteryPayload_t payload;
    uint8_t crc;
} PACKED CrsfBatteryFrame_t;

class CrsfReceiver : public SerialDevice
{
public:
    CrsfReceiver(UART_HandleTypeDef *huart);

    void handleReceiveData(uint8_t byte) override;
    int getChannel(uint8_t channel_num) const;
    const int *getAllChannels() const { return channels_; }

    // 新增发送遥测数据的方法
    void sendAttitude(float pitch, float roll, float yaw);
    void sendBattery(float voltage, float current, uint32_t capacity, uint8_t remaining);
    void sendGps(double latitude, double longitude, uint16_t groundspeed,
                 uint16_t heading, uint16_t altitude, uint8_t satellites);

private:
    int channels_[CRSF_NUM_CHANNELS];
    CrsfLinkStatistics_t link_statistics_;
    CrsfRcChannelsFrame_t current_rc_frame_;
    uint8_t packet_byte_index_;
    uint8_t *payload_ptr_;
    uint8_t calculated_crc = 0;

    // CRC 对象
    GENERIC_CRC8 crc_; // 使用多项式 0xD5 初始化

    enum CrsfRxState
    {
        CRSF_WAITING_FOR_ADDRESS,
        CRSF_WAITING_FOR_FRAMESIZE,
        CRSF_WAITING_FOR_TYPE,
        CRSF_WAITING_FOR_PAYLOAD_CHANNELS,
        CRSF_WAITING_FOR_CRC_BYTE,
        CRSF_PACKET_COMPLETE
    } rx_state_;

    void processRcChannelsPacket();
    void map_value_compute();
    void flag_set();

public:
    float left_H_map = 0.0f, left_V_map = 0.0f, right_H_map = 0.0f, right_V_map = 0.0f, roll_map = 0.0f;

    float left_H_mapcurve = 0.0f, left_V_mapcurve = 0.0f, right_H_mapcurve = 0.0f, right_V_mapcurve = 0.0f;

    float c = -0.76f;
    uint8_t sar_flag = 0, sal_flag = 0, r_flag = 0, l_flag = 0, btn_r_flag = 0, last_btn_r_flag = 0, trigger_on = 0;

    void reset_trigger_flag();
    void set_trigger_flag_busy();

    float test_channel[4] = {0.0f};

    time_counter delta_counter;
};

#endif // __cplusplus

#endif // CRSF_RECEIVER_H
