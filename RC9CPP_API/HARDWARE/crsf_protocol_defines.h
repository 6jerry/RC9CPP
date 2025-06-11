#ifndef CRSF_PROTOCOL_DEFINES_H
#define CRSF_PROTOCOL_DEFINES_H

#include <stdint.h>

// CRSF Protocol Defines
#define CRSF_BAUDRATE 420000                            // 波特率
#define CRSF_NUM_CHANNELS 16                            // 通道数
#define CRSF_CHANNEL_VALUE_MIN 172                      // 通道最小值限制
#define CRSF_CHANNEL_VALUE_MID 992                      // 通道中位点
#define CRSF_CHANNEL_VALUE_MAX 1811                     // 通道最大值限制
#define CRSF_MAX_PACKET_SIZE 64                         // 最大数据包大小
#define CRSF_MAX_PAYLOAD_LEN (CRSF_MAX_PACKET_SIZE - 4) // Max size of payload in [dest] [len] [type] [payload] [crc8]

enum
{
    CRSF_FRAME_LENGTH_ADDRESS = 1,
    CRSF_FRAME_LENGTH_FRAMELENGTH = 1,
    CRSF_FRAME_LENGTH_TYPE = 1,
    CRSF_FRAME_LENGTH_CRC = 1,
    CRSF_FRAME_LENGTH_TYPE_CRC = 2,
    CRSF_FRAME_LENGTH_EXT_TYPE_CRC = 4,
    CRSF_FRAME_LENGTH_NON_PAYLOAD = 4,
};

enum
{
    CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
    CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
    CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
    CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22, // 11 bits per channel * 16 channels = 22 bytes.
    CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
};

typedef enum
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C,
} crsf_frame_type_e;

typedef enum
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
} crsf_addr_e;

// 使用 __attribute__((packed)) 确保结构体紧密打包
#define PACKED __attribute__((packed))

typedef struct
{
    uint8_t device_addr; // from crsf_addr_e
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t type;        // from crsf_frame_type_e
    // Data payload follows this header in the actual byte stream
} PACKED Crsf_Header_t;

typedef struct
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED crsf_channels_t;

typedef struct crsfPayloadLinkstatistics_s
{
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} PACKED CrsfLinkStatistics_t;

#endif // CRSF_PROTOCOL_DEFINES_H
