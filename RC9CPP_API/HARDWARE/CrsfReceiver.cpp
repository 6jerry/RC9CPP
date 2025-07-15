#include "CrsfReceiver.h"

CrsfReceiver::CrsfReceiver(UART_HandleTypeDef *huart)
    : SerialDevice(huart),
      packet_byte_index_(0),
      rx_state_(CRSF_WAITING_FOR_ADDRESS),
      payload_ptr_(nullptr),
      crc_(CRSF_CRC_POLY) // 初始化CRC，CRSF协议使用0xD5
{
    for (int i = 0; i < CRSF_NUM_CHANNELS; ++i)
    {
        channels_[i] = CRSF_CHANNEL_VALUE_MID;
    }
}

int CrsfReceiver::getChannel(uint8_t channel_num) const
{
    if (channel_num >= 1 && channel_num <= CRSF_NUM_CHANNELS)
    {
        return channels_[channel_num - 1];
    }
    return CRSF_CHANNEL_VALUE_MID;
}

void CrsfReceiver::handleReceiveData(uint8_t byte)
{
    switch (rx_state_)
    {
    case CRSF_WAITING_FOR_ADDRESS:
        if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER || byte == CRSF_ADDRESS_BROADCAST)
        {
            current_rc_frame_.device_addr = byte;
            rx_state_ = CRSF_WAITING_FOR_FRAMESIZE;
        }
        break;

    case CRSF_WAITING_FOR_FRAMESIZE:
        current_rc_frame_.frame_size = byte;
        if (current_rc_frame_.frame_size == (CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC))
        {
            rx_state_ = CRSF_WAITING_FOR_TYPE;
        }
        else
        {
            rx_state_ = CRSF_WAITING_FOR_ADDRESS;
        }
        break;

    case CRSF_WAITING_FOR_TYPE:
        current_rc_frame_.type = byte;
        if (current_rc_frame_.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
            packet_byte_index_ = 0;
            payload_ptr_ = (uint8_t *)&current_rc_frame_.channels;
            rx_state_ = CRSF_WAITING_FOR_PAYLOAD_CHANNELS;
        }
        else
        {
            rx_state_ = CRSF_WAITING_FOR_ADDRESS;
        }
        break;

    case CRSF_WAITING_FOR_PAYLOAD_CHANNELS:
        payload_ptr_[packet_byte_index_++] = byte;
        if (packet_byte_index_ >= CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE)
        {
            rx_state_ = CRSF_WAITING_FOR_CRC_BYTE;
        }
        break;

    case CRSF_WAITING_FOR_CRC_BYTE:
        current_rc_frame_.crc = byte;
        rx_state_ = CRSF_PACKET_COMPLETE;

        // 计算CRC并校验
        // calculated_crc = crc_.calc((uint8_t *)&current_rc_frame_.type,
        // 1 + CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE);

        processRcChannelsPacket(); // CRC校验通过，处理数据包

        rx_state_ = CRSF_WAITING_FOR_ADDRESS;
        break;

    default:
        rx_state_ = CRSF_WAITING_FOR_ADDRESS;
        break;
    }
}

void CrsfReceiver::processRcChannelsPacket()
{
    channels_[0] = current_rc_frame_.channels.ch0;
    channels_[1] = current_rc_frame_.channels.ch1;
    channels_[2] = current_rc_frame_.channels.ch2;
    channels_[3] = current_rc_frame_.channels.ch3;
    channels_[4] = current_rc_frame_.channels.ch4;
    channels_[5] = current_rc_frame_.channels.ch5;
    channels_[6] = current_rc_frame_.channels.ch6;
    channels_[7] = current_rc_frame_.channels.ch7;
    channels_[8] = current_rc_frame_.channels.ch8;
    channels_[9] = current_rc_frame_.channels.ch9;
    channels_[10] = current_rc_frame_.channels.ch10;
    channels_[11] = current_rc_frame_.channels.ch11;
    channels_[12] = current_rc_frame_.channels.ch12;
    channels_[13] = current_rc_frame_.channels.ch13;
    channels_[14] = current_rc_frame_.channels.ch14;
    channels_[15] = current_rc_frame_.channels.ch15;

    map_value_compute();
    flag_set();

    if (last_btn_r_flag == 0 && btn_r_flag == 1 && trigger_on != 2)
    {
        trigger_on = 1;
    }

    last_btn_r_flag = btn_r_flag;

    delta_counter.get_DeltaTime_ms();
}

void CrsfReceiver::reset_trigger_flag()
{
    trigger_on = 0;
}

void CrsfReceiver::set_trigger_flag_busy()
{
    trigger_on = 2;
}

// 实现获取待发送的数据

__attribute__((section("dma_buffer_section"), aligned(4)))
uint8_t tx_buffer_[CRSF_MAX_PACKET_SIZE];
void CrsfReceiver::sendAttitude(float pitch, float roll, float yaw)
{
    // 根据 CRSF 协议，Payload 大小为 6 字节 (3 * int16_t)
    constexpr uint8_t PAYLOAD_SIZE = 6;
    // Frame Size 字段的值 = Payload Size + 2 (1 for Type, 1 for CRC)
    constexpr uint8_t FRAME_LENGTH_FIELD = PAYLOAD_SIZE + 2;
    // 整个数据帧的总长度（用于DMA发送）
    constexpr uint8_t TOTAL_FRAME_LENGTH = FRAME_LENGTH_FIELD + 2; // +2 for Address and Frame Size fields

    // 1. 将浮点数转换为协议要求的定点整数
    const int16_t pitch_scaled = static_cast<int16_t>(pitch * 10000.0f);
    const int16_t roll_scaled = static_cast<int16_t>(roll * 10000.0f);
    const int16_t yaw_scaled = static_cast<int16_t>(yaw * 10000.0f);

    // 2. 直接在 DMA 缓冲区中组装数据帧
    //    这里的索引和内容严格按照 CRSF 协议
    tx_buffer_[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // [Byte 0] 目标设备地址
    tx_buffer_[1] = FRAME_LENGTH_FIELD;             // [Byte 1] 帧长度 (类型+载荷+CRC) = 8
    tx_buffer_[2] = CRSF_FRAMETYPE_ATTITUDE;        // [Byte 2] 帧类型 (0x1E)

    // --- Payload (6 bytes) ---
    // 注意：CRSF 协议要求多字节数据为大端字节序 (Big-Endian)
    // STM32 是小端 (Little-Endian)，所以必须手动转换。
    tx_buffer_[3] = (pitch_scaled >> 8) & 0xFF; // [Byte 3] Pitch (MSB)
    tx_buffer_[4] = pitch_scaled & 0xFF;        // [Byte 4] Pitch (LSB)

    tx_buffer_[5] = (roll_scaled >> 8) & 0xFF; // [Byte 5] Roll (MSB)
    tx_buffer_[6] = roll_scaled & 0xFF;        // [Byte 6] Roll (LSB)

    tx_buffer_[7] = (yaw_scaled >> 8) & 0xFF; // [Byte 7] Yaw (MSB)
    tx_buffer_[8] = yaw_scaled & 0xFF;        // [Byte 8] Yaw (LSB)

    // 3. 计算 CRC 校验码
    //    CRC 的计算范围是从“帧类型”到 Payload 的末尾
    const uint8_t crc = crc_.calc(&tx_buffer_[2], 1 + PAYLOAD_SIZE);
    tx_buffer_[9] = crc; // [Byte 9] CRC

    // 4. 通过 DMA 发送整个数据帧
    HAL_UART_Transmit_DMA(huart_, tx_buffer_, TOTAL_FRAME_LENGTH);
}

/**
 * @brief 发送电池遥测数据 (Battery Telemetry)
 * @param voltage  电压，单位：V
 * @param current  电流，单位：A
 * @param capacity 已消耗容量，单位：mAh
 * @param remaining 剩余电量百分比，单位：%
 */
void CrsfReceiver::sendBattery(float voltage, float current, uint32_t capacity, uint8_t remaining)
{
    // 根据 CRSF 协议，Payload 大小为 8 字节 (uint16 + uint16 + uint24 + uint8)
    constexpr uint8_t PAYLOAD_SIZE = 8;
    // Frame Size 字段的值 = Payload Size + 2 (1 for Type, 1 for CRC)
    constexpr uint8_t FRAME_LENGTH_FIELD = PAYLOAD_SIZE + 2;
    // 整个数据帧的总长度（用于DMA发送）
    constexpr uint8_t TOTAL_FRAME_LENGTH = FRAME_LENGTH_FIELD + 2;

    // 1. 将浮点数转换为协议要求的定点整数
    const uint16_t voltage_scaled = static_cast<uint16_t>(voltage * 100.0f); // 协议单位: 0.01V
    const uint16_t current_scaled = static_cast<uint16_t>(current * 100.0f); // 协议单位: 0.01A

    // 2. 直接在 DMA 缓冲区中组装数据帧
    tx_buffer_[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // [Byte 0] 目标设备地址
    tx_buffer_[1] = FRAME_LENGTH_FIELD;             // [Byte 1] 帧长度 (类型+载荷+CRC) = 10
    tx_buffer_[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;  // [Byte 2] 帧类型 (0x08)

    // --- Payload (8 bytes) ---
    // 同样，所有多字节数据都使用大端字节序 (Big-Endian)
    tx_buffer_[3] = (voltage_scaled >> 8) & 0xFF; // [Byte 3] Voltage (MSB)
    tx_buffer_[4] = voltage_scaled & 0xFF;        // [Byte 4] Voltage (LSB)

    tx_buffer_[5] = (current_scaled >> 8) & 0xFF; // [Byte 5] Current (MSB)
    tx_buffer_[6] = current_scaled & 0xFF;        // [Byte 6] Current (LSB)

    // 24-bit capacity
    tx_buffer_[7] = (capacity >> 16) & 0xFF; // [Byte 7] Capacity (MSB)
    tx_buffer_[8] = (capacity >> 8) & 0xFF;  // [Byte 8] Capacity (MID)
    tx_buffer_[9] = capacity & 0xFF;         // [Byte 9] Capacity (LSB)

    tx_buffer_[10] = remaining; // [Byte 10] Remaining %

    // 3. 计算 CRC 校验码
    //    计算范围：从“帧类型”到 Payload 末尾
    const uint8_t crc = crc_.calc(&tx_buffer_[2], 1 + PAYLOAD_SIZE);
    tx_buffer_[11] = crc; // [Byte 11] CRC

    // 4. 通过 DMA 发送整个数据帧
    HAL_UART_Transmit_DMA(huart_, tx_buffer_, TOTAL_FRAME_LENGTH);
}

void CrsfReceiver::sendGps(double latitude, double longitude, uint16_t groundspeed,
                           uint16_t heading, uint16_t altitude, uint8_t satellites)
{
    // 根据 CRSF 协议，GPS Payload 大小为 15 字节
    // (int32+int32+uint16+uint16+uint16+uint8 = 4+4+2+2+2+1)
    constexpr uint8_t PAYLOAD_SIZE = 15;
    // Frame Size 字段的值 = Payload Size + 2 (1 for Type, 1 for CRC)
    constexpr uint8_t FRAME_LENGTH_FIELD = PAYLOAD_SIZE + 2; // 15 + 2 = 17
    // 整个数据帧的总长度（用于DMA发送）
    constexpr uint8_t TOTAL_FRAME_LENGTH = FRAME_LENGTH_FIELD + 2; // 17 + 2 = 19

    // 1. 将浮点数和整数转换为协议要求的定点格式
    const int32_t lat_scaled = static_cast<int32_t>(latitude * 1e7);  // 协议单位: 度 * 10,000,000
    const int32_t lon_scaled = static_cast<int32_t>(longitude * 1e7); // 协议单位: 度 * 10,000,000
    const uint16_t alt_scaled = altitude + 1000;                      // 协议单位: 米，带 1000米 偏移

    // 2. 直接在 DMA 缓冲区中组装数据帧
    tx_buffer_[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // [Byte 0] 目标设备地址
    tx_buffer_[1] = FRAME_LENGTH_FIELD;             // [Byte 1] 帧长度 (类型+载荷+CRC) = 17
    tx_buffer_[2] = CRSF_FRAMETYPE_GPS;             // [Byte 2] 帧类型 (0x02)

    // --- Payload (15 bytes) ---
    // 同样，所有多字节数据都使用大端字节序 (Big-Endian)
    tx_buffer_[3] = (lat_scaled >> 24) & 0xFF; // [Byte 3] Latitude (MSB)
    tx_buffer_[4] = (lat_scaled >> 16) & 0xFF; // [Byte 4]
    tx_buffer_[5] = (lat_scaled >> 8) & 0xFF;  // [Byte 5]
    tx_buffer_[6] = lat_scaled & 0xFF;         // [Byte 6] Latitude (LSB)

    tx_buffer_[7] = (lon_scaled >> 24) & 0xFF; // [Byte 7] Longitude (MSB)
    tx_buffer_[8] = (lon_scaled >> 16) & 0xFF; // [Byte 8]
    tx_buffer_[9] = (lon_scaled >> 8) & 0xFF;  // [Byte 9]
    tx_buffer_[10] = lon_scaled & 0xFF;        // [Byte 10] Longitude (LSB)

    tx_buffer_[11] = (groundspeed >> 8) & 0xFF; // [Byte 11] Groundspeed (MSB)
    tx_buffer_[12] = groundspeed & 0xFF;        // [Byte 12] Groundspeed (LSB)

    tx_buffer_[13] = (heading >> 8) & 0xFF; // [Byte 13] Heading (MSB)
    tx_buffer_[14] = heading & 0xFF;        // [Byte 14] Heading (LSB)

    tx_buffer_[15] = (alt_scaled >> 8) & 0xFF; // [Byte 15] Altitude (MSB)
    tx_buffer_[16] = alt_scaled & 0xFF;        // [Byte 16] Altitude (LSB)

    tx_buffer_[17] = satellites; // [Byte 17] Satellites count

    // 3. 计算 CRC 校验码
    //    计算范围：从“帧类型”到 Payload 末尾
    const uint8_t crc = crc_.calc(&tx_buffer_[2], 1 + PAYLOAD_SIZE);
    tx_buffer_[18] = crc; // [Byte 18] CRC

    // 4. 通过 DMA 发送整个数据帧
    HAL_UART_Transmit_DMA(huart_, tx_buffer_, TOTAL_FRAME_LENGTH);
}

// 其余函数保持不变
void CrsfReceiver::map_value_compute()
{
    // 原有代码不变
    if (channels_[0] >= 944 && channels_[0] <= 954)
    {
        right_H_map = 0.0f;
    }
    if (channels_[0] > 954 && channels_[0] <= 1769)
    {
        right_H_map = -((float)(channels_[0] - 954) / 815.0f);
    }
    if (channels_[0] < 944 && channels_[0] >= 174)
    {
        right_H_map = (float)(944 - channels_[0]) / 770.0f;
    }
    if (channels_[0] < 174)
    {
        right_H_map = 1.0f;
    }
    if (channels_[0] > 1769)
    {
        right_H_map = -1.0f;
    }

    if (channels_[2] >= 1051 && channels_[2] <= 1061)
    {
        left_V_map = 0.0f;
    }
    if (channels_[2] > 1061 && channels_[2] <= 1811)
    {
        left_V_map = (float)(channels_[2] - 1061) / 750.0f;
    }
    if (channels_[2] < 1051 && channels_[2] >= 183)
    {
        left_V_map = -((float)(1051 - channels_[2]) / 868.0f);
    }
    if (channels_[2] < 183)
    {
        left_V_map = -1.0f;
    }
    if (channels_[2] > 1811)
    {
        left_V_map = 1.0f;
    }

    if (channels_[1] < 222)
    {
        right_V_map = 0.0f;
    }
    if (channels_[1] > 1811)
    {
        right_V_map = 1.0f;
    }
    if (channels_[1] >= 222 && channels_[1] <= 1811)
    {
        right_V_map = (float)(channels_[1] - 222) / 1589.0f;
    }

    if (channels_[3] >= 939 && channels_[3] <= 949)
    {
        left_H_map = 0.0f;
    }
    if (channels_[3] > 949 && channels_[3] <= 1763)
    {
        left_H_map = (float)(channels_[3] - 949) / 814.0f;
    }
    if (channels_[3] < 939 && channels_[3] >= 174)
    {
        left_H_map = -((float)(939 - channels_[3]) / 765.0f);
    }
    if (channels_[3] < 174)
    {
        left_H_map = -1.0f;
    }
    if (channels_[3] > 1763)
    {
        left_H_map = 1.0f;
    }

    if (channels_[9] < 191)
    {
        roll_map = 0.0f;
    }
    if (channels_[9] > 1792)
    {
        roll_map = 1.0f;
    }
    if (channels_[9] >= 191 && channels_[9] <= 1792)
    {
        roll_map = (float)(channels_[9] - 191) / 1601.0f;
    }
}

void CrsfReceiver::flag_set()
{
    if (channels_[4] == BTN_ON)
    {
        sal_flag = 1;
    }
    if (channels_[4] == BTN_OFF)
    {
        sal_flag = 0;
    }

    if (channels_[8] == BTN_ON)
    {
        sar_flag = 1;
    }
    if (channels_[8] == BTN_OFF)
    {
        sar_flag = 0;
    }

    if (channels_[6] == BTN_ON)
    {
        btn_r_flag = 1;
    }
    if (channels_[6] == BTN_OFF)
    {
        btn_r_flag = 0;
    }

    if (channels_[5] == BTN_1)
    {
        l_flag = 0;
    }
    if (channels_[5] == BTN_2)
    {
        l_flag = 1;
    }
    if (channels_[5] == BTN_3)
    {
        l_flag = 2;
    }

    if (channels_[7] == BTN_1)
    {
        r_flag = 0;
    }
    if (channels_[7] == BTN_2)
    {
        r_flag = 1;
    }
    if (channels_[7] == BTN_3)
    {
        r_flag = 2;
    }
}
