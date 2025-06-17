#include "CrsfReceiver.h"

CrsfReceiver::CrsfReceiver(UART_HandleTypeDef *huart)
    : SerialDevice(huart),
      packet_byte_index_(0),
      rx_state_(CRSF_WAITING_FOR_ADDRESS),
      payload_ptr_(nullptr) // 初始化 payload_ptr_
{
    // 初始化通道值
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
        if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER || byte == CRSF_ADDRESS_BROADCAST) // 期望接收飞控或广播地址
        {
            current_rc_frame_.device_addr = byte;
            rx_state_ = CRSF_WAITING_FOR_FRAMESIZE;
        }
        else
        {
            // 如果不是我们期望的地址，丢弃当前字节，继续等待地址
            // printf("CRSF: Unexpected address 0x%02X, waiting for address.\n", byte);
            // 保持 CRSF_WAITING_FOR_ADDRESS 状态
        }
        break;

    case CRSF_WAITING_FOR_FRAMESIZE:
        current_rc_frame_.frame_size = byte;
        // 帧长度校验：对于 RC_CHANNELS_PACKED，frame_size 应该是 CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE (22)
        // 加上 Type (1) 和 CRC (1)，所以 frame_size 应该是 22 + 1 + 1 = 24
        if (current_rc_frame_.frame_size == (CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC))
        {
            rx_state_ = CRSF_WAITING_FOR_TYPE;
        }
        else
        {
            // 帧长度不匹配，重置状态机，等待下一个地址
            // printf("CRSF: Invalid frame_size %d (expected %d), resetting.\n", current_rc_frame_.frame_size, CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
            rx_state_ = CRSF_WAITING_FOR_ADDRESS;
        }
        break;

    case CRSF_WAITING_FOR_TYPE:
        current_rc_frame_.type = byte;
        // 校验帧类型是否为 RC_CHANNELS_PACKED
        if (current_rc_frame_.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
            packet_byte_index_ = 0;                                // 重置 payload 字节索引
            payload_ptr_ = (uint8_t *)&current_rc_frame_.channels; // 初始化 payload_ptr_ 指向 channels 部分
            rx_state_ = CRSF_WAITING_FOR_PAYLOAD_CHANNELS;
        }
        else
        {
            // 如果类型不匹配，重置状态机，等待下一个地址
            // printf("CRSF: Unexpected frame type 0x%02X, resetting.\n", byte);
            rx_state_ = CRSF_WAITING_FOR_ADDRESS;
        }
        break;

    case CRSF_WAITING_FOR_PAYLOAD_CHANNELS:
        // 填充 channels 结构体的字节
        payload_ptr_[packet_byte_index_++] = byte;

        if (packet_byte_index_ >= CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE)
        {
            // 所有通道数据字节已接收完毕
            rx_state_ = CRSF_WAITING_FOR_CRC_BYTE;
        }
        break;

    case CRSF_WAITING_FOR_CRC_BYTE:
        current_rc_frame_.crc = byte;     // 接收 CRC 字节 (尽管不校验)
        rx_state_ = CRSF_PACKET_COMPLETE; // 标记包已完成

        // fallthrough to CRSF_PACKET_COMPLETE
        // 注意：这里 HWT101CT 示例是直接调用 processDecodedData()，然后重置状态机。
        // 我们也在这里处理包。

    case CRSF_PACKET_COMPLETE:
        processRcChannelsPacket();            // 处理接收到的 RC Channels 包
        rx_state_ = CRSF_WAITING_FOR_ADDRESS; // 重置状态机，等待下一个完整帧
        // 注意：这里假定当前字节 `byte` 是上一个包的最后一个字节。
        // 下一个 `handleReceiveData` 调用将处理新包的第一个字节。
        break;

    default:
        // 异常状态，重置
        rx_state_ = CRSF_WAITING_FOR_ADDRESS;
        // printf("CRSF: Unknown state, resetting.\n");
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

    if (last_btn_r_flag == 0 && btn_r_flag == 1)
    {
        BTN_L_CALLBACK();
    }

    last_btn_r_flag = btn_r_flag;

    switch (r_flag)
    {
    case 1:
        R_1();
        break;

    case 2:
        R_2();
        break;
    case 3:
        R_3();
        break;

    default:
        break;
    }

    switch (l_flag)
    {
    case 1:
        L_1();
        break;

    case 2:
        L_2();
        break;
    case 3:
        L_3();
        break;

    default:
        break;
    }

    if (sar_flag == 1)
    {
        SAR_ON();
    }

    else if (sar_flag == 2)
    {
        SAR_OFF();
    }

    if (sal_flag == 1)
    {
        SAL_ON();
    }

    else if (sal_flag == 2)
    {
        SAL_OFF();
    }
}

void CrsfReceiver::map_value_compute()
{
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

    if (channels_[1] >= 1034 && channels_[1] <= 1044)
    {
        right_V_map = 0.0f;
    }
    if (channels_[1] > 1044 && channels_[1] <= 1811)
    {
        right_V_map = (float)(channels_[1] - 1044) / 767.0f;
    }
    if (channels_[1] < 1034 && channels_[1] >= 222)
    {
        right_V_map = -((float)(1034 - channels_[1]) / 812.0f);
    }
    if (channels_[1] < 222)
    {
        right_V_map = -1.0f;
    }
    if (channels_[1] > 1811)
    {
        right_V_map = 1.0f;
    }

    if (channels_[2] < 183)
    {
        left_V_map = 0.0f;
    }
    if (channels_[2] > 1750)
    {
        left_V_map = 1.0f;
    }
    if (channels_[2] >= 183 && channels_[2] <= 1750)
    {
        left_V_map = (float)(channels_[2] - 183) / 1567.0f;
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
        sal_flag = 2;
    }

    if (channels_[8] == BTN_ON)
    {
        sar_flag = 1;
    }
    if (channels_[8] == BTN_OFF)
    {
        sar_flag = 2;
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
        l_flag = 1;
    }
    if (channels_[5] == BTN_2)
    {
        l_flag = 2;
    }
    if (channels_[5] == BTN_3)
    {
        l_flag = 3;
    }

    if (channels_[7] == BTN_1)
    {
        r_flag = 1;
    }
    if (channels_[7] == BTN_2)
    {
        r_flag = 2;
    }
    if (channels_[7] == BTN_3)
    {
        r_flag = 3;
    }
}
