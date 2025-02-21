#include "RC9Protocol.h"

// 使用你提供的联合体结构体

// 构造函数：传入 UART 句柄，是否启用 CRC 校验
RC9Protocol_cdc::RC9Protocol_cdc(bool enableCrcCheck)
    : SerialDevice(nullptr, cdc), state_(WAITING_FOR_HEADER_0), rxIndex_(0), enableCrcCheck_(enableCrcCheck)
{
}

// 实现接收数据的处理逻辑
void RC9Protocol_cdc::handleReceiveData(uint8_t byte)
{
    switch (state_)
    {
    case WAITING_FOR_HEADER_0:
        if (byte == FRAME_HEAD_0_RC9)
        {
            state_ = WAITING_FOR_HEADER_1;
            rx_frame_mat.frame_head[0] = byte; // 存储帧头
        }
        break;
    case WAITING_FOR_HEADER_1:
        if (byte == FRAME_HEAD_1_RC9)
        {
            state_ = WAITING_FOR_ID;
            rx_frame_mat.frame_head[1] = byte; // 存储帧头
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0;
        }
        break;
    case WAITING_FOR_ID:
        rx_frame_mat.frame_id = byte; // 存储帧ID
        state_ = WAITING_FOR_LENGTH;
        break;
    case WAITING_FOR_LENGTH:
        rx_frame_mat.data_length = byte; // 存储数据长度
        rxIndex_ = 0;
        state_ = WAITING_FOR_DATA;
        break;
    case WAITING_FOR_DATA:
        rx_frame_mat.rx_temp_data_mat[rxIndex_++] = byte; // 存储接收到的数据
        if (rxIndex_ >= rx_frame_mat.data_length)
        {

            state_ = WAITING_FOR_CRC_0;
        }
        break;
    case WAITING_FOR_CRC_0:
        rx_frame_mat.check_code.crc_buff[0] = byte; // 存储 CRC 校验的高字节
        state_ = WAITING_FOR_CRC_1;
        break;
    case WAITING_FOR_CRC_1:
        rx_frame_mat.check_code.crc_buff[1] = byte; // 存储 CRC 校验的低字节
        state_ = WAITING_FOR_END_0;
        break;
    case WAITING_FOR_END_0:
        if (byte == FRAME_END_0_RC9)
        {
            state_ = WAITING_FOR_END_1;
            rx_frame_mat.frame_end[0] = byte; // 存储帧尾
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0;
        }
        break;
    case WAITING_FOR_END_1:
        if (byte == FRAME_END_1_RC9)
        {
            rx_frame_mat.frame_end[1] = byte; // 存储帧尾

            for (uint8_t i = 0; i < rx_frame_mat.data_length; i++)
            {
                rx_frame_mat.data.buff_msg[i] = rx_frame_mat.rx_temp_data_mat[i];
            }

            state_ = WAITING_FOR_HEADER_0;
        }
        state_ = WAITING_FOR_HEADER_0;
        break;
    default:
        state_ = WAITING_FOR_HEADER_0;
        break;
    }
}

// 实现获取待发送的数据
void RC9Protocol_cdc::process_data()
{

    sendBuffer_[0] = FRAME_HEAD_0_RC9;
    sendBuffer_[1] = FRAME_HEAD_1_RC9;
    sendBuffer_[2] = tx_frame_mat.frame_id;
    sendBuffer_[3] = tx_frame_mat.data_length;

    for (int q = 0; q < tx_frame_mat.data_length; q++)
    {
        sendBuffer_[4 + q] = tx_frame_mat.data.buff_msg[q];
    }

    // 发送时仍然启用 CRC 校验
    tx_frame_mat.check_code.crc_code = CRC16_Table(tx_frame_mat.data.buff_msg, tx_frame_mat.data_length);
    sendBuffer_[4 + tx_frame_mat.data_length] = tx_frame_mat.check_code.crc_buff[0];
    sendBuffer_[5 + tx_frame_mat.data_length] = tx_frame_mat.check_code.crc_buff[1];
    sendBuffer_[6 + tx_frame_mat.data_length] = FRAME_END_0_RC9;
    sendBuffer_[7 + tx_frame_mat.data_length] = FRAME_END_1_RC9;

    CDC_Transmit_FS(sendBuffer_, tx_frame_mat.data_length + 8);
}

// 注册观察者函数

void RC9Protocol_cdc::load_Txfloat(uint8_t data_id, const float *data_float, uint8_t numbers)
{
    tx_frame_mat.frame_id = data_id;
    tx_frame_mat.data_length = numbers * 4;

    for (uint8_t i = 0; i < numbers; i++)
    {
        tx_frame_mat.data.msg_get[i] = data_float[i];
    }
}
