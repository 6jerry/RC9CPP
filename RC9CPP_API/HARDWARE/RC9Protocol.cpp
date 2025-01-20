#include "RC9Protocol.h"

// 使用你提供的联合体结构体

// 构造函数：传入 UART 句柄，是否启用 CRC 校验
RC9Protocol::RC9Protocol(UART_HandleTypeDef *huart, bool enableCrcCheck)
    : SerialDevice(huart), state_(WAITING_FOR_HEADER_0), rxIndex_(0), enableCrcCheck_(enableCrcCheck)
{
    for (int i = 0; i < MAX_SUBSCRIBERS; i++)
    {
        observers_[i] = nullptr;
    }
}

// 实现接收数据的处理逻辑
void RC9Protocol::handleReceiveData(uint8_t byte)
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
            if (enableCrcCheck_)
            {
                // 计算 CRC 并与接收到的 CRC 进行比较
                rx_frame_mat.crc_calculated = CRC16_Table(rx_frame_mat.rx_temp_data_mat, rx_frame_mat.data_length);
                if (rx_frame_mat.crc_calculated == rx_frame_mat.check_code.crc_code)
                {
                    for (uint8_t i = 0; i < rx_frame_mat.data_length; i++)
                    {
                        rx_frame_mat.data.buff_msg[i] = rx_frame_mat.rx_temp_data_mat[i];
                    }

                    publish(rx_frame_mat.frame_id, rx_frame_mat.data_length, rx_frame_mat.data.buff_msg, rx_frame_mat.data.msg_get); // 发布数据

                    state_ = WAITING_FOR_HEADER_0;
                }
            }
            if (enableCrcCheck_ == 0)
            {
                for (uint8_t i = 0; i < rx_frame_mat.data_length; i++)
                {
                    rx_frame_mat.data.buff_msg[i] = rx_frame_mat.rx_temp_data_mat[i];
                }

                publish(rx_frame_mat.frame_id, rx_frame_mat.data_length, rx_frame_mat.data.buff_msg, rx_frame_mat.data.msg_get);

                state_ = WAITING_FOR_HEADER_0;
            }
        }
        state_ = WAITING_FOR_HEADER_0;
        break;
    default:
        state_ = WAITING_FOR_HEADER_0;
        break;
    }
}

// 实现获取待发送的数据
void RC9Protocol::process_data()
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

    HAL_UART_Transmit(huart_, sendBuffer_, tx_frame_mat.data_length + 8, HAL_MAX_DELAY);
}

// 注册观察者函数
bool RC9Protocol::addsubscriber(RC9Protocol_subscriber *observer)
{
    if (observerCount_ < MAX_SUBSCRIBERS)
    {
        observers_[observerCount_++] = observer;
        observer->subtarget = this; // 建立联系
        return true;
    }
    return false; // 注册失败，观察者数量已达到上限
}

void RC9Protocol::publish(uint8_t data_id, uint8_t datalenth, const uint8_t *data_char, const float *data_float)
{

    for (int i = 0; i < observerCount_; i++)
    {
        if (observers_[i] != nullptr)
        {
            observers_[i]->update(data_id, datalenth, data_char, data_float); // 调用观察者的update方法
        }
    }
}
