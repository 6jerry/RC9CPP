#include "RC9Protocol.h"

RC9Protocol::RC9Protocol(uart_type type_, UART_HandleTypeDef *huart, bool enableCrcCheck)
    : SerialDevice(huart, type_), state_(WAITING_FOR_HEADER_0), rxIndex_(0), enableCrcCheck_(enableCrcCheck), type(type_)
{
    // sendQueue_ = osMessageQueueNew(MAX_QUEUE_SIZE, sizeof(serial_frame_mat_t), NULL);
}

void RC9Protocol::initQueue()
{
    sendQueue_ = osMessageQueueNew(MAX_QUEUE_SIZE, sizeof(serial_frame_mat_t), NULL);
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

                    // 数据接收成功，调用回调函数
                    for (int i = 0; i < observerCount_; i++)
                    {
                        if (subscribers[i] != nullptr)
                        {
                            subscribers[i]->DataReceivedCallback(rx_frame_mat.data.buff_msg, rx_frame_mat.data.msg_get, rx_frame_mat.frame_id, rx_frame_mat.data_length);
                        }
                    }
                    state_ = WAITING_FOR_HEADER_0;
                }
            }
            if (enableCrcCheck_ == 0)
            {
                for (uint8_t i = 0; i < rx_frame_mat.data_length; i++)
                {
                    rx_frame_mat.data.buff_msg[i] = rx_frame_mat.rx_temp_data_mat[i];
                }

                // 数据接收成功，调用回调函数
                for (int i = 0; i < observerCount_; i++)
                {
                    if (subscribers[i] != nullptr)
                    {
                        subscribers[i]->DataReceivedCallback(rx_frame_mat.data.buff_msg, rx_frame_mat.data.msg_get, rx_frame_mat.frame_id, rx_frame_mat.data_length);
                    }
                }
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

    serial_frame_mat_t data_to_send;

    // 尝试从队列中获取一个数据包
    osStatus_t status = osMessageQueueGet(sendQueue_, &data_to_send, NULL, 0); // 非阻塞
    if (status == osOK)
    {
        // 填充发送缓冲区
        sendBuffer_[0] = FRAME_HEAD_0_RC9;
        sendBuffer_[1] = FRAME_HEAD_1_RC9;
        sendBuffer_[2] = data_to_send.frame_id;
        sendBuffer_[3] = data_to_send.data_length;

        for (int q = 0; q < data_to_send.data_length; q++)
        {
            sendBuffer_[4 + q] = data_to_send.data.buff_msg[q];
        }

        if (enableCrcCheck_)
        {
            data_to_send.check_code.crc_code = CRC16_Table(data_to_send.data.buff_msg, data_to_send.data_length);
        }
        else
        {
            data_to_send.check_code.crc_code = 0x0000;
        }

        sendBuffer_[4 + data_to_send.data_length] = data_to_send.check_code.crc_buff[0];
        sendBuffer_[5 + data_to_send.data_length] = data_to_send.check_code.crc_buff[1];
        sendBuffer_[6 + data_to_send.data_length] = FRAME_END_0_RC9;
        sendBuffer_[7 + data_to_send.data_length] = FRAME_END_1_RC9;

        // 发送数据
        if (type == uart)
        {
            HAL_UART_Transmit_DMA(huart_, sendBuffer_, data_to_send.data_length + 8);
        }
        else if (type == cdc)
        {
            // CDC_Transmit_FS(sendBuffer_, data_to_send.data_length + 8);
        }
    }
    // 如果队列为空，返回并等待下次处理
}

bool RC9Protocol::load_Txfloat(uint8_t data_id, const float *data_float, uint8_t numbers)
{
    tx_frame_mat.frame_id = data_id;
    tx_frame_mat.data_length = numbers * 4;

    for (uint8_t i = 0; i < numbers; i++)
    {
        tx_frame_mat.data.msg_get[i] = data_float[i];
    }

    // 尝试将数据包放入队列中
    osStatus_t status = osMessageQueuePut(sendQueue_, &tx_frame_mat, 0, 0); // 非阻塞
    if (status != osOK)
    {
        // 队列已满，返回加载失败
        return false;
    }
    else
    {
        return true;
    }
}

bool RC9Protocol::load_Txbyte(uint8_t id, const uint8_t *data, uint8_t length)
{
    tx_frame_mat.frame_id = id;
    tx_frame_mat.data_length = length;

    for (uint8_t i = 0; i < length; i++)
    {
        tx_frame_mat.data.buff_msg[i] = data[i];
    }

    // 尝试将数据包放入队列中
    osStatus_t status = osMessageQueuePut(sendQueue_, &tx_frame_mat, 0, 0); // 非阻塞
    if (status != osOK)
    {

        return false;
    }
    else
    {
        return true;
    }
}

bool RC9Protocol::addSubscriber(RC9subscriber *subscriber)
{
    if (observerCount_ < MAX_SUBSCRIBERS)
    {
        subscribers[observerCount_] = subscriber;
        observerCount_++;
        return true;
    }
    else
    {
        return false;
    }
}

bool RC9subscriber::sendFloatData(uint8_t id, const float *data, uint8_t count)
{
    return serialport_->load_Txfloat(id, data, count);
}
bool RC9subscriber::sendByteData(uint8_t id, const uint8_t *data, uint8_t length)
{
    return serialport_->load_Txbyte(id, data, length);
}
void RC9subscriber::addport(RC9Protocol *port_)
{
    serialport_ = port_;

    serialport_->addSubscriber(this); // 互相传指针
}