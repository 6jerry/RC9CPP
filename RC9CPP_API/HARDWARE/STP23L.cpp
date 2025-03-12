#include "STP23L.h"
#include <string.h>

STP23L::STP23L(UART_HandleTypeDef *huart)
    : SerialDevice(huart),
      rx_state(WAIT_FOR_HEADER),
      headerCount(0),
      dataLen(0),
      payloadIndex(0),
      avg_distance(0)
{
    memset(payloadBuffer, 0, sizeof(payloadBuffer));
}

uint16_t STP23L::getAverageDistance() const
{
    return avg_distance;
}

void STP23L::handleReceiveData(uint8_t byte)
{
    switch (rx_state)
    {
    case WAIT_FOR_HEADER:
        if (byte == 0xAA)
        {
            headerCount++;
            if (headerCount == 4)
            {
                rx_state = WAIT_FOR_DEVICE_ADDR;
            }
        }
        else
        {
            // 非头字节则重置计数
            headerCount = 0;
        }
        break;

    case WAIT_FOR_DEVICE_ADDR:
        // 可选：验证设备地址（一般为 0x00），此处直接进入下一状态
        rx_state = WAIT_FOR_CMD;
        break;

    case WAIT_FOR_CMD:
        // 可选：验证命令码（应为 0x02 获取测量数据），直接进入下一状态
        rx_state = WAIT_FOR_CHK_OFFSET1;
        break;

    case WAIT_FOR_CHK_OFFSET1:
        // 略过偏移地址第1字节
        rx_state = WAIT_FOR_CHK_OFFSET2;
        break;

    case WAIT_FOR_CHK_OFFSET2:
        // 略过偏移地址第2字节
        rx_state = WAIT_FOR_DATALEN_LOW;
        break;

    case WAIT_FOR_DATALEN_LOW:
        dataLen = byte; // 存储数据长度低字节
        rx_state = WAIT_FOR_DATALEN_HIGH;
        break;

    case WAIT_FOR_DATALEN_HIGH:
        dataLen |= ((uint16_t)byte << 8); // 合成数据长度
        // 可选：可检查 dataLen 是否符合预期（184），此处直接进入数据接收
        payloadIndex = 0;
        rx_state = RECEIVE_PAYLOAD;
        break;

    case RECEIVE_PAYLOAD:
        if (payloadIndex < PAYLOAD_SIZE)
        {
            payloadBuffer[payloadIndex++] = byte;
            if (payloadIndex == PAYLOAD_SIZE)
            {
                rx_state = WAIT_FOR_CHECKSUM;
            }
        }
        break;

    case WAIT_FOR_CHECKSUM:
        // 收到校验字节，但这里不进行校验，直接处理数据包
        processPacket();
        // 重置状态，等待下一个数据包
        rx_state = WAIT_FOR_HEADER;
        headerCount = 0;
        break;

    default:
        rx_state = WAIT_FOR_HEADER;
        headerCount = 0;
        break;
    }
}

void STP23L::processPacket()
{
    // 数据域 payloadBuffer 结构：
    // 前 180 字节：12 个测量点，每个 15 字节
    // 后 4 字节：时间戳（此处不做处理）
    uint32_t sum = 0;
    uint8_t validCount = 0;

    for (int i = 0; i < 12; i++)
    {
        uint16_t offset = i * 15;
        // 解析距离数据（2 字节，低字节在前）
        int16_t distance = (int16_t)(payloadBuffer[offset] | (payloadBuffer[offset + 1] << 8));
        points[i].distance = distance;

        // 其他数据字段（如噪声、强度、置信度等）也可按需要解析，这里给出参考
        points[i].noise = payloadBuffer[offset + 2] | (payloadBuffer[offset + 3] << 8);
        points[i].peak = payloadBuffer[offset + 4] | (payloadBuffer[offset + 5] << 8) |
                         (payloadBuffer[offset + 6] << 16) | (payloadBuffer[offset + 7] << 24);
        points[i].confidence = payloadBuffer[offset + 8];
        points[i].intg = payloadBuffer[offset + 9] | (payloadBuffer[offset + 10] << 8) |
                         (payloadBuffer[offset + 11] << 16) | (payloadBuffer[offset + 12] << 24);
        points[i].reftof = (int16_t)(payloadBuffer[offset + 13] | (payloadBuffer[offset + 14] << 8));

        if (distance != 0)
        {
            sum += distance;
            validCount++;
        }
    }

    if (validCount > 0)
    {
        avg_distance = sum / validCount;
    }
    else
    {
        avg_distance = 0;
    }

    // 此处可加入后续数据处理，比如发送数据到上层任务或打印调试信息
}
