#ifndef STP23L_H
#define STP23L_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "Serial_device.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// 与商家文档对应的测量点数据结构
typedef struct
{
    int16_t distance;   // 测量距离（单位：mm）
    uint16_t noise;     // 环境噪声
    uint32_t peak;      // 接收强度
    uint8_t confidence; // 置信度
    uint32_t intg;      // 积分次数
    int16_t reftof;     // 温度表征值
} LidarPointTypedef;

class STP23L : public SerialDevice
{
public:
    // 构造函数，传入对应的 UART 句柄
    STP23L(UART_HandleTypeDef *huart);

    // 必须实现的串口接收数据处理函数
    virtual void handleReceiveData(uint8_t byte);

    // 获取解析后计算的平均距离（单位：mm）
    uint16_t getAverageDistance() const;

private:
    // 定义数据包解析状态机
    enum RxState
    {
        WAIT_FOR_HEADER,       // 等待 4 字节 0xAA 头
        WAIT_FOR_DEVICE_ADDR,  // 设备地址
        WAIT_FOR_CMD,          // 命令码（应为 0x02 获取测量数据）
        WAIT_FOR_CHK_OFFSET1,  // 第1字节偏移（一般为 0x00）
        WAIT_FOR_CHK_OFFSET2,  // 第2字节偏移（一般为 0x00）
        WAIT_FOR_DATALEN_LOW,  // 数据长度低字节
        WAIT_FOR_DATALEN_HIGH, // 数据长度高字节
        RECEIVE_PAYLOAD,       // 接收数据域（包括测量点数据和时间戳，共 184 字节）
        WAIT_FOR_CHECKSUM      // 等待校验字节（收到后直接丢弃，不进行校验）
    } rx_state;

    uint8_t headerCount;                      // 记录连续收到的 0xAA 个数
    uint16_t dataLen;                         // 数据长度字段（理论上应为 184，即 12×15+4）
    uint16_t payloadIndex;                    // 数据域接收索引
    static const uint16_t PAYLOAD_SIZE = 184; // 数据域大小

    uint8_t payloadBuffer[PAYLOAD_SIZE]; // 用于存放数据域（测量点 + 时间戳）的缓冲区

    LidarPointTypedef points[12]; // 存储 12 个测量点数据
    uint16_t avg_distance;        // 计算后的平均距离

    // 当接收完整个数据包后解析数据
    void processPacket();
};

#endif // __cplusplus

#endif // STP23L_H
