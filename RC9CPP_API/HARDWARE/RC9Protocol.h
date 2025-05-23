#ifndef RC9_PROTOCOL_H
#define RC9_PROTOCOL_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "crc_util.h"
#include "TaskManager.h"
#include "Serial_device.h"
#include "netswitch.h"
#include "rcncore.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class RC9Protocol;
#define FRAME_HEAD_0_RC9 0xFC
#define FRAME_HEAD_1_RC9 0xFB
#define FRAME_END_0_RC9 0xFD
#define FRAME_END_1_RC9 0xFE
#define MAX_DATA_LENGTH_RC9 64
#define MAX_SUBSCRIBERS 5

class RC9subscriber
{
public:
    virtual void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) {}; // 数据回调函数
    // 用户调用此函数发送浮动数据
    bool sendFloatData(uint8_t id, const float *data, uint8_t count);

    // 用户调用此函数发送字节数据
    bool sendByteData(uint8_t id, const uint8_t *data, uint8_t length);

    void addport(RC9Protocol *port_);

protected:
private:
    RC9Protocol *serialport_ = nullptr;
};

typedef struct serial_frame_mat
{
    uint8_t data_length = 0; // 数据载荷的字节数
    uint8_t frame_head[2];
    uint8_t frame_id = 0;
    uint16_t crc_calculated = 0;
    uint8_t rx_temp_data_mat[MAX_DATA_LENGTH_RC9];
    union data
    {
        float msg_get[MAX_DATA_LENGTH_RC9 / 4] = {0.0f}; // 用于浮点数的接收
        uint8_t buff_msg[MAX_DATA_LENGTH_RC9];           // 用于字节流的接收
    } data;
    union check_code
    {
        uint16_t crc_code;
        uint8_t crc_buff[2]; // CRC 校验的字节形式
    } check_code;
    uint8_t frame_end[2];
} serial_frame_mat_t;

class RC9Protocol : public SerialDevice, public ITaskProcessor
{

public:
    bool load_Txfloat(uint8_t id, const float *data, uint8_t count);

    // 实现发送字节数据
    bool load_Txbyte(uint8_t id, const uint8_t *data, uint8_t length);

    bool addSubscriber(RC9subscriber *subscriber);

public:
    // 构造函数，传入 UART 句柄，是否启用发送任务，是否启用 CRC 校验
    RC9Protocol(uart_type type_ = uart, UART_HandleTypeDef *huart = nullptr, bool enableCrcCheck = false);

    // 实现接收数据的处理逻辑
    void handleReceiveData(uint8_t byte) override;

    void process_data();

    serial_frame_mat_t rx_frame_mat; // 接收数据的数据帧结构体
    serial_frame_mat_t tx_frame_mat; // 发送数据的数据帧结构体

    void initQueue();

private:
    osMessageQueueId_t sendQueue_;
    static const uint8_t MAX_QUEUE_SIZE = 5; // 最大队列大小

    uint8_t sendBuffer_[MAX_DATA_LENGTH_RC9 + 8];
    uint8_t rxIndex_; // 当前接收到的字节的索引

    bool enableCrcCheck_; // 是否启用 CRC 校验

    int observerCount_ = 0;
    // 状态机
    enum rxState
    {
        WAITING_FOR_HEADER_0,
        WAITING_FOR_HEADER_1,
        WAITING_FOR_ID,
        WAITING_FOR_LENGTH,
        WAITING_FOR_DATA,
        WAITING_FOR_CRC_0,
        WAITING_FOR_CRC_1,
        WAITING_FOR_END_0,
        WAITING_FOR_END_1
    } state_;

    RC9subscriber *subscribers[MAX_SUBSCRIBERS]; // 订阅者数组
    uint8_t subscriberCount_ = 0;                // 订阅者数量

    uart_type type = uart;
};
#endif
#endif // RC9_PROTOCOL_H