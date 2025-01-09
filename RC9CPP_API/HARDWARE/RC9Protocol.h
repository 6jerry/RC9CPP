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
#define FRAME_HEAD_0_RC9 0xFC
#define FRAME_HEAD_1_RC9 0xFB
#define FRAME_END_0_RC9 0xFD
#define FRAME_END_1_RC9 0xFE
#define MAX_DATA_LENGTH_RC9 64
#define MAX_SUBSCRIBERS 5

// 该类的订阅者,一般这种生产数据的类都是发布者
class RC9Protocol_subscriber
{
public:
    virtual ~RC9Protocol_subscriber() {}
    void *subtarget = nullptr; // 用来建立双向联系的
    // 接口方法：更新数据
    virtual void update(uint8_t data_id, uint8_t data_length, const uint8_t *data_char, const float *data_float) = 0;
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

class RC9Protocol : public SerialDevice, public ITaskProcessor, public rcnode
{

public:
    // 面向用户的友好接口函数
   
    void load_Txfloat(uint8_t data_id, const float *data_float, uint8_t numbers); // id ,待传输的数组，要发送的数组长度
    float Get_Rxfloat(uint8_t numbers);                                           // 获取接收到的第几个float，传入“第几个”

public:
    // 构造函数，传入 UART 句柄，是否启用发送任务，是否启用 CRC 校验
    RC9Protocol(UART_HandleTypeDef *huart, bool enableCrcCheck = true);

    // 实现接收数据的处理逻辑
    void handleReceiveData(uint8_t byte);

    bool addsubscriber(RC9Protocol_subscriber *observer);
    void process_data();

    serial_frame_mat_t rx_frame_mat; // 接收数据的数据帧结构体
    serial_frame_mat_t tx_frame_mat; // 发送数据的数据帧结构体

    uint8_t msgin(uint8_t rcnID_, const void *data) override;
    uint8_t msgout(uint8_t rcnID_, void *output) override;

private:
    uint8_t sendBuffer_[MAX_DATA_LENGTH_RC9 + 8];
    uint8_t rxIndex_; // 当前接收到的字节的索引

    void publish(uint8_t data_id, uint8_t datalenth, const uint8_t *data_char, const float *data_float); // 发布数据

    bool enableCrcCheck_; // 是否启用 CRC 校验
    RC9Protocol_subscriber *observers_[MAX_SUBSCRIBERS] = {nullptr};
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
};
#endif
#endif // RC9_PROTOCOL_H
