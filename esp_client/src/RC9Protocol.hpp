#ifdef RC9PROTOCOL
#endif
#ifdef __cplusplus
extern "C"
{
#endif
#include "crc_util.h"

#ifdef __cplusplus
}
#endif

#include <Arduino.h>
#include "XboxSeriesXControllerESP32_asukiaaa.hpp"

#define FRAME_HEAD_0_RC9 0xFC
#define FRAME_HEAD_1_RC9 0xFB
#define FRAME_END_0_RC9 0xFD
#define FRAME_END_1_RC9 0xFE
#define MAX_DATA_LENGTH_RC9 64
#define MAX_SUBSCRIBERS 5

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

class RC9Protocol
{
public:
    RC9Protocol(){
        Serial.begin(115200);
    }
    ~RC9Protocol(){}
    void send_data();
    void send_zero();
    void pack_data(XboxControllerNotificationParser *xboxNotif);

    uint8_t sendBuffer_[MAX_DATA_LENGTH_RC9 + 8];
    uint8_t rxIndex_; // 当前接收到的字节的索引
    serial_frame_mat_t rx_frame_mat; // 接收数据的数据帧结构体
    serial_frame_mat_t tx_frame_mat; // 发送数据的数据帧结构体
};


