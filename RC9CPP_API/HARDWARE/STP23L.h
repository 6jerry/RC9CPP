#ifndef STP23L_H
#define STP23L_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "Serial_device.h"
#include "usart.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define USART_REC_LEN 200 // 定义最大接收字节数 200
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define HEADER 0xAA            /* 起始符 */
#define device_address 0x00    /* 设备地址 */
#define chunk_offset 0x00      /* 偏移地址命令 */
#define PACK_GET_DISTANCE 0x02 /* 获取测量数据命令 */
#define PACK_RESET_SYSTEM 0x0D /* 复位命令 */
#define PACK_STOP 0x0F         /* 停止测量数据传输命令 */
#define PACK_ACK 0x10          /* 应答码命令 */
#define PACK_VERSION 0x14      /* 获取传感器信息命令 */
typedef struct
{
    int16_t distance;   /* 距离数据：测量目标距离单位 mm */
    uint16_t noise;     /* 环境噪声：当前测量环境下的外部环境噪声，越大说明噪声越大 */
    uint32_t peak;      /* 接收强度信息：测量目标反射回的光强度 */
    uint8_t confidence; /* 置信度：由环境噪声和接收强度信息融合后的测量点的可信度 */
    uint32_t intg;      /* 积分次数：当前传感器测量的积分次数 */
    int16_t reftof;     /* 温度表征值：测量芯片内部温度变化表征值，只是一个温度变化量无法与真实温度对应 */
} LidarPointTypedef;

class STP23L : public SerialDevice
{
public:
    void handleReceiveData(uint8_t byte);
    char USART_RX_BUF[USART_REC_LEN]; // 接收缓冲,最大USART_REC_LEN个字节.
    uint16_t point1;
    LidarPointTypedef Pack_Data[12]; /* 雷达接收的数据储存在这个变量之中 */
    LidarPointTypedef Pack_sum;      /* 输出结果储存 */
    uint16_t receive_cnt;
    uint8_t confidence;
    uint16_t distance, noise, reftof;
    uint32_t peak, intg;

    uint8_t Hand(char *a);
    void CLR_Buf(void);
    void data_process(void);

    STP23L(UART_HandleTypeDef *huart_);

    uint8_t state = 0;      // 状态位
    uint8_t crc = 0;        // 校验和
    uint8_t cnt = 0;        // 用于一帧12个点的计数
    uint8_t PACK_FLAG = 0;  // 命令标志位
    uint8_t data_len = 0;   // 数据长度
    uint32_t timestamp = 0; // 时间戳
    uint8_t state_flag = 1; // 转入数据接收标志位
    uint8_t temp_data;

    float real_distance = 0.0f;
};

#endif // __cplusplus

#endif // STP23L_H
