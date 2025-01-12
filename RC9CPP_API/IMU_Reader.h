#ifndef IMUREADER_H
#define IMUREADER_H

#include <stdint.h> // 用于 uint8_t 和 int16_t
#include <stddef.h> // 用于 size_t

class IMUReader {
public:
    // 构造函数和析构函数
    IMUReader(UART_HandleTypeDef *huart);
    ~IMUReader();
    // 定义结构体来存储解析后的角度
        typedef struct {
        float Roll;   // 滚转角
        float Pitch;  // 俯仰角
        float Yaw;    // 航向角
    } Angles;
    // 读取三轴角度
    void readAngles();

private:
    // 发送命令
    void sendCommand();
    // 解析数据
    void parseResponse(const uint8_t* response);
};

#endif // IMUREADER_H