/**
 * @file encoder.h
 * @brief 编码器数据接收与处理模块
 * @details 实现编码器数据的接收、解析和距离计算功能
 * @version 1.0
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
#include "Serial_device.h"
#include "math.h"
#include "imu.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @class Encoder
 * @brief 编码器处理类
 * @details 继承自SerialDevice和imu类，实现编码器数据的接收和处理
  @串口波特率默认 ： 19200
 */
class Encoder : public SerialDevice, public imu
{
public:
    /**
     * @brief 获取当前距离
     * @return float 计算得到的距离值
     */
    float get_distance(void);

    /**
     * @brief 数据接收处理函数
     * @param byte 接收到的单字节数据
     */
    void handleReceiveData(uint8_t byte);

    /**
     * @brief 构造函数
     * @param huart_ UART句柄指针
     */
    Encoder(UART_HandleTypeDef *huart_);
	  float get_absolute_distance(void);
private:
    float length = 0;           ///< 当前计算长度
    float distance = 0;         ///< 当前编码器距离
    float init_distance = 0;    ///< 初始距离
    uint32_t Encoder_conut = 0; ///< 编码器计数值
    float delta_length = 0.01;  ///< 单圈对应长度
    bool init_flag = false;     ///< 初始化标志
};

#endif