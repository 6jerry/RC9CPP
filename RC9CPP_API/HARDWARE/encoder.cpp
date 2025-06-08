/**
 * @file encoder.cpp
 * @brief 编码器模块实现文件
 * @details 实现编码器数据的接收、解析和距离计算功能
 * @version 1.0
 */

#include "encoder.h"
#include <cstdint>

/**
 * @brief 构造函数
 * @param huart_ UART句柄指针
 */
Encoder::Encoder(uint32_t can_id_, FDCAN_HandleTypeDef *hcan_) : CanDevice(hcan_, CAN_FRAME_STD, can_id_) {}

float Encoder::get_absolute_distance(void)
{

    return distance;
}
/**
 * @brief 获取当前距离
 * @return float 计算得到的距离值
 * @note 距离值为当前距离减去初始距离，最小值为0
 */
float Encoder::get_distance(void)
{
    // length = distance - init_distance;
    // if ((length) < 0)
    //{
    // length = 0;
    //}
    return distance;
}

/**
 * @brief 数据接收处理函数
 * @param byte 接收到的单字节数据
 * @details 处理编码器数据帧，解析计数值并计算距离
 * @note 数据帧格式：0xAB 0xCD [数据] ... [校验]
 */
void Encoder::can_update(uint8_t can_RxData[8])
{

    // 数据头校验
    //    if (ucRxBuffer1[0] != 0xAB || (ucRxBuffer1[1] != 0xCD && ucRxCnt1 > 1))
    //    {
    //        ucRxCnt1 = 0;
    //        return;
    //    }

    //    // 数据长度校验
    //    if (ucRxCnt1 < 10)
    //    {
    //        return;
    //    }

    // 解析编码器计数值
    Encoder_conut = can_RxData[6];
    Encoder_conut = Encoder_conut << 8;
    Encoder_conut |= can_RxData[5];
    Encoder_conut = Encoder_conut << 8;
    Encoder_conut |= can_RxData[4];
    Encoder_conut = Encoder_conut << 8;
    Encoder_conut |= can_RxData[3];

    //    // 计算圈数换算距离
    //    if (!init_flag)
    //    {
    //        init_distance = (float)Encoder_conut / 4096 * delta_length;
    //        init_flag = true;
    //    }
    //    else
    //    {
    distance = (((float)Encoder_conut / (float)1024) * delta_length) / 2.0f;
    // test_count = (float)Encoder_conut;
    //     }
}

void Encoder::send_reset()
{
     uint8_t data[8];
	   data[0] = 0x04;
     data[1] = 0x01;
	   data[2] = 0x06;
	   data[3] = 0x00;
	
	  CAN_Send(0x01,false,data,&hfdcan3);
}
