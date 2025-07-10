/**
 * @file encoder.cpp
 * @brief 编码器模块实现文件
 * @details 实现编码器数据的接收、解析和距离计算功能
 * @version 2.0
 */

#include "encoder.h"
#include <cstdint>

/**
 * @brief 构造函数
 * @param can_id_ CANID
 * @param hcan_   CAN总线句柄
 * @param gear_   齿轮比
 * @param mode_   编码器模式
 */
Encoder::Encoder(uint32_t can_id_, FDCAN_HandleTypeDef *hcan_, float gear_) : CanDevice(hcan_, CAN_FRAME_STD, can_id_)
{
  gear = gear_;
}

/**
 * @brief 获取当前距离
 * @return float 计算得到的距离值
 */
float Encoder::get_distance(void)
{
  return distance;
}
float Encoder::get_rpm(void)
{
  return rpm;
}

/**
 * @brief 数据接收处理函数
 * @param byte 接收到的单字节数据
 * @details 处理编码器数据帧，解析计数值并计算距离
 */
void Encoder::can_update(uint8_t can_RxData[8])
{

  // 解析编码器计数值
  Encoder_conut = can_RxData[6];
  Encoder_conut = Encoder_conut << 8;
  Encoder_conut |= can_RxData[5];
  Encoder_conut = Encoder_conut << 8;
  Encoder_conut |= can_RxData[4];
  Encoder_conut = Encoder_conut << 8;
  Encoder_conut |= can_RxData[3];

  if (init_flag == 0)
  {
    Last_Encoder_conut = Encoder_conut;
    init_flag = 1;
  }
  else if (init_flag == 1)
  {
    float error = ((float)Encoder_conut - (float)Last_Encoder_conut) / 1024.0f;
    rpm = (error /0.1 ) * 60/gear;
		Last_Encoder_conut = Encoder_conut;
  }

  float Encoder_delta = (float)Encoder_conut / 1024.0f;
  Encoder_delta = Encoder_delta - 5.0f;
  // 计算距离
  distance = Encoder_delta * delta_length / gear;
  all_angle = (Encoder_delta / gear) * 360.0f;            // 累计总角度
  angle = all_angle - (int)(all_angle / 360.0f) * 360.0f; // 当前角度
}

void Encoder::send_reset()
{

  uint8_t data[8];
  data[0] = 0x04;
  data[1] = can_id_;
  data[2] = 0x0F;
  data[3] = 0x01;

  CAN_Send(can_id_, false, data, hcan_);
}

void Encoder::set_clockwise()
{

  uint8_t data[8];
  data[0] = 0x04;
  data[1] = can_id_;
  data[2] = 0x07;
  data[3] = 0x00;

  CAN_Send(can_id_, false, data, hcan_);
}
void Encoder::set_anti_clockwise()
{
  uint8_t data[8];
  data[0] = 0x04;
  data[1] = can_id_;
  data[2] = 0x07;
  data[3] = 0x01;

  CAN_Send(can_id_, false, data, hcan_);
}

void Encoder::set_dis()
{

  uint8_t data[8];
  data[0] = 0x04;
  data[1] = can_id_;
  data[2] = 0x04;
  data[3] = 0xAA;

  CAN_Send(can_id_, false, data, hcan_);
}
