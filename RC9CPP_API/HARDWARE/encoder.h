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
#include "fdcan_device.h"
#include "math.h"
#include "imu.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @class Encoder
 * @brief 编码器处理类
 * @details 继承自CanDevice和imu类，实现编码器数据的接收和处理
 */
class Encoder : public CanDevice, public imu
{
public:
    /**
     * @brief 获取当前距离
     * @return float 计算得到的距离值
     */

    Encoder(uint32_t can_id_, FDCAN_HandleTypeDef *hcan_, float gear_);

    void can_update(uint8_t can_RxData[8]) override;

    // 外部使用接口
    float get_distance(void) override;
    float get_rpm(void);
    float get_angle(void) { return angle; }         ///< 获取当前角度
    float get_all_angle(void) { return all_angle; } ///< 获取累计总角度

    // 编码器设置函数
    //*********************** */
    // 重定位，设为五圈值
    void send_reset();
    // 设置顺时针
    void set_clockwise();
    // 设置逆时针
    void set_anti_clockwise();

    void set_dis();

    //*********************** */
private:
    uint8_t sampling_time = 10; ///< 采样时间
    float rpm;                  ///< 转速 转/分钟
    float gear;                 ///< 齿轮减速比
    float distance = 0;         ///< 当前编码器距离
    int init_flag = 0;
    uint32_t Last_Encoder_conut = 0; ///< 上一次编码器计数值
    uint32_t Encoder_conut = 0;      ///< 编码器计数值
    float delta_length = 0.01;       ///< 单圈对应长度
    float all_angle = 0.0f;          ///< 累计总角度
    float angle = 0.0f;              ///< 当前角度
};

#endif