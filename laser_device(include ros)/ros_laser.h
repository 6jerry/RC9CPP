#ifndef ROS_LASER_H
#define ROS_LASER_H

#include "Vector2D.h"
#ifdef __cplusplus
extern "C"
{
#endif
#include "RC9Protocol.h"
#include "Serial_device.h"
#include <math.h>
#include "imu.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus



// 
class ros_laser : public RC9subscriber, public imu
{
public:
    //信息储存结构体
    struct{
        Vector2D world_pos;
        float yaw_angle = 0.0f;
    } ros_laser_loaction;

    float center_offset = 0.35;
    float angle_offset = 0.0f; // 360 degrees
    float previous_world_pos_x = 0.0f;
    float previous_world_pos_y = 0.0f;
    float previous_yaw_angle = 0.0f;
    // 信息获取接口
    Vector2D get_world_pos() override;
    //float get_yaw_angle() override;
    float get_heading() override;
	float get_yaw_rad() override;

public:
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
};

#endif
#endif