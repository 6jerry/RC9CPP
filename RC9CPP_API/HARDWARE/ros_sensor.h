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

#include "transformation_of_coordinates.h"
#include "filter.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus



// 
class ros_sensor : public RC9subscriber, public imu, public KalmanFilter
{
public:
    tf tf_;
    imu * imu_ = nullptr;
    //信息储存结构体
    struct{
        Vector2D world_pos;
        float yaw_angle = 0.0f;
    } ros_radar_loaction;
    struct{
        Vector2D  vertial_plane_deviation; //竖直平面偏差(像素值)，x,y需要转换(x -> yaw, y -> pitch)
    } camera_info;

    Vector2D real_radar_world_pos;
    Vector2D map_origin; //映射后原点
    bool map_origin_init_flag = false; //原点映射标志位
    bool relocate_flag = false; //是否开启重定位标志
	
    ros_sensor();
    // 信息获取接口
    Vector2D get_world_pos() override;
    //float get_yaw_angle() override;
    float get_heading() override;
	float get_yaw_rad() override;
    void relocate_imu();
    void add_recolate_imu(imu * imu_);
public:
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
};

#endif
#endif