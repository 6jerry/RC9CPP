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
    imu *imu_ = nullptr;
    // 信息储存结构体
    struct
    {
        Vector2D vertial_plane_deviation; // 竖直平面偏差(像素值)，x,y需要转换(x -> yaw, y -> pitch)
    } camera_info;
    struct
    {
        Vector2D world_pos;
        float yaw_angle = 0.0f;
    } ros_radar_loaction;
    Vector2D real_radar_world_pos;                   // 映射后坐标
    Vector2D map_inverse_relocate_pos;               // 逆映射坐標
    Vector2D map_plot = Vector2D(-0.39f, -0.02428f); // 偏移点坐标
    bool relocate_flag = false;                      // 是否开启重定位标志
    bool get_zero_flag = false;                      // 雷达有时会发送同时0的值，需要过滤

    ros_sensor();
    // 信息获取接口
    Vector2D get_world_pos() override;
    // float get_yaw_angle() override;
    float get_heading() override;
    float get_yaw_rad() override;
    void relocate_imu();
    void add_recolate_imu(imu *imu_);

    void imu_rst() override;

public:
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;

    float delta_w_x = 0.0f, delta_w_y = 0.0f, w_x_sum = 0.0f, w_y_sum = 0.0f, now_x = 0.0f, last_x = 0.0f, now_y = 0.0f, last_y = 0.0f; // 逆变换后坐标

    float origin_x = 0.0f, origin_y = 0.0f;
};

#endif
#endif