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
#include "TaskManager.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

//
// class ros_sensor : public RC9subscriber, public imu, public ITaskProcessor
class ros_sensor : public RC9subscriber, public imu
{
public:
    tf tf_;
    imu *imu_ = nullptr;
	static const uint8_t msg_num = 6;
	First_Order_BPF f_BFP[msg_num];
	KalmanFilter kalman[msg_num];
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
    Vector2D real_radar_world_pos; // 映射后坐标
	float my_r = 0.2889019f; //  雷达半径
	float my_rad = 53.17476f * 0.017453f; //  雷达初始弧度
	Vector2D map_inverse_relocate_pos; // 逆映射坐標
    bool relocate_flag = false; //  是否开启重定位标志
	bool get_zero_flag = false; //  雷达有时会发送同时0的值，需要过滤
	bool rst_radar = false; // 雷达重启标志位（debug专用）
    bool if_active = true; //  雷达是否活跃
    ros_sensor();
    // 信息获取接口
    Vector2D get_world_pos() override;
    // float get_yaw_angle() override;
    float get_heading() override;
    float get_yaw_rad() override;
    void imu_rst() override;
    void relocate_imu();
    void stop_relocate();
    void add_recolate_imu(imu *imu_);

    // void process_data() override;
public:
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;

    
};

#endif
#endif
