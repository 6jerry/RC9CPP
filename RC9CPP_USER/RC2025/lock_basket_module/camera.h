#ifndef CAMERA_H
#define CAMERA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "TaskManager.h"
#include <arm_math.h>
#include "PID.h"
#include "robot_chassis.h"
#include "ros_sensor.h"
#include "err_code.h"


#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

enum CameraMode
{
    camera_suspend, // 挂起
    camera_start, // 开始
    camera_finish, // 完成
};

class Camera : public error_check, public RC9subscriber
{
private:     
    bool gaze_flag = true;//视野是否丢失标志

public:    
    struct{
        Vector2D vertial_plane_deviation; // 竖直平面偏差(像素值)，x,y需要转换(x -> yaw, y -> pitch)
    } camera_info;

    Camera();
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
    
    ///错误码
    err_code check_error() override;
};

class CameraOperation :  public chassis_user, public ITaskProcessor
{
//需要添加底盘
private:     
    float lock_vol;
    pid lock_basket_pid; // 锁框  //三分Y50
    CameraMode camera_mode = camera_suspend;    //初始化为挂起模式

public:
    Camera *camera_ptr= nullptr;
    bool camera_ready = false; //相机是否瞄准完成标志位, 注：需要手动复位
    float camera_Y = 0.0f; //记录相机瞄准完成时的Y值

	float deadlock = 8.0f; //死区
	float decelerate_x = 50.0f; 
	float decelerate_speed = 0.06f;
    //10, 50, 0.05

    CameraOperation(Camera *camera_ptr_);
    float lock_basket_vol ();
    void camera_on();
    void camera_off();      //强制结束挂起
    void process_data();    //适用于相机的特定频率
};

#endif
#endif