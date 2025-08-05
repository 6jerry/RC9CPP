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
    camera_start,   // 开始
    camera_keeping, // 锁定中
    camera_finish,  // 完成
};

class Camera : public error_check, public RC9subscriber
{
private:
public:
    struct
    {
        Vector2D vertial_plane_deviation; // 竖直平面偏差(像素值)，x,y需要转换(x -> yaw, y -> pitch)
    } camera_info;

    bool gaze_flag = true; // 视野是否丢失标志

    Camera();
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;

    /// 错误码
    err_code check_error() override;
};

class CameraOperation : public chassis_user, public ITaskProcessor
{
    // 需要添加底盘
private:
    float lock_vol;
    pid lock_basket_pid;                     // 锁框  //三分Y50
    CameraMode camera_mode = camera_suspend; // 初始化为挂起模式
    float count = 0;

public:
    Camera *camera_ptr = nullptr;
    bool camera_ready = false; // 相机是否瞄准完成标志位, 注：需要手动复位
    float camera_Y = 0.0f;     // 记录相机瞄准完成时的Y值

    float deadlock = 8.0f; // 死区
    float decelerate_x = 50.0f;
    float decelerate_speed = 0.06f;
    // 10, 50, 0.05

    float camera_X = 0.0f;
    float Tick = 0.0f; // 校准最大时间限制

    // 手动偏置防止相机被撞歪
    float offest_x = -5.0f; // 相机横向定位偏置(通常加减5个像素点)，给大往右,注：除非特定原因不要改变此参数
    float offest_out = 0.0f;
    // 相机输出偏置(通常加减10转) 注：此处只是表明有此功能具体在shoot文件里面有定义变量camera_offset

    //修正记录
    //8.3 目前发射偏右为保证最佳校准效率，机械调整到+5为0点

    CameraOperation(Camera *camera_ptr_);
    float lock_basket_vol();
    void camera_on();
    void camera_off();   // 强制结束挂起
    void process_data(); // 适用于相机的特定频率
    bool time_delay();   // 确保准确帧都处于死区内
    bool check();        // 用于检查相机是否可以使用
};

#endif
#endif