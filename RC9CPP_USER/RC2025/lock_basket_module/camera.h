#ifndef CAMERA_H
#define CAMERA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "TaskManager.h"
#include <arm_math.h>
#include "PID.h"
#include "ros_sensor.h"
#include "err_code.h"


#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class Camera : public error_check, public RC9subscriber
{
private:     
    float lock_vol;
    pid lock_basket_pid; // 锁框

    ///错误码
    device_id err_id = ERR_DEVICE_CAMERA; //0x08


public:
    struct{
        Vector2D vertial_plane_deviation; // 竖直平面偏差(像素值)，x,y需要转换(x -> yaw, y -> pitch)
    } camera_info;
    bool gaze_flag = true;//视野是否丢失标志

    Camera();
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;

    float lock_basket_vol ();
    
    ///错误码
    err_code check_error() override;
   
    
};

#endif
#endif