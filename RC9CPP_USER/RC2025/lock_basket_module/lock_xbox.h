#ifndef LOCK_XBOX_H
#define LOCK_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "debug_xbox.h"
#include "TaskManager.h"
#include <arm_math.h>
#include "RC9Protocol.h"
#include "PID.h"
#include "imu.h"
#include "robot_chassis.h"
#include "ros_sensor.h"
#include "auto_shooter.h"


#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class lock_xbox : public xbox_debug_base, public chassis_user
{
private:     

    void mode_2() override;
    void mode_1() override;
    void xbox_on() override;

    imu *imu_ptr; // 指向imu类的指针
    ros_sensor *ros_ptr; // 导入雷达的指针
    AutoShooter* auto_shooter_ptr;//导入射球实例

    Vector2D center_point, tan_dir, nor_dir;    //圆心坐标
    float dis_2_center = 0.0f, center_heading = 0.0f, nor_speed = 0.0f; // 半径

    float shoot_dis;

public:
    lock_xbox(imu *imu_ptr_, ros_sensor *ros_ptr_);
    void add_AutoShooter(AutoShooter *auto_shooter_);
    void calc_error();

    float lock_vol;
    pid lock_basket; // 锁框
    
    float dis;
};

#endif
#endif