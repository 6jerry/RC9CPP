#ifndef __R3_TEST_XBOX_H__
#define __R3_TEST_XBOX_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include "debug_xbox.h"

#include "RC9Protocol.h"
#include "motor.h"
#include "IO_Interrupt.h"
#include "robot_chassis.h"
#include "encoder.h"
#include "auto_yunball_R3.h"
#include "R3_shooter.h"
#include "camera.h"
#include "BasketCalibrator.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class R3_xbox : public xbox_debug_base, public chassis_user
{

public:
    AutoYunballR3 *auto_yunball_ptr = nullptr;
    R3Shooter *shooter = nullptr;
    imu *imu_ptr, *ros_imu;
    CameraOperation *camera_ops; // 导入相机操作的指针
    BasketCalibrator basketCalibrator;

    float move_rpm = 1000.0f;                                              // 指向imu类的指针
    Vector2D center_point, nor_dir, robot_point, nor_dir_robot; // 圆心坐标
    float dis_2_center = 0.0f, center_heading = 0.0f;  // 半径
    float dis_2_robot = 0.0f, robot_heading = 0.0f;
    float limit_yaw_error = 0.1f, limit_yaw_speed = 1.0f;
    float target_rpm = 300.0f;
    float shoot_dis = 0.0f;

	float test_v = 0.0f;
        
    R3_xbox(imu *imu_ptr_,imu *ros_imu_ptr_, CameraOperation *camera_ops_);
    void mode_0() override;
    void mode_1() override;
    void mode_2() override;
    void mode_3() override;
    void for_btnY();
    void xbox_on() override;
    void not_start() override;
    void xbox_share() override;
    void add_autoyunball(AutoYunballR3 *auto_yunball_);
    void add_R3shooter(R3Shooter *shooter_); 
    void calc_error();
    void calc_robot_point(); 
};

#endif
#endif
