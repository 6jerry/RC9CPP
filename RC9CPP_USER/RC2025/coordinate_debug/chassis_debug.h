#ifndef CHASSIS_DEBUG_H
#define CHASSIS_DEBUG_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "debug_xbox.h"
#include "robot_chassis.h"
#include "RC9Protocol.h"
#include "imu.h"
#include "auto_shooter.h"
#include "auto_yunball.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class chassis_adjust_xbox : public xbox_debug_base, public chassis_user
{
public:
  Vector2D max_target_robot_vel; // 最大目标速度

  auto_yunball *auto_yunball_ptr; 
  AutoShooter *auto_shooter;
  imu *imu_ptr, *ros_imu; // 指向imu类的指针
  CameraOperation *camera_ops; // 导入相机操作的指针

  float debug_dis = 0.15f;
  Vector2D center_point, nor_dir, robot_point, nor_dir_robot; // 圆心坐标
  float dis_2_center = 0.0f, center_heading = 0.0f;  // 半径
  float dis_2_robot = 0.0f, robot_heading = 0.0f;

  float limit_yaw_error = 0.1f, limit_yaw_speed = 1.0f;
  float offest = 0.0f;
  float pass_correct_distance = 0.25f; // 传球修正距离
  float shoot_dis = 0.0f;

  chassis_adjust_xbox(imu *imu_ptr_, imu *ros_imu_, CameraOperation *camera_ops_);
  void add_AutoShooter(AutoShooter *auto_shooter_);
  void add_autoyunball(auto_yunball *auto_yunball_);
  void calc_error();
  void not_start() override;
  void mode_2() override;
  void mode_1() override;
  void mode_3() override;
  void mode_4() override;
  void xbox_on() override;
  void xbox_share() override;
  void btnY_callback() override;
  bool calc_robot_point();
};

#endif
#endif