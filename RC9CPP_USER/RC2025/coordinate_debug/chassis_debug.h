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
  float lock_yaw = 0.0f;
  imu *imu_ptr, *ros_imu; // 指向imu类的指针

  float debug_dis = 0.12f;
  Vector2D center_point, tan_dir, nor_dir;                            // 圆心坐标
  float dis_2_center = 0.0f, center_heading = 0.0f, nor_speed = 0.0f; // 半径
  AutoShooter *auto_shooter;

  uint8_t test_flag = 0, putball = 0;

  float limit_yaw_error = 0.4f, limit_yaw_speed = 1.0f;

public:
  void add_AutoShooter(AutoShooter *auto_shooter_);
  void add_autoyunball(auto_yunball *auto_yunball_);
  void calc_error();
  chassis_adjust_xbox(imu *imu_ptr_);
  void not_start() override;
  void mode_2() override;
  void mode_1() override;
  void mode_3() override;
  void mode_4() override;
  void xbox_on() override;
};

#endif
#endif