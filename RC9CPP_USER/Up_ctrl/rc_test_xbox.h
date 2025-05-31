#ifndef RC_TEST_XBOX_H
#define RC_TEST_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "xbox.h"
#include "TaskManager.h"
#include "motor.h"
#include "gpio.h"
#include "imu.h"
#include "serial_studio.h"
#include "auto_shooter.h"
#include "auto_yunball.h"

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class yun_ball_xbox : public ITaskProcessor, public xbox
{
private:
  AutoShooter *auto_shooter = nullptr;

  power_motor *lifter_motor = nullptr, *turn_motor = nullptr;

  serialStudio *serial_studio = nullptr;
  auto_yunball *yunball = nullptr;

  GPIO_TypeDef *yun_port = nullptr;
  uint16_t yun_pin = 0;

  uint8_t if_motor_start = 0, trigger_start = 0, shooter_trigger = 0, yun_trigger = 0, shoot_yunball = 0;

  uint8_t auto_mode = 0, auto_revert;
  // 拉伸状态 0静止状态
  uint8_t lifter_status = 0;
  // 俯仰状态 0静止状态 1标准俯仰
  uint8_t pithcer_status = 0;

  uint8_t auto_flag = 0;
  float lifter_speed = 0.0f, turn_speed = 0.0f, shooter_speed = 0.0f, pithcer_speed = 0.0f;

  float max_lifter_speed = 420.0f, max_turn_speed = 80.0f, max_shooter_speed = 600.0f, max_pithcer_speed = 430.0f, shoot_dis = 0.16f;

public:
  void process_data();
  yun_ball_xbox();
  void btn_scan();
  void btnconfig_init();
  void add_motor(power_motor *lifter_motor_, power_motor *turn_motor_);
  void add_trigger(uint16_t yun_pin_, GPIO_TypeDef *yun_port_);
  void add_serial_studio(serialStudio *serial_studio_);
  void add_auto_shooter(AutoShooter *auto_shooter_);
  void adjust_pitcher(float pitch_angle);
  void add_yunball(auto_yunball *yunball_);
};

#endif
#endif