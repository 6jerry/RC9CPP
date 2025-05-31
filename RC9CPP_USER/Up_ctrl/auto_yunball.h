#ifndef AUTO_YUNBALL_H
#define AUTO_YUNBALL_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "SuperPID.h"
#include "motor.h"
#include "PID.h"
#include "SuperPID.h"
#include "gpio.h"
#include "imu.h"

#include "xbox.h"
#include "debug_xbox.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

enum yunball_mode
{
    yunball_init_locate,
    yunball_remote_control,
    yunball_standby,
    yunball_move_2_catch_point,
    yunball_move_2_throw_point,
    yunball_move_2_turn_point,
    yunball_turn_2_throw_point,
    yunball_turn_back
};

class auto_yunball : public ITaskProcessor
{
private:
    GPIO_TypeDef *locate_sensor_port = nullptr, *ball_sensor_port = nullptr, *claw_port = nullptr;
    uint16_t locate_sensor_pin = 0, ball_sensor_pin = 0, claw_pin = 0;

    power_motor *lift_motor = nullptr;
    power_motor *turn_motor = nullptr;
    yunball_mode workmode = yunball_standby;

    void scan_sensor();

    uint8_t locate_flag = 1, ball_flag = 1;

    float catch_ball_dis = 360.0f, throw_ball_dis = 798.0f;

    float max_catch_speed = 1500.0f, max_catch_acc = 2500.0f, max_catch_dec = 2500.0f, final_catch_speed = 0.0f;
    float max_throw_speed = 1000.0f, max_throw_acc = 1400.0f, max_throw_dec = 1400.0f, final_throw_speed = 600.0f;

    void system_init();

    void move_2_catch_point();

    void move_2_throw_point();

    void move_2_turn_point();

    void turn_2_throw_point();

    void turn_back();

    uint32_t time_cnt = 0, time_flag = 0;;

public:
    void
    process_data();
    void claw_open();
    void claw_close();
    void add_motor(power_motor *lift_motor_, power_motor *turn_motor_);
    void add_io(GPIO_TypeDef *locate_sensor_port_, uint16_t locate_sensor_pin_, GPIO_TypeDef *ball_sensor_port_, uint16_t ball_sensor_pin_, GPIO_TypeDef *claw_port_, uint16_t claw_pin_);

    void start_multi_yun();

    void stop();
};

class auto_yunball_xbox : public xbox_debug_base
{
private:
    auto_yunball *yunball = nullptr;
    power_motor *lifter_motor = nullptr;

public:
    void not_start() override;

    void mode_2() override;
    void mode_3() override;

    void add_yunball(auto_yunball *yunball_);
    void add_lifter(power_motor *lifter_motor_);
};

#endif
#endif