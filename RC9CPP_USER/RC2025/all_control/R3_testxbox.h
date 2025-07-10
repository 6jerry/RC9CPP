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
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class photogate_shoot : public GPIODevice
{

public:
    int flag = 0;
    photogate_shoot();

    void handleInterrupt() override;
    void add_io_interrupt(GPIO_TypeDef *port, uint16_t pin) override;
    bool is_finish();
};

class R3_xbox : public xbox_debug_base, public chassis_user
{
private:
public:
    void calc_error();

    power_motor *shoot_motor_1 = nullptr;
    power_motor *shoot_motor_2 = nullptr;
    Encoder *encoder = nullptr;
    float c = 0.0f;
    float rpm = 1000.0f;
    float move_rpm = 1000.0f;

    imu *imu_ptr, *ros_imu;                                              // 指向imu类的指针
    Vector2D center_point, tan_dir, nor_dir, robot_point, nor_dir_robot; // 圆心坐标
    float dis_2_center = 0.0f, center_heading = 0.0f, nor_speed = 0.0f;  // 半径
    float pass_correct_distance = 0.30f;                                 // 修正距离
    float offest = 0.0f;
    float limit_yaw_error = 0.4f, limit_yaw_speed = 1.0f;
    void mode_1() override;
    void mode_2() override;
    void mode_3() override;
    void not_start() override;

    photogate_shoot gate;
    float target_rpm = 1000.0f;

    void init(power_motor *shoot_motor_1_, power_motor *shoot_motor_2_, Encoder *encoder_);
};

#endif
#endif
