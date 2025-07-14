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

class photogate_shoot_down : public GPIODevice
{
private:
    power_motor *motor1;
    power_motor *motor2;

public:
    void set_motors(power_motor *m1, power_motor *m2)
    {
        motor1 = m1;
        motor2 = m2;
    }
    float rpm1 = 0.0f;
    float rpm2 = 0.0f;
    int flag = 0;
    int cont = 0;
    photogate_shoot_down();
    float max_speed = 0.0f;
    void handleInterrupt() override;
    void add_io_interrupt(GPIO_TypeDef *port, uint16_t pin) override;
};
class photogate_shoot : public GPIODevice
{
private:
    power_motor *motor1;
    power_motor *motor2;

public:
    void set_motors(power_motor *m1, power_motor *m2)
    {
        motor1 = m1;
        motor2 = m2;
    }
    float rpm1 = 0.0f;
    float rpm2 = 0.0f;
    int flag = 0;
    int cont = 0;
    photogate_shoot();
    float max_speed = 0.0f;
    void handleInterrupt() override;
    void add_io_interrupt(GPIO_TypeDef *port, uint16_t pin) override;
};

class R3_xbox : public xbox_debug_base, public chassis_user
{
private:
public:
    void calc_error();
    R3_xbox(imu *imu_ptr_);
    power_motor *shoot_motor_1 = nullptr;
    power_motor *shoot_motor_2 = nullptr;
    Encoder *encoder = nullptr;

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
    void xbox_on() override;
    void not_start() override;
    float rpm1 = 0.0f;
    float rpm2 = 0.0f;
    uint8_t test_flag = 0;
    uint32_t last_tick = 0;
    photogate_shoot gate;
    photogate_shoot_down gate_down;
    float target_rpm = 100.0f;
    float max_speed = 0.0f;
    float offest2 = 0.0f;
    void init(power_motor *shoot_motor_1_, power_motor *shoot_motor_2_, Encoder *encoder_);

    float a=27.4543f;
    float b=0.9230f;
    float c=1054.6237f;
    float calc_rpm(float dis);
    float max_rpm= 2500.0f; //最大转速
    void Shoot(float rpm);
};

#endif
#endif
