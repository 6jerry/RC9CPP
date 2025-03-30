#ifndef LJH_XBOX_TEST_H
#define LJH_XBOX_TEST_H

#ifdef __cplusplus
extern "C" {
#endif
#include "xbox.h"
#include "TaskManager.h"
#include "gpio.h"
#include "motor.h"
#include "robot_chassis.h"
#ifdef __cplusplus
}
#endif

enum yunball_step
{
    reset,
    catch_point,
    throw_point,
    turn_point,
    turn_to,
    turn_back
};


#ifdef __cplusplus
class xbox_controller : public ITaskProcessor, public xbox
{
public:
    xbox_controller();
    void process_data();
    void btn_scan();
    void add_motor(power_motor* motor1, power_motor* motor2, power_motor* motor3, power_motor* motor4);
    void add_trigger(GPIO_TypeDef *port_1, uint16_t pin_1, GPIO_TypeDef *port_2, uint16_t pin_2,GPIO_TypeDef *port_3, uint16_t pin_3, GPIO_TypeDef *port_4, uint16_t pin_4);
    void auto_yunball();
    void auto_yunball_pro();
    void control_shooter_speed(float speed);

private:
    uint8_t test1 = 0, test2 = 0, test3 = 0, test4 = 0;
    power_motor* motor1 = nullptr, *motor2 = nullptr, *motor3 = nullptr, *motor4 = nullptr;

    GPIO_TypeDef* port_1 = nullptr, *port_2 = nullptr, *port_3 = nullptr, *port_4 = nullptr;
    uint16_t pin_1 = 0, pin_2 = 0, pin_3 = 0, pin_4 = 0;
    float max_lifter_speed = 1500.0f,max_turn_speed = 80.0f, max_pithcer_speed = 430.0f,max_shooter_speed = 600.0f;
    yunball_step step = reset;
	int time_cnt = 0;
};
#endif

#endif /* LJH_XBOX_TEST_H */