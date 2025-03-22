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

#ifdef __cplusplus
class xbox_controller : public ITaskProcessor, public xbox
{
public:
    xbox_controller();
    void process_data();
    void btn_scan();
    void add_motor(power_motor* motor1, power_motor* motor2, power_motor* motor3, power_motor* motor4);
    void add_trigger(GPIO_TypeDef *port_1, uint16_t pin_1, GPIO_TypeDef *port_2, uint16_t pin_2);

private:
    uint8_t test1 = 0, test2 = 0, test3 = 0, test4 = 0;
    power_motor* motor1 = nullptr, *motor2 = nullptr, *motor3 = nullptr, *motor4 = nullptr;
    GPIO_TypeDef* port_1 = nullptr, *port_2 = nullptr;
    uint16_t pin_1 = 0, pin_2 = 0;
    float max_lifter_speed = 420.0f,max_turn_speed = 80.0f, max_pithcer_speed = 430.0f,,max_shooter_speed = 600.0f;
};
#endif

#endif /* LJH_XBOX_TEST_H */