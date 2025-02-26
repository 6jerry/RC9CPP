#ifndef SHOOT_XBOX_H
#define SHOOT_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "xbox.h"
#include "motor.h"
#include "TaskManager.h"
#include "chassis.h"
#include "gpio.h"
#include <arm_math.h>
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class shoot_xbox : public xbox, public ITaskProcessor
{
private:
    uint8_t speed_level = 1; // 0---低速，1---中速，2---高速

    uint8_t trigger_flag = 0; // 发射按钮状态

public:
    power_motor *shooter = nullptr; // 控拉弹簧发射的电机
    power_motor *lifter = nullptr;
    power_motor *pitcher = nullptr;     // 俯仰角电机
    chassis *control_chassis = nullptr; // 底盘指针

    shoot_xbox(power_motor *shooter_, power_motor *pitch, power_motor *lifter_, chassis *control_chassis_);
    GPIO_TypeDef *trigger_port = nullptr;
    uint16_t trigger_pin = 0;

    void sbtnconfig_init(); // 按键初始化
    void process_data();
    void btn_scan(); // 扫描并获取按键状态
    float mapsum = 0.0f, MAX_RPM = 0.0f;

    void config_trigger(GPIO_TypeDef *port, uint16_t pin); // 配置发射按钮
};

#endif
#endif
