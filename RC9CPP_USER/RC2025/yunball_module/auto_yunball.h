#ifndef AUTO_YUNBALL_H
#define AUTO_YUNBALL_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "SuperPID.h"
#include "M3508.h"
#include "PID.h"
#include "SuperPID.h"
#include "gpio.h"
#include "auto_shooter.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

enum state_flag
{
    static_flag,
    yunball_flag,
    putball_flag,
    stop_flag,
    emergency_stop,
    double_yunball_flag
};

typedef union
{
    float value;
    uint8_t bytes[sizeof(float)];
} FloatUnion;

class auto_yunball : public ITaskProcessor
{
private:
    GPIO_TypeDef *claw_port, *push_port, *turn_port;
    uint16_t claw_pin, push_pin, turn_pin;
    float get_speed_turn = 0.0f;
    float get_speed_lift = 0.0f;
    float max_turn_speed = 70.0f;
    float max_lift_speed = 30.0f;

    void set_claw(bool if_open);
    void set_push(bool if_push);
    void double_yunball();
    void yunball();
    void putball();
    m3508p *turn_motor, *lift_motor;
    state_flag flag = static_flag;
    AutoShooter *shooter;

public:
    float liftTo_dis = -13.0f;
    float liftBask_dis = -0.8f;
    float turnTo_angle = 180.0f; // 旋转角度
    float turnBack_angle = 95.0f; // 旋转回转角度
    float turn_deadzone = 6.0f;  // 死区
    float lift_deadzone = 0.5f;  // 死区
    float shooter_lift = 0.15f;   // 皮筋拉伸量
    uint8_t mode_flag = 2, start_flag = 0, lb_flag = 0, rb_flag = 0, cnt_flag = 0, up_flag = 0, down_flag = 0, left_flag = 0, right_flag = 0, emergency_stop_flag = 0;
    volatile int32_t emergency_semaphore = 0; // 新增信号量计数器

    void process_data();
    void add_motor(m3508p *turn_motor_, m3508p *lift_motor_);
    void add_io(GPIO_TypeDef *claw_port_, uint16_t claw_pin_, GPIO_TypeDef *push_port_, uint16_t push_pin_, GPIO_TypeDef *turn_port_, uint16_t turn_pin_);
    void add_shooter(AutoShooter *shooter_);

    void control_turn_motor(float speed_); // 外部控制接口
    void control_lift_motor(float speed_);
    void control_claw(bool if_open);
    void control_push(bool if_push);

    bool start_yunball();
    bool start_putball();
    bool start_double_yunball();
    void stop();

    bool if_is_finish(); // 查看机构是否繁忙
};

#endif
#endif