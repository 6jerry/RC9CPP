#ifndef YUNBALL_TEST_XBOX_H
#define YUNBALL_TEST_XBOX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "robot_chassis.h"
#include "RC9Protocol.h"
#include "xbox.h"
#include "motor.h"
#include "auto_yunball.h"
#include "TaskManager.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class yunball_test_xbox : public xbox, public ITaskProcessor, public chassis_user
{
private:
    power_motor *turn_motor;
    auto_yunball *auto_yunball_ptr;
    GPIO_TypeDef *lift_port, *claw_port, *push_port;
    uint16_t lift_pin, claw_pin, push_pin;

    Vector2D max_target_robot_vel; // 最大目标速度
    //Vector2D track_point[7] = {};
    float lock_yaw = 0.0f;
    imu *imu_ptr; // 指向imu类的指针

    float max_turn_speed = 70.0f;
    uint8_t point_cnt = 0;
public:
    uint8_t mode_flag = 2, start_flag = 0, lb_flag = 0, rb_flag = 0, cnt_flag = 0, up_flag = 0, down_flag = 0, left_flag = 0, right_flag = 0, btn_start_flag = 0;

    yunball_test_xbox(imu *imu_ptr_);
    void process_data();
    void btn_scan();
    void btnconfig_init();

    void add_io(GPIO_TypeDef *lift_port_, uint16_t lift_pin_,  GPIO_TypeDef *claw_port_, uint16_t claw_pin_, GPIO_TypeDef *push_port_, uint16_t push_pin_);
    void add_motor(power_motor *turn_motor_);
    void add_autoyunball(auto_yunball *auto_yunball_);
    void not_start();
    void mode_0();
    void mode_1();
    void mode_2();

    void yunball();
    void putball();
    void set_claw(bool if_open);
    void set_lift(bool if_up);
    void set_push(bool if_push);
};
#endif

#endif /* YUNBALL_TEST_XBOX_H */