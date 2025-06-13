#ifndef YUNBALL_TEST_XBOX_H
#define YUNBALL_TEST_XBOX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "robot_chassis.h"
#include "RC9Protocol.h"
#include "xbox.h"
#include "M3508.h"
#include "auto_yunball.h"
#include "TaskManager.h"

#ifdef __cplusplus
}
#endif

/*typedef union {
    float value;
    uint8_t bytes[sizeof(float)];
} FloatUnion;*/

#ifdef __cplusplus
class yunball_test_xbox : public xbox, public ITaskProcessor
{
private:
    m3508p *turn_motor;
    auto_yunball *auto_yunball_ptr;
    //FloatUnion float_data;
    GPIO_TypeDef *turn_port;
    uint16_t turn_pin;

    float max_turn_speed = 70.0f;
public:
    uint8_t mode_flag = 2, start_flag = 0, lb_flag = 0, rb_flag = 0, cnt_flag = 0, up_flag = 0, down_flag = 0, left_flag = 0, right_flag = 0;

    yunball_test_xbox();
    void process_data();
    void btn_scan();
    void btnconfig_init();

    void add_io(GPIO_TypeDef *turn_port_, uint16_t turn_pin_);
    void add_motor(m3508p *turn_motor_);
    void add_autoyunball(auto_yunball *auto_yunball_);
    void not_start();
    void mode_0();
    void mode_1();

    void yunball();
    void putball();
    void set_claw(bool if_open);
    void set_lift(bool if_up);
    void set_push(bool if_push);
};
#endif

#endif /* YUNBALL_TEST_XBOX_H */