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
class yunball_test_xbox : public xbox, public ITaskProcessor, public chassis_user
{
private:
    m3508p *turn_motor;
    auto_yunball *auto_yunball_ptr;
    //FloatUnion float_data;
    GPIO_TypeDef *turn_port;
    uint16_t turn_pin;

    imu *imu_ptr;

    float max_turn_speed = 70.0f;
	
    //Vector2D t_points[7] = {{4.52f, 2.18f}, {3.62f, 2.97f}, {3.61f, 4.81f},\
    //{5.69f, 4.85f}, {7.75f, 4.90f}, {7.81f, 3.06f}, {6.96f, 2.27f}};

	
	Vector2D t_points[7] = {{4.32f, 1.22f}, {3.55f, 2.12f}, {3.55f, 3.76f},\
    {5.49f, 3.76f}, {7.70f, 3.76f}, {7.70f, 1.85f}, {6.76f, 1.22f}};
	
// 	Vector2D t_points[7] = {{4.26f, 1.31f}, {6.81f, 1.39f}, {7.56f, 1.93f},\
//    {7.56f, 3.54f}, {5.43f, 3.54f}, {3.76f, 3.54f}, {3.76f, 1.80f}};
		
    float change_dis = 0.01f;
	float Vy=0.0f,Vx=0.0f,zhuan_su = 0;
public:
    uint8_t mode_flag = 2, start_flag = 0, lb_flag = 0, rb_flag = 0, cnt_flag = 0, up_flag = 0, down_flag = 0, left_flag = 0, right_flag = 0,yun_flag = 1;
    yunball_test_xbox(imu *imu_ptr_);
    void process_data();
    void btn_scan();
    void btnconfig_init();

    void add_io(GPIO_TypeDef *turn_port_, uint16_t turn_pin_);
    void add_motor(m3508p *turn_motor_);
    void add_autoyunball(auto_yunball *auto_yunball_);
    void not_start();
    void mode_0();
    void mode_1();
    void mode_2();
    void mode_3();
    void mode_4();
    void xbox_on();
    void btnXBOX_callback();

    void yunball();
    void putball();
    void set_claw(bool if_open);
    void set_lift(bool if_up);
    void set_push(bool if_push);
};
#endif

#endif /* YUNBALL_TEST_XBOX_H */