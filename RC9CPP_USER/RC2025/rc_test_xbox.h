#ifndef RC_TEST_XBOX_H
#define RC_TEST_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "xbox.h"
#include "TaskManager.h"
#include "motor.h"
#include "gpio.h"
#include "imu.h"
#include "encoder.h"
#include "serial_studio.h"
#include "wit_gyro.h"
#include "TrapezoidalPlanner.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class yun_ball_xbox : public ITaskProcessor, public xbox
{
private:
    TrapezoidalPlanner1D planer;

    uint8_t plan_flag = 0;
    // 梯形规划参数
    float max_acc = 200.0f, max_dcc = 200.0f,
          max_speed = 1000.0f, inital_speed = 400.0f, final_speed = 250.0f;

    power_motor *lfter_motor = nullptr, *turn_motor = nullptr, *shooter_motor = nullptr, *pithcer_motor = nullptr;

    Encoder *encoder = nullptr;
    wit_gyro *wit_imu = nullptr;
    serial_studio *test_port = nullptr;

    GPIO_TypeDef *trigger_port = nullptr, *shooter_port = nullptr;
    GPIO_TypeDef *yun_port = nullptr, *stop_port = nullptr;
    uint16_t trigger_pin = 0, shooter_pin = 0, yun_pin = 0, stop_pin = 0;

    uint8_t if_motor_start = 0, trigger_start = 0, shooter_trigger = 0, yun_trigger = 0, auto_shooter = 0;
    // 拉伸状态 0静止状态 1复位状态 2三分 3罚球线
    uint8_t lifter_status = 0;
    // 俯仰状态 0静止状态 1标准俯仰
    uint8_t pithcer_status = 0;
    // 0.1720
    //  0.1620
    //  0.1920
    float dis_data[9] = {0.023f, 0.217f, 0.1827f, 0.1820f, 0.2320f, 0.2340f, 0.1771f, 0.1890f, 0.2211f};
 
    float inital = 0.023f, one = 0.1600f, two = 0.1580f, three = 0.1920f;

    // 初始拉伸距离
    float initial_lifter = 0.1611f;
    // 三分线拉伸距离
    float three_lifter = 0.3510f;
    // 初始俯仰
    float initial_pitcher = -1.142f;

    float lifter_speed = 0.0f, turn_speed = 0.0f, shooter_speed = 0.0f, pithcer_speed = 0.0f;

    float max_lifter_speed = 420.0f, max_turn_speed = 80.0f, max_shooter_speed = 600.0f, max_pithcer_speed = 430.0f, shoot_dis = 0.16f;

    float shoot_disdance = 0.0f, shoot_pitch_angle = 0.0f;

    imu *laser = nullptr;

public:
    void process_data();
    yun_ball_xbox();
    void btn_scan();
    void btnconfig_init();
    void add_motor(power_motor *lfter_motor_, power_motor *turn_motor_, power_motor *shooter_motor_, power_motor *pithcer_motor_);
    void add_trigger(GPIO_TypeDef *trigger_port_, uint16_t trigger_pin_, GPIO_TypeDef *shooter_port_, uint16_t shooter_pin_, uint16_t yun_pin_, GPIO_TypeDef *yun_port_, uint16_t stop_pin_, GPIO_TypeDef *stop_port_);
    void add_encoder(Encoder *encoder_);
    void add_laser(imu *laser_);
    void add_serial_studio(serial_studio *serial_studio_);
    void add_imu(wit_gyro *imu_);
    void adjust_pitcher(float pitch_angle);
    void adjust_lifter(float lifter_distance);
    // 读取GPIO状态
    uint32_t Read_GPIO_State(void);
};

#endif
#endif