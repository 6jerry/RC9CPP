#ifndef __AUTO_YUNBALL_R3_H__
#define __AUTO_YUNBALL_R3_H__

#ifdef __cplusplus
extern "C" {
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

enum state_flag
{
    init_flag,
    static_flag,
    yunball_flag,
    putball_flag,
    stop_flag,
    emergency_stop
};

#ifdef __cplusplus
class AutoYunballR3 : public ITaskProcessor
{
private:
    power_motor *putball_motor;
    state_flag flag = init_flag;
    GPIO_TypeDef *put_port_in, *put_port_out, *push_port, *claw_port;
    uint16_t put_pin_in, put_pin_out, push_pin, claw_pin;

    void set_put(bool if_out);
    void set_claw(bool if_open);
    void set_push(bool if_push);

    void yunball();
    void putball();

    float put_speed = 500.0f;
    float get_speed_put = 0.0f;
    float putball_dis = 0.0f, putball_speed = 0.0f, putball_acc = 0.0f, putball_dec = 0.0f,putball_finalspeed = 0.0f;
    float yunball_dis = 0.0f, yunball_speed = 0.0f, yunball_acc = 0.0f, yunball_dec = 0.0f,yunball_finalspeed = 0.0f;
    
    uint8_t emergency_stop_flag = 0; // 停止标志位
    volatile int32_t emergency_semaphore = 0; // 新增信号量计数器
public:
    void process_data();
    void add_motor(power_motor *putball_motor_);
    void add_io(GPIO_TypeDef *put_port_in_, uint16_t put_pin_in_, GPIO_TypeDef *put_port_out_, uint16_t put_pin_out_, GPIO_TypeDef *push_port_, uint16_t push_pin_, GPIO_TypeDef *claw_port_, uint16_t claw_pin_);

    void control_put_motor(float speed_map);
    void control_claw(bool if_open);
    void control_push(bool if_push);

    bool start_yunball();
    bool start_putball();
    void stop();

    bool if_is_finish(); // 查看机构是否繁忙

    uint8_t test = 0;
    float test_time1 = 200;
    float test_time2 = 350;
};



#endif

#endif //__AUTO_YUNBALL_R3_H__