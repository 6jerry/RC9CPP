#ifndef R3_SHOOTER__H
#define R3_SHOOTER__H

#ifdef __cplusplus
extern "C"
{
#endif
#include "encoder.h"
#include "TaskManager.h"
#include "RC9Protocol.h"
#include "motor.h"
#include "PID.h"
#include "IO_Interrupt.h"
#ifdef __cplusplus
}
// enum R3AutoMode
// {
//     auto_lift,   // 拉伸状态
//     auto_shoot,  // 发射状态
//     auto_revert, // 复位状态
//     auto_finish, // 停止状态

// };

enum R3Mode
{
    Stop, // 停止
    Hand, // 手动模式
    Auto, // 自动模式
};

enum gate_mode
{
    rising,  // 上升沿触发
    falling, // 下降沿触发
};

typedef struct R3ShootInfo
{
    float hand_rpm = 0.0f; // 手动模式下射球电机转速
    float auto_rpm = 0.0f; // 手动模式下射球电机转速
    float real_dis = 0.0f; // 从编码器获取的拉伸距离
};
class photogate_shoot : public GPIODevice
{
private:
    gate_mode mode;
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
    photogate_shoot(gate_mode mode_, GPIO_TypeDef *port_, uint16_t pin_);
    void handleInterrupt() override;
    void add_io_interrupt(GPIO_TypeDef *port_, uint16_t pin_);
    void reset();
};
class R3Shooter : public ITaskProcessor
{

private:
    power_motor *m1 = nullptr;
    power_motor *m2 = nullptr;
    R3Mode mode = Stop;
    uint32_t last_tick = 0;
    photogate_shoot *gate = nullptr;
    photogate_shoot *gate_down = nullptr;
    float test_rpm;

public:
    // 编码器
    Encoder *encoder = nullptr;
    pid dis_control;
    R3ShootInfo info;
    R3Shooter();
    void process_data();
    // void get_data();

    void init(power_motor *m1_, power_motor *m2_, Encoder *encoder_);
    void add_gate(photogate_shoot *gate_, photogate_shoot *gate_down_);
    bool auto_adjust();

    void hand_adjust();

    // float calc(float x);
    void set_shooter_mode(uint8_t mode_);
    // int set_auto_byFitter(uint8_t mode, float r);
    void set_auto_byrpm(uint8_t mode_ ,float rpm);
    void set_hand(float rpm);
};
#endif
#ifdef __cplusplus

#endif
#endif