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

enum R3Mode
{
    Stop,   // 停止
    Hand,   // 手动模式
    Auto,   // 自动模式
    Auto_2, // 自动二分模式
    Lift,   // 复位模式
    Keep,   // 保持模式
};

enum R3AutoMode
{
    lift,   // 拉伸状态
    shoot,  // 发射状态
    revert, // 复位状态
    finish, // 停止状态

};

enum gate_mode
{
    rising,  // 上升沿触发
    falling, // 下降沿触发
};

typedef struct R3ShootInfo
{
    float hand_rpm = 0.0f; // 手动模式转速
    float auto_rpm = 0.0f; // 自动模式转速
    float real_dis = 0.0f;
    float debug_dis = 0.0245f;
    R3AutoMode auto_mode = finish;
};
class photogate_shoot : public GPIODevice
{
private:
    gate_mode mode;
    power_motor *motor1;
    power_motor *motor2;

public:
    float rpm1 = 0.0f;
    float rpm2 = 0.0f;
    int flag = 0;
    int cont = 0;

    void set_motors(power_motor *m1, power_motor *m2)
    {
        motor1 = m1;
        motor2 = m2;
    }
    photogate_shoot(gate_mode mode_, GPIO_TypeDef *port_, uint16_t pin_);

    // 光电门触发处理函数
    void handleInterrupt() override;
    void add_io_interrupt(GPIO_TypeDef *port_, uint16_t pin_);
    void reset();
};

class R3Shooter : public ITaskProcessor
{

public:
    power_motor *m1 = nullptr;
    power_motor *m2 = nullptr;
    R3Mode mode = Stop;
    uint32_t last_tick = 0;

    photogate_shoot *gate = nullptr;
    photogate_shoot *gate_down = nullptr;
    int flag = 0;
    float start_dis = 0.0f;
    float end_dis = 0.78f;
    float revert_dis = 0.0207f;
    float two_dis = 0.20f;
    float max = 2800.0f;
    float a = 77.4489f;
    float b = 0.5614f;
    float c = 835.2703f;
    float v;
    float offset = 0.0f;
    float camera_offset = 0.002f;
    float pass_offset = 30.0f;
    float s_dis;

    float k;

    float c_dis;

    float zone = 0.003f;

    float camera_to_dis=0.0f; // 相机到发射器的距离
public:
    // 编码器
    Encoder *encoder = nullptr;
    pid dis_control;
    R3ShootInfo info;
    R3Shooter();
    void process_data();
    void get_data();
    void init(power_motor *m1_, power_motor *m2_, Encoder *encoder_);
    void add_gate(photogate_shoot *gate_, photogate_shoot *gate_down_);
    bool auto_adjust(float target_rpm);
    void hand_adjust();
    bool lift_adjust();
    void keep_adjust(float dis);
    float test_rpm = 0.0f;
    float test_c = 0.0f;
    // int set_auto_byFitter(uint8_t mode, float r);
    void set_shooter_mode(uint8_t mode_);
    void set_auto_byrpm(uint8_t mode_, float rpm);
    void set_hand(float rpm);
    int set_auto_byFitter(uint8_t mode_, float r);
    int set_auto_byCameraFitter(uint8_t mode_, float camera_r);
    int set_autoPass_byFitter(uint8_t mode_, float r);
    void set_lift(float dis);
    float calc_camera(float camera_r);
    float calc(float r);
    float calc_pass(float pass_r);
    float CalculateDistance(float camera_r);
};

#endif
#endif