#ifndef AUTO_SHOOTER__H
#define AUTO_SHOOTER__H

#ifdef __cplusplus
extern "C"
{
#endif
#include "encoder.h"
#include "TaskManager.h"
#include "RC9Protocol.h"
#include "TrapezoidalPlanner.h"
#include "motor.h"
#include "PID.h"
#include "fitter.h"
#ifdef __cplusplus
}

enum liftMode
{
    PID,    // PID
    TP,     // 梯形规划
    TP_PID, // 梯形规划+PID
};

enum autoMode
{
    auto_lift,   // 拉伸状态
    auto_shoot,  // 发射状态
    auto_revert, // 复位状态
    auto_finish, // 停止状态

};
enum shooterMode
{
    shooter_stop, // 停止
    shooter_hand, // 手动模式
    shooter_auto, // 全自动模式
    shooter_lift, // 单步模式

    shooter_move // 手动移动指定距离

};

// 梯形规划参数
typedef struct planInfo
{

    float max_acc;      // 最大加速度
    float max_dcc;      // 最大减速度
    float max_speed;    // 最大速度
    float inital_speed; // 初始速度
    float final_speed;  // 最终速度
};
typedef struct shooterInfo
{
    float hand_shooter_rpm = 0.0f;         // 手动模式下射球电机转速
    float auto_shooter_rpm = 0.0f;         // 手动模式下射球电机转速
    float shoot_dis = 0.013f;              // 自动模式下拉伸距离  单位 m
    float debug_dis = 0.013f;              // 调试模式下调试距离
    float start_dis = 0.013f;              // 开始位置
    float shoot_disdance = 0.0f;           // 从编码器获取的拉伸距离
    autoMode shooter_status = auto_finish; // 自动射球状态
    liftMode lift_mode = TP_PID;           // 拉伸规划方式
};
class AutoShooter : public ITaskProcessor
{

private:
    power_motor *shooter_motor = nullptr;
    shooterMode shooter_mode = shooter_stop;

    uint8_t timecnt = 0;
    uint8_t count = 0;

    float test_dis;

    GPIO_TypeDef *shooter_port = nullptr, *stop_port = nullptr;
    uint16_t shooter_pin = 0, stop_pin = 0;

    TrapezoidalPlanner1D planer;
    planInfo plan_info;
    float dis_data[9] = {0.01f, 0.1968f, 0.1958f, 0.2190f, 0.2337f, 0.228f, 0.172f, 0.1800f, 0.2211f};
    float lidar_data[9] = {0.020f, 0.1998f, 0.1948f, 0.2050f, 0.1937f, 0.1877f, 0.2420f, 0.2420f, 0.2211f};

    float r[6] = {1.833, 1.955, 2.324, 2.688, 3.553, 3.667};
    float d[6] = {0.152, 0.157, 0.168, 0.183, 0.208, 0.215};
    uint8_t n = 6;

public:
    // 编码器
    Encoder *encoder = nullptr;
    shooterInfo shooter_info;
    uint8_t shooter_flag = 0;
    pid dis_control;
    PolynomialFitter *fitter;
    AutoShooter();
    void process_data();
    void get_data();

    void add_fitter(PolynomialFitter *fitter_);
    void add_encoder(Encoder *encoder_);
    void add_trigger(GPIO_TypeDef *stop_port_, uint8_t stop_pin_, GPIO_TypeDef *shooter_port_, uint16_t shooter_pin_);
    void add_motor(power_motor *shooter_motor_);
    void add_plan_info(float max_acc_, float max_dcc_, float max_speed_, float inital_speed_, float final_speed_);

    void lift_adjust(float shoot_dis);
    bool auto_adjust(float lifter_dis);
    bool TP_adjust(float lifter_dis);
    void allAuto_adjust(float lifter_dis);
    void hand_adjust();
    bool isfinish();

    void shooter_hand_move();
    float hand_move_dis = 0.17f;

    void
    check_shooter();
    void calc_fitter();
    void set_shooter_mode(uint8_t mode);
    void set_auto(uint8_t mode, float r);
    void set_shooter_rpm(float rpm);
    void set_shooter_dis(float dis);

    // 调试用
    float get_t_dis();
    float get_shooter_rpm();
    float get_shooter_dis();

    uint32_t Read_GPIO_State(void);
};
#endif
#ifdef __cplusplus

#endif
#endif