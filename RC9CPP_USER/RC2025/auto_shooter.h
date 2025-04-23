#ifndef AUTO_SHOOTER__H
#define AUTO_SHOOTER__H

#ifdef __cplusplus
extern "C"
{
#endif
#include "encoder.h"
#include "serial_studio.h"
#include "wit_gyro.h"
#include "TaskManager.h"
#include "RC9Protocol.h"
#include "TrapezoidalPlanner.h"
#include "motor.h"
#ifdef __cplusplus
}

enum autoMode
{
    auto_lift,   // 拉伸状态
    auto_shoot,  // 发射状态
    auto_revert, // 复位状态
    auto_finish, // 停止状态

};
enum shooterMode
{
    shooter_stop,     // 停止
    shooter_hand,     // 手动模式
    shooter_halfAuto, // 半自动模式
    shooter_allAuto,  // 全自动模式
    shooter_debug,    // 调试模式

};

enum pitcherMode
{
    pitcher_stop,   // 停止
    pitcher_inital, // 初始俯仰模式
    pitcher_inside, // 篮下俯仰模式
    pitcher_hand,   // 手动模式

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
    float hand_shooter_rpm = 0.0f;       // 手动模式下射球电机转速
    float hand_pitcher_rpm = 0.0f;       // 手动模式下俯仰电机转速
    float shoot_dis = 0.013f;            // 自动模式下拉伸距离  单位 m
    float debug_dis = 0.013f;            // 调试模式下调试距离
    float shoot_disdance = 0.0f;         // 从编码器获取的拉伸距离
    float shoot_pitch_angle = 0.0f;      // 从imu获取的俯仰角度
    autoMode shooter_status = auto_finish; // 自动射球状态
};
class AutoShooter : public ITaskProcessor
{

private:
    power_motor *shooter_motor = nullptr, *pitcher_motor = nullptr;
    shooterMode shooter_mode = shooter_stop;
    pitcherMode pitcher_mode = pitcher_stop;

    uint8_t timecnt = 0;

    // 编码器
    Encoder *encoder = nullptr;
    // 维特imu
    wit_gyro *wit_imu = nullptr;

    GPIO_TypeDef *trigger_port = nullptr, *shooter_port = nullptr, *stop_port = nullptr;
    uint16_t trigger_pin = 0, shooter_pin = 0, stop_pin = 0;

    TrapezoidalPlanner1D planer;
    planInfo plan_info;
    float dis_data[9] = {0.01f, 0.1968f, 0.1958f, 0.2190f, 0.2337f, 0.228f, 0.172f, 0.1800f, 0.2211f};
    float lidar_data[9] = {0.020f, 0.1998f, 0.1948f, 0.1937f, 0.1937f, 0.1877f, 0.2420f, 0.2420f, 0.2211f};
    // 标准俯仰                     篮下俯仰（待测量）
    float inital_angle = -0.0158f, inside_angle = 0.0f;

public:
    shooterInfo shooter_info;
    uint8_t trigger_flag = 0, shooter_flag = 0;

    AutoShooter();
    void process_data();
    void get_data();

    void add_imu(Encoder *encoder_, wit_gyro *wit_imu_);
    void add_trigger(GPIO_TypeDef *stop_port_, uint8_t stop_pin_, GPIO_TypeDef *trigger_port_, uint16_t trigger_pin_, GPIO_TypeDef *shooter_port_, uint16_t shooter_pin_);
    void add_motor(power_motor *shooter_motor_, power_motor *pithcer_motor_);
    void add_plan_info(float max_acc_, float max_dcc_, float max_speed_, float inital_speed_, float final_speed_);

    void pitcher_adjust(float pitch_angle);
    bool auto_adjust(float lifter_distance);
    void allAuto_adjust(float lifter_distance);
    void hand_adjust();
    bool isfinish();
    void check_trigger();
    void check_shooter();

    void set_shooter_mode(uint8_t mode);
    void set_halfAuto(uint8_t index);
    void set_allAuto(uint8_t index);
    void set_pitcher_mode(uint8_t mode);

    uint32_t Read_GPIO_State(void);
};
#endif
#ifdef __cplusplus

#endif
#endif