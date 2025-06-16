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
typedef struct shootInfo
{
    float hand_rpm = 0.0f;               // 手动模式下射球电机转速
    float auto_rpm = 0.0f;               // 手动模式下射球电机转速
    float target_dis = 0.013f;           // 自动模式下目标拉伸距离  单位 m
    float debug_dis = 0.013f;            // 调试模式下调试距离
    float start_dis = 0.013f;            // 开始位置
    float real_dis = 0.0f;               // 从编码器获取的拉伸距离
    autoMode shoot_status = auto_finish; // 自动射球状态
    liftMode lift_mode = TP_PID;         // 拉伸规划方式
};
class AutoShooter : public ITaskProcessor
{

private:
    power_motor *shooter_motor = nullptr;
    shooterMode shoot_mode = shooter_stop;

    uint8_t timecnt = 0;
    uint8_t count = 0;
    float target_error = 0.002f;
    float target_rpm = 60.0f;
    float test_dis;
    float offest = 0.0f;
	  float a= 0.0491;
		float b=0.9509;
		float c=0.0772;

    GPIO_TypeDef *shooter_port = nullptr, *stop_port = nullptr;
    uint16_t shooter_pin = 0, stop_pin = 0;

    TrapezoidalPlanner1D planer;
    planInfo plan_info;
    float dis_data[9] = {0.01f, 0.1968f, 0.1958f, 0.2190f, 0.2337f, 0.228f, 0.172f, 0.1800f, 0.2211f};
    float lidar_data[9] = {0.020f, 0.1998f, 0.1948f, 0.2050f, 0.1937f, 0.1877f, 0.2420f, 0.2420f, 0.2211f};

    float r[24] = {2.980f, 3.290f, 3.530f, 3.180f, 2.870f, 2.300f, 2.810f, 3.120f, 3.460f, 3.600f,
                   3.500f, 3.220f, 3.100f, 3.000f, 2.900f, 2.800f, 2.700f, 2.600f, 2.430f, 3.170f,
                   3.280f, 3.430f, 3.536f, 3.530f};
    float d[24] = {0.206f, 0.218f, 0.228f, 0.215f, 0.202f, 0.181f, 0.198f, 0.211f, 0.226f, 0.228f,
                   0.225f, 0.215f, 0.208f, 0.204f, 0.197f, 0.196f, 0.194f, 0.191f, 0.186f, 0.212f,
                   0.219f, 0.225f, 0.227f, 0.226f};
    uint8_t n = 24;
    float s ;
public:
    // 编码器
    Encoder *encoder = nullptr;
    shootInfo shoot_info;
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

    float calc(float x);
    void check_shooter();
    void calc_fitter();
    void set_shooter_mode(uint8_t mode);
    void set_auto_byFitter(uint8_t mode, float r);
    void set_auto_byDis(uint8_t mode, float shoot_dis);
    void set_hand(float rpm);
    void set_lift(float dis);
    // 往下拉一定距离
    //  set_shooter_mode(shooter_lift);
    // 进入停止模式
    //  set_shooter_mode(shooter_stop);

    uint32_t Read_GPIO_State(void);
};
#endif
#ifdef __cplusplus

#endif
#endif