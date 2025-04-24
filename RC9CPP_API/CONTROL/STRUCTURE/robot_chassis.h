#ifndef ROBOT_CHASSIS_H
#define ROBOT_CHASSIS_H

//舵轮底盘加速度开启
#define USE_VEL_ACCEL 1

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "motor.h"
#include "Vector2D.h"
#include "pure_pursuit.h"
#include "imu.h"
#include <arm_math.h>
#include "gpio.h"
#include "RC9Protocol.h"
#include "TrapezoidalPlanner.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

enum RoboChassisType
{
    omni3_chassis,
    custom_chassis,
    omni4_chassis,
    mecanum_chassis,
    swerve4_chassis, // 四舵轮
    swerve3_chassis
};

enum RoboChassis_mode
{
    stop,
    robotv,
    worldv,
    point_track,
    point_track_speedplan,
    curve_track,
    chassis_init,
    ppp_track,
    ppc_track,
    swerve_stable

};

enum chassis_yaw_mode
{
    yaw_lock,
    yaw_TurnTo,
    yaw_TurnTo_speedplan,
    yaw_free,
};

enum chassis_cmd_type
{
    move_cmd,
    turn_cmd,
};

typedef struct chassis_info
{
    float wheel_r = 0.0f;
    float wide = 0.0f;
    float length = 0.0f;
    float R = 0.0f; // 轮子到底盘中心

    float L1 = 0.0f;
    float L2 = 0.0f;
};

typedef struct chassis_target
{
    Vector2D target_robovel, target_worldvel, target_point;
    float target_yaw = 0.0f, target_w = 0.0f;

    float swerve_motor_angle[4] = {0.0f};

    Vector2D swerve_motor_target[4];

    Vector2D pppoint[2];
};

class RoboChassis;
class chassis_user
{
public:
    uint8_t set_RobotVel(Vector2D robovel, uint8_t PriorityCode);
    uint8_t set_WorldVel(Vector2D worldvel, uint8_t PriorityCode);
    uint8_t set_RobotW(float w, uint8_t PriorityCode);
    uint8_t yaw_lock(uint8_t PriorityCode);

    uint8_t yaw_TurnTo(float yaw, uint8_t PriorityCode);
    uint8_t yaw_TurnTo_speedplan(float yaw, uint8_t PriorityCode);
    uint8_t move_to(Vector2D target_p, uint8_t PriorityCode);
    uint8_t move_to_speedplan(Vector2D target, uint8_t PriorityCode);
    uint8_t move_through(Vector2D new_points[], uint8_t length, uint8_t PriorityCode);

    void pp_track_point(Vector2D target_p);

    void rst_state();

    uint8_t get_track_dis(float *dis); // 看看追踪还剩多少

    float get_yaw(); // 获取当前的yaw
    float calc_dis(Vector2D target);
    void chassis_back_priority(); // 回退到上一个优先级
    void chassis_rst_priority();  // 重置优先级

    void add_chassis(RoboChassis *chassis_); // 加入底盘
    float get_world_x();
    float get_world_y();

    void stablize_swerve(); // 稳定四舵轮

    void init_locate(); // 初始化定位

private:
    RoboChassis *robochassis_ = nullptr;

public:
};

class RoboChassis : public ITaskProcessor, public RC9subscriber
{

public:
    void process_data();
    RoboChassis(RoboChassisType type_);
    void config(chassis_info info_);
    void add_imu(imu *imu_);
    void add4_motors(power_motor *front_left_motor, power_motor *front_right_motor, power_motor *back_right_motor, power_motor *back_left_motor);
    void add3_motors(power_motor *front_motor, power_motor *right_motor, power_motor *left_motor);
    void add8_motors(power_motor *front_right_motor, power_motor *front_left_motor, power_motor *back_right_motor, power_motor *back_left_motor, power_motor *front_right_dmotor, power_motor *front_left_dmotor, power_motor *back_right_dmotor, power_motor *back_left_dmotor);
    void pointtrack_config(float kp, float ki, float kd, float integral_limit, float output_limit, float deadzone, float integral_separation_threshold);

    void yawadjuster_config(float kp, float ki, float kd, float integral_limit, float output_limit, float deadzone, float integral_separation_threshold);

    void add_6_motors(power_motor *front_d_motor, power_motor *front_motor, power_motor *right_d_motor, power_motor *right_motor, power_motor *left_d_motor, power_motor *left_motor);

    void add_photogate(GPIO_TypeDef *port1, uint16_t pin1, GPIO_TypeDef *port2, uint16_t pin2, GPIO_TypeDef *port3, uint16_t pin3, GPIO_TypeDef *port4, uint16_t pin4);

    void enable_debug();

private:
    chassis_user *user = nullptr; // 当前是哪个类正在使用底盘
    RoboChassisType type = omni3_chassis;
    RoboChassis_mode mode = chassis_init;
    chassis_yaw_mode yaw_mode = yaw_free;
    uint8_t current_priority = 0, last_priority = 0;
    imu *IMU = nullptr;
    chassis_info chassis_info_;
    chassis_target target;

    power_motor *motors[4] = {nullptr};
    power_motor *dmotors[4] = {nullptr};

    float dmotor_locate_angles[4] = {0.0f};

    GPIO_TypeDef *photogate_port[4] = {nullptr};
    uint16_t photogate_pin[4] = {0};
    uint8_t photogate_state[4] = {0}; // 前轮，右轮，左轮

    bool if_not_init[4] = {true, true, true, true};

    bool if_init_ok = false;

    bool if_enable_debug = false;

    uint32_t time_cnt = 0;

    float last_robovel = 0.0f; float dt = 0.0f; float last_tick = 0.0f;

public:
    pointrack pointtracker;
    // pure_pursuit purepursuiter;
    yaw_adjuster yawadjuster;

    pure_pursuit pp_tracker;

    void scan_photogate();

    float yawadjuster_process();

    void chassis_calc(Vector2D robovel, float w);
    Vector2D worldv_2_robov(Vector2D worldvel);

    void mecanum_calc(Vector2D robovel, float w);
    // void swerve4_calc(Vector2D robovel, float w);

    void chassis_initialize();

    void swerve3_initialize();

    void swerve3_calc(Vector2D robovel, float w);

    float v_2_rpm(float v);

    bool priority_judge(uint8_t PriorityCode, chassis_user *user_, chassis_cmd_type cmd_type); // 优先级仲裁

public:
    uint8_t set_CRobotVel(Vector2D robovel, uint8_t PriorityCode, chassis_user *user_);
    uint8_t set_CWorldVel(Vector2D worldvel, uint8_t PriorityCode, chassis_user *user_);
    uint8_t set_CRobotW(float w, uint8_t PriorityCode, chassis_user *user_);
    uint8_t yaw_Clock(uint8_t PriorityCode, chassis_user *user_);
    uint8_t set_CRobotVel_ACCLE(Vector2D robovel, float accle, uint8_t PriorityCode, chassis_user *user_);

    uint8_t yaw_CTurnTo(float yaw, uint8_t PriorityCode, chassis_user *user_);
    uint8_t yaw_CTurnTo_speedplan(float yaw, uint8_t PriorityCode, chassis_user *user_);
    uint8_t Cmove_to(Vector2D target_p, uint8_t PriorityCode, chassis_user *user_);
    float C_calc_dis(Vector2D target);
    uint8_t Cmove_to_speedplan(Vector2D target, uint8_t PriorityCode, chassis_user *user_);

    uint8_t get_track_disC(float *dis); // 看看追踪还剩多少

    void chassis_back_priorityC(); // 回退到上一个优先级
    void chassis_rst_priorityC();  // 重置优先级

    void C_pp_move_2_point(Vector2D target_p, chassis_user *user_);

    float get_cyaw();
    void C_pp_track_point(Vector2D target_p);

    void C_rst_state();

    float get_cworld_x();
    float get_cworld_y();

    void swerve_stablize();

    void C_stablize();

    void C_init_locate();
};

#endif
#endif