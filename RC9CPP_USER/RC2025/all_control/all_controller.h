#ifndef ALL_CONTROLLER_H
#define ALL_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "CrsfReceiver.h"
#include "RC9Protocol.h"
#include "imu.h"
#include "TaskManager.h"
#include "robot_chassis.h"
#include "EncodingStateMachine.h"
#include "auto_shooter.h"
#include "auto_yunball.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

typedef struct crsf_send
{

    /* data */

    float position_x = 0.0f, position_y = 0.0f, position_yaw_rad = 0.0f, mid360_yaw_rad = 0.0f; // 显示位姿数据

    uint8_t status_flag = 0, error_flag = 0; // 状态码和错误码

    float debug_dis = 0.0f, dis_2_target = 0.0f;
};

class AllController : public RC9subscriber, public ITaskProcessor, public chassis_user
{
public:
    float max_x_speed = 6.0f, max_y_speed = 6.0f, max_yaw_speed = 6.0f, max_delta_acc = 6.0f, target_accle = 0.0f;
    Vector2D center_point, robot_point, nor_dir; // 篮筐坐标和友军坐标

    void set_accle();

    float dis_2_center = 0.0f, dis_2_robot = 0.0f, heading_2_center = 0.0f, heading_2_robot = 0.0f;

    float pian_x = 3;
    float pian_y = 14.406;

    float target_r = 30.0f;

    CrsfReceiver *crsf_port = nullptr;
    AllController();

    uint8_t sal_flag_ = 0, trigger_on_ = 0, sar_flag_ = 0, l_flag_ = 0, r_flag_ = 0;

    uint8_t send_cnt = 0;
    void send_crsf_datas();
    crsf_send send_datas;

    void
    update_flag();

    AutoShooter *auto_shooter = nullptr;
    auto_yunball *auto_yunball_ptr = nullptr;

    imu *position_imu_ = nullptr, *ros_imu = nullptr;

    void add_yunball_and_shooter(AutoShooter *auto_shooter_ptr_, auto_yunball *auto_yunball_ptr_);
    void add_position_and_ros(imu *position_imu_ptr, imu *ros_imu_ptr);

public:
    void
    process_data();
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override; // 接收友军坐标

    EncodingStateMachine mode_selector;
    static const uint8_t bitWidths[5]; // 位宽数组
    void efsm_init();                  // 初始化编码状态机
    uint8_t currentStateflag = 255;

public:
    void
    remote_move();             // 世界坐标系遥控
    void remote_move_revert(); // 头反过来
    void remote_move_robot();
    void all_stop();
    void calc_data(); // 计算两个目标距离和两个朝向

    // 将机器人的行为详细拆分为超多个比较细节的小函数，方便后面修改，遥控器总共有两个3档按键和两个2档按键，所以最多可有3*3*2*2=36个小行为函数，这些对应关系用编码状态机来控制

    // 扳机触发扳机回调函数后扳机flag才会置1，然后需要程序里面手动将其赋值为0，即扳机flag变成1后会进到对应的行为函数中去，除非这个行为函数自己在执行完之后把flag变为0，否则flag一直为1，再按按键也不会有用

    // 全功能进攻的情况下，没有东西损坏，正常打
    void all_auto_yunball(); // 开局运球,正常移动，但是按键会触发运球 **这是一个与扳机flag相关的行为，行为结束后记得把flag改回去，不然会一直卡在这个函数

    void auto_reload_ball(); // 装填球

    void lock_on_center_point(); // 进攻，锁定篮筐，在该模式下可以移动但是不能自转，车头始终锁定篮筐

    void lock_on_r2(); // 战术传球，锁定队友

    void shoot_2_center_point(); // 原地开火，所有速度为0，不可移动机器人，直到球射出机器人，**这是一个与扳机flag相关的行为，行为结束后记得把flag改回去，不然会一直卡在这个函数

    void shoot_2_r2(); // 原地传球

    void attack_move_mode(); // 常规进攻模式，在该模式下可以自由遥控底盘并进行速度档位和加速度档位控制
    void defend_move_mode(); // 常规防守模式，与进攻模式不同的是机器人的朝向反过来了

    // 准备模式，该模式可以刷新坐标，校准舵轮
    void wait_mode_move(); // 准备模式下的普通遥控是机器人坐标系的
    void reset_sw_motor();
    void reset_all_imu();

    // 手动模式，一般在调试或者车辆受损的情况下使用

    void hand_shoot(); // 拉伸固定量然后发射

    void hand_set_clawpos(); // 手动调整夹爪位置

    float debug_dis = 0.12f, max_debug_dis = 0.3f;

    void add_elrs(CrsfReceiver *crsf_port_);
};

#endif
#endif