#ifndef RESXBOX_H
#define RESXBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "xbox.h"
#include "TaskManager.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class resxbox : public xbox, public ITaskProcessor
{
private:
    uint8_t head_locking_flag = 0;   // 0是不锁死，1是锁死
    uint8_t catch_ball_flag = 0;     // 0是松开，1是夹紧
    uint8_t world_robot_flag = 0;    // 0是机器人坐标系控制，1是世界坐标系控制
    uint8_t robot_stop_flag = 0;     // 0是正常运行，1是触发急停
    uint8_t speed_level = 1;         // 0---低速，1---中速，2---高速
    uint8_t if_point_track_flag = 0; // 0---不开始点追踪，1---开始点追踪
    uint8_t if_pure_pusit = 0;
    FlagConfig flagConfigs[4];         // 标志位配置数组
    EncodingStateMachine stateMachine; // 编码状态机
    uint8_t currentState = 255;

public:
    void process_data();
    void state_machine_init();
    void chassis_btn_init();
    void chassisbutton_scan();
    void btnRB_callback() override;
    void btnXBOX_callback() override;

    resxbox(chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_ = 1.50f, float MAX_ROBOT_SPEED_X_ = 1.50f, float MAX_ROBOT_SPEED_W_ = 3.60f);
};

#endif
#endif