/*******************************************************************************
 * @file xbox.h
 * @author 6Jerry (1517752988@qq.com)
 * @brief xbox remote control.
 * @version 1.0
 * @date 2024-10-26
 *
 * @copyright Copyright (c) 2024-10-26 6Jerry
 *
 * @license MIT
 *
 * @disclaimer This software is provided "as is", without warranty of any kind, express or implied,
 *             including but not limited to the warranties of merchantability, fitness for a
 *             particular purpose and noninfringement. In no event shall the authors be liable for any
 *             claim, damages or other liability, whether in an action of contract, tort or otherwise,
 *             arising from, out of or in connection with the software or the use or other dealings
 *             in the software.
 ******************************************************************************/
#ifndef XBOX_H
#define XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "RC9Protocol.h"
#include "chassis.h"
#include "Action.h"
#include "EncodingStateMachine.h"

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

#define MAX_SHOOT_RPM_UP 4600.0f
#define MAX_SHOOT_RPM_DOWN 3600.0f
typedef struct
{
    // 按键数据（bool类型）
    bool btnY;
    bool btnY_last;
    bool btnB;
    bool btnB_last;
    bool btnA;
    bool btnA_last;
    bool btnX;
    bool btnX_last;
    bool btnShare;
    bool btnShare_last;
    bool btnStart;
    bool btnStart_last;
    bool btnSelect;
    bool btnSelect_last;
    bool btnXbox;
    bool btnXbox_last;
    bool btnLB;
    bool btnLB_last;
    bool btnRB;
    bool btnRB_last;
    bool btnLS;
    bool btnLS_last;
    bool btnRS;
    bool btnRS_last;
    bool btnDirUp;
    bool btnDirup_last;
    bool btnDirLeft;
    bool btnDirLeft_last;
    bool btnDirRight;
    bool btnDirRight_last;
    bool btnDirDown;
    bool btnDirDown_last;

    // 霍尔值（16位数值）
    uint16_t joyLHori;
    uint16_t joyLVert;
    uint16_t joyRHori;
    uint16_t joyRVert;
    uint16_t trigLT;
    uint16_t trigRT;

    float joyLHori_map;
    float joyLVert_map;
    float joyRHori_map;
    float joyRVert_map;
    float trigLT_map;
    float trigRT_map;
} XboxControllerData_t;

// xbox的基类，只实现底盘控制，因为不同的车的机构不同
class xbox : public RC9Protocol_subscriber
{
public:
    uint8_t head_locking_flag = 0;   // 0是不锁死，1是锁死
    uint8_t catch_ball_flag = 0;     // 0是松开，1是夹紧
    uint8_t world_robot_flag = 0;    // 0是机器人坐标系控制，1是世界坐标系控制
    uint8_t robot_stop_flag = 0;     // 0是正常运行，1是触发急停
    uint8_t speed_level = 1;         // 0---低速，1---中速，2---高速
    uint8_t if_point_track_flag = 0; // 0---不开始点追踪，1---开始点追踪
    uint8_t if_pure_pusit = 0;
    float MAX_ROBOT_SPEED_Y = 1.50f;
    float MAX_ROBOT_SPEED_X = 1.50f;
    float locking_heading = 0.0f;
    float MAX_ROBOT_SPEED_W = 3.60f;
    XboxControllerData_t xbox_msgs;
    action *ACTION = nullptr;
    chassis *control_chassis = nullptr;
    float target_trackpoint_x = 0.0f;
    float target_trackpoint_y = 0.0f;
    float target_tracking_dis = 500.0f;
    enum class ButtonActionType
    {
        Toggle,    // 按键状态翻转
        Increment, // 状态递增
        Decrement, // 状态递减
        Custom     // 自定义操作
    };
    struct ButtonConfig
    {
        bool *currentState;
        bool *lastState;              // 上次按键状态
        uint8_t *toggleState;         // 状态变量
        uint8_t maxState;             // 最大状态
        ButtonActionType actionType;  // 按键行为类型
        void (xbox::*customAction)(); // 自定义操作
    };
    void handleButton(ButtonConfig &config);
    ButtonConfig btnAConfig, btnBConfig, btnXConfig, btnRBConfig, btnLBConfig, btnLSConfig, btnXboxConfig, btnRSConfig;
    virtual void btnRB_callback();
    virtual void btnXBOX_callback();
    virtual void btnA_callback() {}
    virtual void btnX_callback() {}

    virtual void chassis_btn_init();

    virtual void chassisbutton_scan();

    FlagConfig flagConfigs[4];         // 标志位配置数组
    EncodingStateMachine stateMachine; // 编码状态机
    void state_machine_init();
    uint8_t currentState = 255;

public:
    xbox(action *ACTION_, chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_ = 1.50f, float MAX_ROBOT_SPEED_X_ = 1.50f, float MAX_ROBOT_SPEED_W_ = 3.60f);
    void chassis_control();
    void update(uint8_t data_id, uint8_t data_length, const uint8_t *data_char, const float *data_float);
};

// 以下是具体车辆的xbox遥控器类

// 九期r1的遥控
class xbox_r1n : public xbox, public ITaskProcessor
{
private:
    /* data */
public:
    xbox_r1n(action *ACTION_, chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_ = 1.50f, float MAX_ROBOT_SPEED_X_ = 1.50f, float MAX_ROBOT_SPEED_W_ = 3.60f);
    void process_data();
};

// 九期r2的遥控
class xbox_r2n : public xbox, public ITaskProcessor
{
private:
    RC9Protocol *robot_data_chain;

public:
    xbox_r2n(action *ACTION_, RC9Protocol *robot_data_chain_, chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_ = 1.50f, float MAX_ROBOT_SPEED_X_ = 1.50f, float MAX_ROBOT_SPEED_W_ = 3.60f);
    void process_data();
};

#endif
#endif