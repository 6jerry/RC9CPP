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
 * @note 作为xbox的基类，实现了数据更新，按键的处理，霍尔值的解析及摇杆的映射以及获取按键值。
 *       使用时需要继承该类与ITaskProcessor类设计用户自定义Xbox_Ctrl类：
 *      (1)基类已继承RC9subscriber，使用时需对实例调用addport()函数添加串口实例（即传输的xbox数据的串口），后续的数据更新将自动完成。
 *      (2)将控制逻辑写在process_data()函数中，该函数将在TaskManager中被自动调用。
 *      (3)若要使用某按键单击触发，需对其btnconfig进行配置，如：
 *       ButtonConfig btnAConfig = {
 *          &xbox_msgs.btnA,
 *          &xbox_msgs.btnA_last,
 *          &if_motor_start,
 *          1,
 *          ButtonActionType::Toggle,
 *          nullptr
 *       };
 *       并在process_data()中调用handleButton(btnAConfig)。
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

// xbox的基类
class xbox : public RC9subscriber
{
public:
    float MAX_ROBOT_SPEED_Y = 1.50f;
    float MAX_ROBOT_SPEED_X = 1.50f;
    float locking_heading = 0.0f;
    float MAX_ROBOT_SPEED_W = 3.60f;
    XboxControllerData_t xbox_msgs;

    enum class ButtonActionType
    {
        Toggle,    // 按键状态翻转
        Increment, // 状态递增
        Decrement, // 状态递减
        Onlyread,  // 仅读取
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

    // 对每个按键做了默认配置
    ButtonConfig btnAConfig = {
        &xbox_msgs.btnA,
        &xbox_msgs.btnA_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnBConfig = {
        &xbox_msgs.btnB,
        &xbox_msgs.btnB_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnXConfig = {
        &xbox_msgs.btnX,
        &xbox_msgs.btnX_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnYConfig = {
        &xbox_msgs.btnY,
        &xbox_msgs.btnY_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnRBConfig = {
        &xbox_msgs.btnRB,
        &xbox_msgs.btnRB_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnLBConfig = {
        &xbox_msgs.btnLB,
        &xbox_msgs.btnLB_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnLSConfig = {
        &xbox_msgs.btnLS,
        &xbox_msgs.btnLS_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnRSConfig = {
        &xbox_msgs.btnRS,
        &xbox_msgs.btnRS_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnXboxConfig = {
        &xbox_msgs.btnXbox,
        &xbox_msgs.btnXbox_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnStartConfig = {
        &xbox_msgs.btnStart,
        &xbox_msgs.btnStart_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnShareConfig = {
        &xbox_msgs.btnShare,
        &xbox_msgs.btnShare_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnSelectConfig = {
        &xbox_msgs.btnSelect,
        &xbox_msgs.btnSelect_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnDirUpConfig = {
        &xbox_msgs.btnDirUp,
        &xbox_msgs.btnDirup_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnDirLeftConfig = {
        &xbox_msgs.btnDirLeft,
        &xbox_msgs.btnDirLeft_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnDirRightConfig = {
        &xbox_msgs.btnDirRight,
        &xbox_msgs.btnDirRight_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    ButtonConfig btnDirDownConfig = {
        &xbox_msgs.btnDirDown,
        &xbox_msgs.btnDirDown_last,
        nullptr,
        1,
        ButtonActionType::Onlyread,
        nullptr};

    void handleButton(ButtonConfig &config);
    void joymap_compute();
    bool getButtonState(ButtonConfig &config);

public:
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
};
;

#endif
#endif