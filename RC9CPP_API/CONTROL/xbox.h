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
    bool btnDirUp_last;
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
    ButtonConfig btnAConfig, btnBConfig, btnXConfig, btnRBConfig, btnLBConfig, btnLSConfig, btnXboxConfig, btnRSConfig, btnStartConfig, btnShareConfig, btnSelectConfig, btnDirUpConfig, btnDirLeftConfig, btnDirRightConfig, btnDirDownConfig, btnYConfig;
    virtual void btnRB_callback() {}
    virtual void btnXBOX_callback() {}

    void joymap_compute();

public:
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
};

#endif
#endif