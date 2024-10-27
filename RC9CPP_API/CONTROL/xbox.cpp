/*******************************************************************************
 * @file xbox.cpp
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
#include "xbox.h"
xbox::xbox(action *ACTION_, chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_, float MAX_ROBOT_SPEED_X_, float MAX_ROBOT_SPEED_W_) : ACTION(ACTION_), control_chassis(control_chassis_), MAX_ROBOT_SPEED_Y(MAX_ROBOT_SPEED_Y_), MAX_ROBOT_SPEED_X(MAX_ROBOT_SPEED_X_), MAX_ROBOT_SPEED_W(MAX_ROBOT_SPEED_W_), flagConfigs{{&world_robot_flag, 1}, {&robot_stop_flag, 1}, {&if_point_track_flag, 1}, {&if_pure_pusit, 1}}, stateMachine(flagConfigs, 4) // 初始化编码状态机
{
    chassis_btn_init();
    state_machine_init();
}

void xbox::state_machine_init()
{
    // 定义状态的索引值数组以及对应的状态处理函数
    uint16_t worldRemoteIndices[] = {1};                        // 假设索引值为0和2对应世界坐标系遥控
    uint16_t robotRemoteIndices[] = {0};                        // 假设索引值为1对应机器人坐标系遥控
    uint16_t robotStopIndices[] = {2, 10, 6, 14, 3, 11, 7, 15}; // 假设索引值为3对应机器人停止
    uint16_t pointTrackIndices[] = {5, 4};                      // 假设索引值为4对应点追踪模式
    uint16_t purePursuitIndices[] = {8, 9};                     // 假设索引值为5对应纯粹追踪模式

    stateMachine.mapStateToIndices(0, worldRemoteIndices, 1);
    stateMachine.mapStateToIndices(1, robotRemoteIndices, 1);
    stateMachine.mapStateToIndices(2, pointTrackIndices, 2);
    stateMachine.mapStateToIndices(3, purePursuitIndices, 2);
    stateMachine.mapStateToIndices(4, robotStopIndices, 8);
}

void xbox::update(uint8_t data_id, uint8_t data_length, const uint8_t *data_char, const float *data_float)
{
    if (data_length == 28)
    {
        // 解析按键数据 (bool 值)
        xbox_msgs.btnY = data_char[0];
        xbox_msgs.btnB = data_char[1];
        xbox_msgs.btnA = data_char[2];
        xbox_msgs.btnX = data_char[3];
        xbox_msgs.btnShare = data_char[4];
        xbox_msgs.btnStart = data_char[5];
        xbox_msgs.btnSelect = data_char[6];
        xbox_msgs.btnXbox = data_char[7];
        xbox_msgs.btnLB = data_char[8];
        xbox_msgs.btnRB = data_char[9];
        xbox_msgs.btnLS = data_char[10];
        xbox_msgs.btnRS = data_char[11];
        xbox_msgs.btnDirUp = data_char[12];
        xbox_msgs.btnDirLeft = data_char[13];
        xbox_msgs.btnDirRight = data_char[14];
        xbox_msgs.btnDirDown = data_char[15];

        // 解析霍尔传感器值（16位数据，高8位和低8位拼接）
        xbox_msgs.joyLHori = ((uint16_t)data_char[16] << 8) | data_char[17];
        xbox_msgs.joyLVert = ((uint16_t)data_char[18] << 8) | data_char[19];
        xbox_msgs.joyRHori = ((uint16_t)data_char[20] << 8) | data_char[21];
        xbox_msgs.joyRVert = ((uint16_t)data_char[22] << 8) | data_char[23];
        xbox_msgs.trigLT = ((uint16_t)data_char[24] << 8) | data_char[25];
        xbox_msgs.trigRT = ((uint16_t)data_char[26] << 8) | data_char[27];
    }
}

void xbox::chassis_control()
{

    chassisbutton_scan();
    currentState = stateMachine.getState();

    // button_scan();
    if (speed_level == 1)
    {
        MAX_ROBOT_SPEED_X = 1.20f;
        MAX_ROBOT_SPEED_Y = 1.20f;
        MAX_ROBOT_SPEED_W = 3.20f;
    }
    if (speed_level == 0)
    {
        MAX_ROBOT_SPEED_X = 0.40f;
        MAX_ROBOT_SPEED_Y = 0.40f;
        MAX_ROBOT_SPEED_W = 1.10f;
    }
    if (speed_level == 2)
    {
        MAX_ROBOT_SPEED_X = 1.96f;
        MAX_ROBOT_SPEED_Y = 1.96f;
        MAX_ROBOT_SPEED_W = 3.98f;
    }
    if (xbox_msgs.btnXbox == 1)
    {
        ACTION->restart();
    }
    if (xbox_msgs.joyLHori > 31000 && xbox_msgs.joyLHori < 350000)
    {
        xbox_msgs.joyLHori_map = 0.0f;
    }
    if (xbox_msgs.joyLHori <= 31000)
    {
        xbox_msgs.joyLHori_map = (31000.0f - (float)xbox_msgs.joyLHori) / 31000.0f;
    }
    if (xbox_msgs.joyLHori >= 35000)
    {
        xbox_msgs.joyLHori_map = (35000.0f - (float)xbox_msgs.joyLHori) / 30535.0f;
    }

    if (xbox_msgs.joyLVert > 31000 && xbox_msgs.joyLVert < 350000)
    {
        xbox_msgs.joyLVert_map = 0.0f;
    }
    if (xbox_msgs.joyLVert <= 31000)
    {
        xbox_msgs.joyLVert_map = (31000.0f - (float)xbox_msgs.joyLVert) / 31000.0f;
    }
    if (xbox_msgs.joyLVert >= 35000)
    {
        xbox_msgs.joyLVert_map = (35000.0f - (float)xbox_msgs.joyLVert) / 30535.0f;
    }

    if (xbox_msgs.joyRHori > 31000 && xbox_msgs.joyRHori < 35000)
    {
        xbox_msgs.joyRHori_map = 0.0f;
    }
    if (xbox_msgs.joyRHori <= 31000)
    {
        xbox_msgs.joyRHori_map = (31000.0f - (float)xbox_msgs.joyRHori) / 31000.0f;
    }
    if (xbox_msgs.joyRHori >= 35000)
    {
        xbox_msgs.joyRHori_map = (35000.0f - (float)xbox_msgs.joyRHori) / 30535.0f;
    }
    if (head_locking_flag == 1)
    {
        control_chassis->lock();
    }
    if (head_locking_flag == 0)
    {
        control_chassis->unlock();
    }
    switch (currentState)
    {
    case 0:
        control_chassis->switch_chassis_mode(remote_worldv);
        control_chassis->setworldv(MAX_ROBOT_SPEED_X * xbox_msgs.joyLHori_map, MAX_ROBOT_SPEED_Y * xbox_msgs.joyLVert_map, -MAX_ROBOT_SPEED_W * xbox_msgs.joyRHori_map);
        break;
    case 1:
        control_chassis->switch_chassis_mode(remote_robotv);
        control_chassis->setrobotv(MAX_ROBOT_SPEED_X * xbox_msgs.joyLHori_map, MAX_ROBOT_SPEED_Y * xbox_msgs.joyLVert_map, -MAX_ROBOT_SPEED_W * xbox_msgs.joyRHori_map);
        break;
    case 2:
        control_chassis->switch_chassis_mode(pure_pursuit);
        break;
    case 3:
        control_chassis->switch_chassis_mode(chassis_standby);
        break;
    case 4:
        control_chassis->switch_chassis_mode(chassis_standby);
        break;
    case 255:
        control_chassis->switch_chassis_mode(chassis_standby);
        break;

    default:
        break;
    }
}
void xbox::handleButton(ButtonConfig &config)
{
    if (*(config.currentState) && !(*(config.lastState)))
    {
        switch (config.actionType)
        {
        case ButtonActionType::Toggle:
            if (config.toggleState)
            {
                *config.toggleState = (*config.toggleState + 1) % (config.maxState + 1);
            }
            break;
        case ButtonActionType::Increment:
            if (config.toggleState && *config.toggleState < config.maxState)
            {
                (*config.toggleState)++;
            }
            break;
        case ButtonActionType::Decrement:
            if (config.toggleState && *config.toggleState > 0)
            {
                (*config.toggleState)--;
            }
            break;
        case ButtonActionType::Custom:
            if (config.customAction)
            {
                (this->*config.customAction)(); // 调用成员函数
            }
            break;
        }
    }
    *config.lastState = *config.currentState;
}
void xbox::chassisbutton_scan()
{

    handleButton(btnAConfig);
    handleButton(btnRBConfig);
    handleButton(btnXboxConfig);
    handleButton(btnXConfig);
    handleButton(btnLSConfig);
    handleButton(btnBConfig);
    handleButton(btnRSConfig);
}
void xbox::chassis_btn_init()
{

    btnAConfig = {
        &xbox_msgs.btnA,
        &xbox_msgs.btnA_last,
        &if_point_track_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnBConfig = {
        &xbox_msgs.btnB,
        &xbox_msgs.btnB_last,
        &speed_level,
        2,
        ButtonActionType::Increment,
        nullptr};

    btnXConfig = {
        &xbox_msgs.btnX,
        &xbox_msgs.btnX_last,
        &speed_level,
        2,
        ButtonActionType::Decrement,
        nullptr};
    btnLSConfig = {
        &xbox_msgs.btnLS,
        &xbox_msgs.btnLS_last,
        &robot_stop_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnRSConfig = {
        &xbox_msgs.btnRS,
        &xbox_msgs.btnRS_last,
        &world_robot_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnRBConfig = {
        &xbox_msgs.btnRB,
        &xbox_msgs.btnRB_last,
        &head_locking_flag,
        1,
        ButtonActionType::Toggle,
        &xbox::btnRB_callback};

    btnXboxConfig = {
        &xbox_msgs.btnXbox,
        &xbox_msgs.btnXbox_last,
        nullptr,
        0,
        ButtonActionType::Custom,
        &xbox::btnXBOX_callback};
}
void xbox::btnRB_callback()
{
    locking_heading = ACTION->pose_data.yaw_rad;
}
void xbox::btnXBOX_callback()
{
    ACTION->restart();
}
xbox_r1n::xbox_r1n(action *ACTION_, chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_, float MAX_ROBOT_SPEED_X_, float MAX_ROBOT_SPEED_W_) : xbox(ACTION_, control_chassis_, MAX_ROBOT_SPEED_Y_, MAX_ROBOT_SPEED_X_, MAX_ROBOT_SPEED_W_)
{
}

void xbox_r1n::process_data()
{
    chassis_control();
}

xbox_r2n::xbox_r2n(action *ACTION_, RC9Protocol *robot_data_chain_, chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_, float MAX_ROBOT_SPEED_X_, float MAX_ROBOT_SPEED_W_) : xbox(ACTION_, control_chassis_, MAX_ROBOT_SPEED_Y_, MAX_ROBOT_SPEED_X_, MAX_ROBOT_SPEED_W_), robot_data_chain(robot_data_chain_)
{
}
void xbox_r2n::process_data()
{
    target_trackpoint_x = robot_data_chain->rx_frame_mat.data.msg_get[0];
    target_trackpoint_y = robot_data_chain->rx_frame_mat.data.msg_get[1];
    chassis_control();
}
