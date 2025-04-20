#include "r2n_xbox.h"

void xbox_r2n::process_data()
{

    chassisbutton_scan();
    currentState = stateMachine.getState();
    joymap_compute();

    // button_scan();
    if (speed_level == 1)
    {
        MAX_ROBOT_SPEED_X = 1.20f;
        MAX_ROBOT_SPEED_Y = 1.20f;
        MAX_ROBOT_SPEED_W = 1.8f;
        MAX_GO1 = 5.6f;
    }
    if (speed_level == 0)
    {
        MAX_ROBOT_SPEED_X = 0.40f;
        MAX_ROBOT_SPEED_Y = 0.40f;
        MAX_ROBOT_SPEED_W = 1.10f;
        MAX_GO1 = 8.0f;
        SERVO->set_ccr(138);
        servo_right->set_ccr(126);
    }
    if (speed_level == 2)
    {
        MAX_ROBOT_SPEED_X = 1.96f;
        MAX_ROBOT_SPEED_Y = 1.96f;
        MAX_ROBOT_SPEED_W = 2.4f;
        MAX_GO1 = 12.0f;
        SERVO->set_ccr(80);
        servo_right->set_ccr(183);
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
        // control_chassis->switch_chassis_mode(remote_worldv);
        //control_chassis->setworldv(MAX_ROBOT_SPEED_X * xbox_msgs.joyLHori_map, MAX_ROBOT_SPEED_Y * xbox_msgs.joyLVert_map, -MAX_ROBOT_SPEED_W * xbox_msgs.joyRHori_map);
        control_chassis->switch_chassis_mode(chassis_standby);

        break;
    case 1:
         control_chassis->switch_chassis_mode(remote_robotv);
        control_chassis->setrobotv(-MAX_ROBOT_SPEED_X * xbox_msgs.joyLHori_map, -MAX_ROBOT_SPEED_Y * xbox_msgs.joyLVert_map, MAX_ROBOT_SPEED_W * xbox_msgs.joyRHori_map);

        break;
    case 2:
        control_chassis->switch_chassis_mode(chassis_standby);

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
void xbox_r2n::chassis_btn_init()
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

void xbox_r2n::chassisbutton_scan()
{

    handleButton(btnAConfig);
    handleButton(btnRBConfig);
    handleButton(btnXboxConfig);
    handleButton(btnXConfig);
    handleButton(btnLSConfig);
    handleButton(btnBConfig);
    handleButton(btnRSConfig);
}
xbox_r2n::xbox_r2n(chassis *control_chassis_, float MAX_ROBOT_SPEED_Y_, float MAX_ROBOT_SPEED_X_, float MAX_ROBOT_SPEED_W_) : xbox(nullptr, control_chassis_, MAX_ROBOT_SPEED_Y_, MAX_ROBOT_SPEED_X_, MAX_ROBOT_SPEED_W_), flagConfigs{{&world_robot_flag, 1}, {&robot_stop_flag, 1}, {&if_point_track_flag, 1}, {&if_pure_pusit, 1}}, stateMachine(flagConfigs, 4) // 初始化编码状态机
{
    chassis_btn_init();
    state_machine_init();
}

void xbox_r2n::state_machine_init()
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

void xbox_r2n::btnRB_callback()
{
    // locking_heading = ACTION->pose_data.yaw_rad;
}
void xbox_r2n::btnXBOX_callback()
{
    // ACTION->restart();
}