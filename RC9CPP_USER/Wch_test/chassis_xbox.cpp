void chassis_xbox::not_start()
{
    set_RobotVel(Vector2D(0.0f, 0.0f), 0);
    set_RobotW(0.0f, 0);
    // rst_state();
    // rst_state();
}

void chassis_xbox::mode_2()
{
    Vector2D tvel_((3.0f * xbox_msgs.joyLHori_map), (3.0f * xbox_msgs.joyLVert_map));

    set_RobotVel(tvel_, 0);
    set_RobotW(-(3.0f * xbox_msgs.joyRHori_map), 0);
}

void chassis_xbox::mode_1()
{
    Vector2D tvel_((1.0f * xbox_msgs.joyLHori_map), (1.0f * xbox_msgs.joyLVert_map));

    set_RobotVel(tvel_, 0);
    set_RobotW(-(1.0f * xbox_msgs.joyRHori_map), 0);
}

void chassis_xbox::mode_3()
{
    Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));

    set_RobotVel(tvel_, 0);
    set_RobotW(-(5.0f * xbox_msgs.joyRHori_map), 0);
    rst_state();
}
void chassis_xbox::mode_0()
{
    Vector2D tvel_(0.0f, 0.25f);

    set_RobotVel(tvel_, 0);
    set_RobotW(-(5.0f * xbox_msgs.joyRHori_map), 0);
}

