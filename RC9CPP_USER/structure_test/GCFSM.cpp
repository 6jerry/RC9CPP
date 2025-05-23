#include "GCFSM.h"

void catcher_fsm::process_data()
{
    switch (claw_state)
    {
    case claw_hold:
        servo_left->set_ccr(0);
        servo_right->set_ccr(0);
        break;

    case claw_open_1ball:
        time_cnt++;
        servo_right->set_ccr(174);
        if (time_cnt > 100)
        {
            servo_left->set_ccr(92);
            servo_right->set_ccr(174);
            time_cnt = 0;
        }
        break;
    case claw_move_1ball:
        time_cnt++;
        servo_right->set_ccr(right_foward);
        if (time_cnt > 50)
        {
            servo_left->set_ccr(left_inside);
            servo_right->set_ccr(right_foward);
            time_cnt = 0;
        }
        break;

    case claw_throw_ball:
        time_cnt++;
        servo_right->set_ccr(right_outside);
        if (time_cnt > 50)
        {
            servo_left->set_ccr(left_outside);
            servo_right->set_ccr(right_outside);
            time_cnt = 0;
        }
        break;
    }
}
void catcher_fsm::add_servo(servo *servo_left_, servo *servo_right_)
{
    servo_left = servo_left_;
    servo_right = servo_right_;
}

void catcher_fsm::hold_claw()
{
    claw_state = claw_hold;
}
void catcher_fsm::move_ball()
{
    claw_state = claw_move_1ball;
}
void catcher_fsm::ready_catch_ball()
{
    claw_state = claw_open_1ball;
}
void catcher_fsm::throw_ball()
{
    claw_state = claw_throw_ball;
}

GC_fsm::GC_fsm()
{
    f_ball_c_point.x = 0.0f;
    f_ball_c_point.y = 0.0f;
    f_ball_point.x = 0.0f;
    f_ball_point.y = 0.0f;
    throw_point.x = 0.0f;
    throw_point.y = 0.0f;
}

void GC_fsm::process_data()
{
}
