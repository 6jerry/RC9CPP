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
        servo_right->set_ccr(right_ready_for_ball);
        if (time_cnt > 50)
        {
            servo_left->set_ccr(left_ready_for_ball);
            servo_right->set_ccr(right_ready_for_ball);
            time_cnt = 0;
        }
        break;
    case claw_move_1ball:
        time_cnt++;
        servo_left->set_ccr(left_foward);
        if (time_cnt > 30)
        {
            servo_right->set_ccr(right_inside);
            servo_left->set_ccr(left_foward);
            time_cnt = 0;
        }
        break;

    case claw_throw_ball:
        time_cnt++;
        servo_right->set_ccr(right_outside);
        if (time_cnt > 30)
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

void catch_ball_fsm::process_data()
{

    switch (state)
    {
    case catch_ball_standby:
        prio_code = 0;
        break;
    case catch_ball_go_to_1_ball_point:
        move_to(first_ball_point, prio_code);
        yaw_TurnTo(first_ball_point_yaw, prio_code);
        if (get_track_dis() < 0.005f && get_track_dis() != 0.0f)
        {
            state = catch_ball_lock_yaw;
        }
        break;

    case catch_ball_lock_yaw:
        prio_code = 3;
        yaw_TurnTo(get_yaw(), prio_code);
        catcher->ready_catch_ball();
        state = catch_ball_locking_ball;
        break;
    case catch_ball_locking_ball:

        catch_speed.x = ball_locker_pid.PID_ComputeError(ball_info_.x_dis);
        catch_speed.y = catch_ball_speed;
        catcher->ready_catch_ball();
        set_RobotVel(catch_speed, prio_code);

        if (ball_info_.y_dis > entererd_ball_size && abs(ball_info_.x_dis) < 10.0f)
        {
            catcher->move_ball();
            state = catch_ball_ball_entered;
        }

        break;
    case catch_ball_ball_entered:
        time_cnt++;
        catcher->move_ball();
        if (time_cnt > 50)
        {
            state = catch_ball_move_to_throw_ball_point;
            time_cnt = 0;
        }
        break;

    case catch_ball_move_to_throw_ball_point:
        move_to(throw_point, prio_code);
        yaw_TurnTo(throw_point_yaw, prio_code);
        if (get_track_dis() < 0.005f && get_track_dis() != 0.0f)
        {
            catcher->throw_ball();
            state = catch_ball_ready_to_throw_ball;
        }
        // state = catch_ball_ball_have;
        break;

    case catch_ball_ready_to_throw_ball:
        time_cnt++;
        if (time_cnt > 50)
        {
            state = catch_ball_rush_to_throw_ball;
            time_cnt = 0;
        }
        break;

    case catch_ball_rush_to_throw_ball:
        set_RobotVel(Vector2D(0.0f, throw_ball_speed), prio_code);
        time_cnt++;
        if (time_cnt > 50)
        {
            state = catch_ball_wait_ball_in;

            set_RobotVel(Vector2D(0.0f, 0.0f), prio_code);
            // catcher->ready_catch_ball();
            if_finish = 1;
            time_cnt = 0;
        }

        break;

    case catch_ball_wait_ball_in:
        time_cnt++;
        if (time_cnt > 50)
        {
            state = catch_ball_back_to_serch_ball_point;
            time_cnt = 0;
            set_RobotVel(Vector2D(0.0f, 0.0f), prio_code);
        }
        break;

    case catch_ball_back_to_serch_ball_point:
        move_to(serch_point, prio_code);
        yaw_TurnTo(serch_point_yaw, prio_code);
        if (get_track_dis() < 0.005f && get_track_dis() != 0.0f)
        {
            state = catch_ball_lock_yaw;
        }
        break;
    }
}

void catch_ball_fsm::start_catch_ball()
{
    if (state == catch_ball_standby)
    {
        state = catch_ball_go_to_1_ball_point;

        if_finish = 0;
    }
}
void catch_ball_fsm::stop_catch_ball()
{
    state = catch_ball_standby;
    if_finish = 0;
    prio_code = 0;
}

bool catch_ball_fsm::if_finish_catch_ball()
{
    return if_finish;
}

void catch_ball_fsm::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    ball_info_.x_dis = -(floatData[1] + 18.0f);
    ball_info_.y_dis = floatData[2];
}

void catch_ball_fsm::set_throw_point(Vector2D point)
{
    throw_point = point;
}

void catch_ball_fsm::config_ball_pid(float kp_, float ki_, float kd_, float integral_limit_, float output_limit_, float deadzone_, float integral_separation_threshold_)
{
    ball_locker_pid.ConfigAll(kp_, ki_, kd_, integral_limit_, output_limit_, deadzone_, integral_separation_threshold_);
}

void dead_ball_catch::process_data()
{
    switch (state)
    {
    case dead_ball_standby:
        // prio_code = 0;
        break;
    case dead_ball_go_to_1_ball_point:
        break;
    }
}

catch_ball_fsm::catch_ball_fsm()
{
    throw_point.x = 0.225f;
    throw_point.y = 1.0069f;
    serch_point.x = -0.3998f;
    serch_point.y = 0.455f;
    first_ball_point.x = -0.053f;
    first_ball_point.y = 0.508f;
    throw_point_yaw = 90.0f;
    serch_point_yaw = -38.0f;
    first_ball_point_yaw = 0.0f;
}