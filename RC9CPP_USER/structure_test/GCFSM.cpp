#include "GCFSM.h"

void catcher_fsm::process_data()
{
    switch (claw_state)
    {
    case claw_hold:
        time_cnt++;
        // servo_left->set_ccr(left_ready_for_ball);
        servo_right->set_ccr(right_inside);
        if (time_cnt > 50)
        {
            servo_left->set_ccr(left_inside);
            time_cnt = 0;
        }
        break;

    case claw_open_1ball:
        time_cnt++;
        servo_left->set_ccr(left_ready_for_ball);
        if (time_cnt > 50)
        {
            servo_left->set_ccr(left_ready_for_ball);
            servo_right->set_ccr(right_ready_for_ball);
            time_cnt = 0;
        }
        break;
    case claw_move_1ball:
        time_cnt++;
        servo_right->set_ccr(right_foward);
        if (time_cnt > 30)
        {
            servo_right->set_ccr(right_foward);
            servo_left->set_ccr(left_inside);
            time_cnt = 0;
        }
        break;

    case claw_throw_ball:
        time_cnt++;
        servo_left->set_ccr(left_outside);
        if (time_cnt > 30)
        {
            servo_left->set_ccr(left_outside);
            servo_right->set_ccr(right_outside);
            time_cnt = 0;
        }
        break;
    case claw_close_1ball:
        time_cnt++;
        servo_left->set_ccr(left_hold_for_ball);
        if (time_cnt > 50)
        {
            servo_right->set_ccr(right_hold_for_ball);
            servo_left->set_ccr(left_hold_for_ball);
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
void catcher_fsm::close_for_ball()
{
    claw_state = claw_close_1ball;
}

void catch_ball_fsm::process_data()
{
    sensor_flag = HAL_GPIO_ReadPin(sensor_port, GPIO_sensor_pin);

   

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
            state = catch_catch_the_first_ball;
        }
        break;

    case catch_catch_the_first_ball:
        ask_lock_ball = 1.0f;
        yaw_TurnTo(0.0f, prio_code);
        if (ball_info_.y_dis > ball_close_size && abs(ball_info_.x_dis) > 70.0f)
        {
            catch_speed.y = -catch_ball_speed;
            catch_speed.x = 0.0f;
            catcher->close_for_ball();
            set_RobotVel(catch_speed, prio_code);
        }
        else
        {
            if (ball_info_.y_dis < ball_close_size)
            {
                catcher->close_for_ball();
            }
            else if (ball_info_.y_dis > ball_close_size)
            {
                catcher->ready_catch_ball();
            }

            catch_speed.x = ball_locker_pid.PID_ComputeError(ball_info_.x_dis);
            catch_speed.y = catch_ball_speed;

            set_RobotVel(catch_speed, prio_code);
        }

        re_try_judge();
        if ((ball_info_.y_dis > entererd_ball_size && abs(ball_info_.x_dis) < 10.0f) | sensor_flag == 1)
        {
            catcher->move_ball();
            state = catch_ball_ball_entered;
        }
        break;

    case catch_ball_lock_yaw:
        prio_code = 3;
        yaw_TurnTo(get_yaw(), prio_code);
        catcher->ready_catch_ball();
        pitcher_motor->set_ccr(pitch_search_ball);
        state = catch_ball_locking_ball;
        break;
    case catch_ball_locking_ball:
        ask_lock_ball = 1.0f;

        if (ball_info_.y_dis > ball_close_size && abs(ball_info_.x_dis) > 70.0f)
        {
            catch_speed.y = -catch_ball_speed;
            catch_speed.x = 0.0f;
            catcher->ready_catch_ball();
            set_RobotVel(catch_speed, prio_code);
        }
        else
        {
            catch_speed.x = ball_locker_pid.PID_ComputeError(ball_info_.x_dis);
            catch_speed.y = catch_ball_speed;
            catcher->ready_catch_ball();
            set_RobotVel(catch_speed, prio_code);
        }

        re_try_judge();
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
            pitcher_motor->set_ccr(pitch_check_ball);
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
            pitcher_motor->set_ccr(pitch_search_ball);
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

    case catch_ball_go_to_retry_point_up_left:
        move_to(retry_point_up_left, prio_code);
        yaw_TurnTo(-66.0f, prio_code);
        if (get_track_dis() < 0.005f && get_track_dis() != 0.0f)
        {
            state = catch_ball_lock_yaw;
        }
        break;

    case catch_ball_go_to_retry_point_up_right:
        move_to(retry_point_up_right, prio_code);
        yaw_TurnTo(66.0f, prio_code);
        if (get_track_dis() < 0.005f && get_track_dis() != 0.0f)
        {
            state = catch_ball_lock_yaw;
        }
        break;

    case catch_ball_go_to_retry_exit_left:
        move_to(retry_exit_left, prio_code);
        yaw_TurnTo(0.0f, prio_code);
        if (get_track_dis() < 0.005f && get_track_dis() != 0.0f)
        {
            state = catch_ball_go_to_retry_point_up_left;
        }
        break;

    case catch_ball_go_to_retry_exit_right:
        move_to(retry_exit_right, prio_code);
        yaw_TurnTo(0.0f, prio_code);
        if (get_track_dis() < 0.005f && get_track_dis() != 0.0f)
        {
            state = catch_ball_go_to_retry_point_up_right;
        }
        break;
    }

    float send_datas[2] = {zone_judge, ask_lock_ball};

    sendFloatData(1, send_datas, 2);
}

void catch_ball_fsm::re_try_judge()
{
    if (get_world_x() > red_zone_retry_x && get_world_y() > re_try_y_down && get_world_y() < re_try_y_up)
    {
        state = catch_ball_go_to_retry_exit_left;
    }

    if (get_world_x() < blue_zone_retry_x && get_world_y() > re_try_y_down && get_world_y() < re_try_y_up)
    {
        state = catch_ball_go_to_retry_exit_right;
    }
}

void catch_ball_fsm::add_servo_and_sensor(servo *pitcher_motor_, GPIO_TypeDef *sensor_port_, uint16_t GPIO_sensor_pin_)
{
    pitcher_motor = pitcher_motor_;
    sensor_port = sensor_port_;
    GPIO_sensor_pin = GPIO_sensor_pin_;
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
    throw_point.x = 0.125f;
    throw_point.y = 1.0869f;
    serch_point.x = 0.0f;
    serch_point.y = 1.128f;
    first_ball_point.x = -0.003f;
    first_ball_point.y = 0.507f;
    throw_point_yaw = 89.0f;
    serch_point_yaw = 0.0f;
    first_ball_point_yaw = 0.0f;

    retry_point_up_left.x = 0.244f;
    retry_point_up_left.y = 1.726f;
    retry_point_up_right.x = -0.37f;
    retry_point_up_right.y = 1.71f;

    retry_exit_left.x = 0.344f;
    retry_exit_left.y = 1.01026f;
    retry_exit_right.x = -0.32f;
    retry_exit_right.y = 0.99f;
}