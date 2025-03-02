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
    /*
      switch (state)
      {
      case catch_ball_standby:
          prio_code = 0;
          break;
      case catch_ball_lock_yaw:
          prio_code = 3;
          yaw_TurnTo(get_yaw(), prio_code);
          catcher->ready_catch_ball();
          state = catch_ball_locking_ball;
          break;
      case catch_ball_locking_ball:
          Vector2D catch_speed;
          catch_speed.x = ball_locker_pid.PID_ComputeError(ball_info_.x_dis);
          catch_speed.y = catch_ball_speed;

          set_RobotVel(catch_speed, prio_code);

          if (ball_info_.y_dis > entererd_ball_size && abs(ball_info_.x_dis) < 10.0f)
          {
              catcher->move_ball();
              state = catch_ball_ball_entered;
          }
          break;
      case catch_ball_ball_entered:
          time_cnt++;
          if (time_cnt > 50)
          {
              state = catch_ball_move_to_throw_ball_point;
              time_cnt = 0;
          }
          break;

      case catch_ball_move_to_throw_ball_point:
          move_to(throw_point, prio_code);
          yaw_TurnTo(90.0f, prio_code);
          if (get_track_dis() < 5.0f)
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
          set_RobotVel(Vector2D(throw_ball_speed, 0.0f), prio_code);
          time_cnt++;
          if (time_cnt > 80)
          {
              state = catch_ball_standby;
              time_cnt = 0;
              set_RobotVel(Vector2D(0.0f, 0.0f), prio_code);
              catcher->ready_catch_ball();
              if_finish = 1;
          }

          break;
      }
          */
}

void catch_ball_fsm::start_catch_ball()
{
    if (state == catch_ball_standby)
    {
        state = catch_ball_lock_yaw;
        if_finish = 0;
    }
}
void catch_ball_fsm::stop_catch_ball()
{
    state = catch_ball_standby;
    if_finish = 0;
}

bool catch_ball_fsm::if_finish_catch_ball()
{
    return if_finish;
}

void catch_ball_fsm::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
}

void catch_ball_fsm::set_throw_point(Vector2D point)
{
    throw_point = point;
}
