#include "robot_chassis.h"

void RoboChassis::process_data()
{

    switch (mode)
    {
    case stop:
        target.target_robovel.x = 0.0f;
        target.target_robovel.y = 0.0f;
        chassis_calc(target.target_robovel, yawadjuster_process());
        break;
    case robotv:
        chassis_calc(target.target_robovel, yawadjuster_process());
        break;
    case worldv:
        chassis_calc(worldv_2_robov(target.target_worldvel), yawadjuster_process());
        break;
    case point_track:
        chassis_calc(worldv_2_robov(pointtracker.track(IMU->get_world_pos(), target.target_point)), yawadjuster_process());
        break;
    case point_track_speedplan:
        break;
    case curve_track:
        break;
    }
}

float RoboChassis::yawadjuster_process()
{
    switch (yaw_mode)
    {
    case yaw_lock:
        break;
    case yaw_TurnTo:
        return yawadjuster.yaw_adjust(IMU->get_heading(), target.target_yaw);
        break;
    case yaw_TurnTo_speedplan:
        break;
    case yaw_free:
        return target.target_w;
        break;
    default:
        break;
    }
}

void RoboChassis::chassis_calc(Vector2D robovel, float w)
{
    switch (type)
    {
    case omni3_chassis:
        break;
    case custom_chassis:
        break;
    case omni4_chassis:
        break;
    case mecanum_chassis:
        mecanum_calc(robovel, w);
        break;
    case swerve4_chassis:
        break;
    case swerve3_chassis:
        break;
    }
}
float RoboChassis::v_2_rpm(float v)
{
    float rpm = v / chassis_info_.wheel_r / (2 * PI) * 60;
    return rpm;
}

Vector2D RoboChassis::worldv_2_robov(Vector2D worldvel)
{
    Vector2D robov;

    robov.x = worldvel.x * arm_cos_f32(IMU->get_yaw_rad()) + worldvel.y * arm_sin_f32(IMU->get_yaw_rad());
    robov.y = worldvel.y * arm_cos_f32(IMU->get_yaw_rad()) - worldvel.x * arm_sin_f32(IMU->get_yaw_rad());

    return robov;
}

void RoboChassis::mecanum_calc(Vector2D robovel, float w)
{
    robovel.x = -robovel.x;
    robovel.y = -robovel.y;
    w = -w;

    motors[0]->set_rpm(-v_2_rpm((-robovel.x + robovel.y + w * (chassis_info_.length + chassis_info_.wide))));
    motors[1]->set_rpm(v_2_rpm((robovel.x + robovel.y - w * (chassis_info_.length + chassis_info_.wide))));
    motors[2]->set_rpm(v_2_rpm((-robovel.x + robovel.y - w * (chassis_info_.length + chassis_info_.wide))));
    motors[3]->set_rpm(-v_2_rpm((robovel.x + robovel.y + w * (chassis_info_.length + chassis_info_.wide))));
}

bool RoboChassis::priority_judge(uint8_t PriorityCode, chassis_user *user_, chassis_cmd_type cmd_type)
{
    if (user_ == user)
    {
        if (PriorityCode < current_priority)
        {
            return false;
        }

        else if (PriorityCode >= current_priority)
        {
            last_priority = current_priority;
            current_priority = PriorityCode;
            return true;
        }
    }
    else
    {
        if (PriorityCode <= current_priority)
        {
            return false;
        }
        else
        {
            user = user_;
            last_priority = current_priority;
            current_priority = PriorityCode;
            return true;
        }
    }
}

uint8_t RoboChassis::set_CRobotVel(Vector2D robovel, uint8_t PriorityCode, chassis_user *user_)
{
    if (priority_judge(PriorityCode, user_, move_cmd))
    {
        mode = robotv;
        target.target_robovel = robovel;
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t RoboChassis::set_CWorldVel(Vector2D worldvel, uint8_t PriorityCode, chassis_user *user_)
{
    if (priority_judge(PriorityCode, user_, move_cmd))
    {
        mode = worldv;
        target.target_worldvel = worldvel;
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t RoboChassis::set_CRobotW(float w, uint8_t PriorityCode, chassis_user *user_)
{
    if (priority_judge(PriorityCode, user_, turn_cmd))
    {
        yaw_mode = yaw_free;
        target.target_w = w;
        return 1;
    }
    else
    {
        return 0;
    }
}

RoboChassis::RoboChassis(RoboChassisType type_) : type(type_)
{
}

void RoboChassis::config(chassis_info info_)
{
    chassis_info_.length = info_.length;
    chassis_info_.wide = info_.wide;
    chassis_info_.wheel_r = info_.wheel_r;
    chassis_info_.R = info_.R;
}

void RoboChassis::add_imu(imu *imu_)
{
    IMU = imu_;
}
void RoboChassis::add4_motors(power_motor *front_left_motor, power_motor *front_right_motor, power_motor *back_right_motor, power_motor *back_left_motor)
{
    motors[0] = front_left_motor;
    motors[1] = front_right_motor;
    motors[2] = back_right_motor;
    motors[3] = back_left_motor;
}

void RoboChassis::chassis_back_priorityC()
{
    current_priority = last_priority;
}

void RoboChassis::chassis_rst_priorityC()
{
    current_priority = 0;
    last_priority = 0;
    user = nullptr;
}

void chassis_user::add_chassis(RoboChassis *chassis_)
{
    robochassis_ = chassis_;
}

uint8_t chassis_user::set_RobotVel(Vector2D robovel, uint8_t PriorityCode)
{
    return robochassis_->set_CRobotVel(robovel, PriorityCode, this);
}

uint8_t chassis_user::set_RobotW(float w, uint8_t PriorityCode)
{
    return robochassis_->set_CRobotW(w, PriorityCode, this);
}

void chassis_user::chassis_back_priority()
{
    robochassis_->chassis_back_priorityC();
}
void chassis_user::chassis_rst_priority()
{
    robochassis_->chassis_rst_priorityC();
}

void RoboChassis::yawadjuster_config(float kp, float ki, float kd, float integral_limit, float output_limit, float deadzone, float integral_separation_threshold)
{
    yawadjuster.yaw_pid.ConfigAll(kp, ki, kd, integral_limit, output_limit, deadzone, integral_separation_threshold);
}

uint8_t RoboChassis::yaw_CTurnTo(float yaw, uint8_t PriorityCode, chassis_user *user_)
{
    if (priority_judge(PriorityCode, user_, turn_cmd))
    {
        yaw_mode = yaw_TurnTo;
        target.target_yaw = yaw;
        return 1;
    }
    else
    {
        return 0;
    }
}
void RoboChassis::pointtrack_config(float kp, float ki, float kd, float integral_limit, float output_limit, float deadzone, float integral_separation_threshold)
{
    pointtracker.track_pid.ConfigAll(kp, ki, kd, integral_limit, output_limit, deadzone, integral_separation_threshold);
}

uint8_t RoboChassis::Cmove_to(Vector2D target_p, uint8_t PriorityCode, chassis_user *user_)
{
    if (priority_judge(PriorityCode, user_, move_cmd))
    {
        mode = point_track;
        target.target_point = target_p;
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t chassis_user::set_WorldVel(Vector2D worldvel, uint8_t PriorityCode)
{
    return robochassis_->set_CWorldVel(worldvel, PriorityCode, this);
}

uint8_t chassis_user::yaw_TurnTo(float yaw, uint8_t PriorityCode)
{
    return robochassis_->yaw_CTurnTo(yaw, PriorityCode, this);
}

float RoboChassis::get_cyaw()
{
    return IMU->get_heading();
}
float chassis_user::get_yaw()
{
    return robochassis_->get_cyaw();
}

uint8_t chassis_user::move_to(Vector2D target_p, uint8_t PriorityCode)
{
    return robochassis_->Cmove_to(target_p, PriorityCode, this);
}