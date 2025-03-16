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
    case chassis_init:
        chassis_initialize();
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
        swerve3_calc(robovel, w);
        break;
    }
}
void RoboChassis::chassis_initialize()
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
        break;
    case swerve4_chassis:
        break;
    case swerve3_chassis:
        swerve3_initialize();
        break;
    }
}

void RoboChassis::swerve3_initialize()
{
    scan_photogate();
    // dmotors[0]->set_rpm(20.0f);
    // dmotors[1]->set_rpm(10.0f);
    // dmotors[2]->set_rpm(20.0f);

    /*
    if (photogate_state[0] == 1 && if_not_init[0])
    {
        dmotors[0]->relocate_pos(-45.0f);
        if_not_init[0] = false;
        dmotors[0]->set_rpm(0.0f);
    }

    if (photogate_state[1] == 1 && if_not_init[1])
    {
        dmotors[1]->relocate_pos(90.0f);
        if_not_init[1] = false;
        dmotors[1]->set_rpm(0.0f);
    }
    if (photogate_state[2] == 1 && if_not_init[2])
    {
        dmotors[2]->relocate_pos(-90.0f);
        if_not_init[2] = false;
        dmotors[2]->set_rpm(0.0f);
    }

    if (if_not_init[0] == false && if_not_init[1] == false && if_not_init[2] == false)
    {
        mode = stop;
    }
        */

    mode = stop;
}

void RoboChassis::scan_photogate()
{
    for (int i = 0; i < 4; i++)
    {
        photogate_state[i] = HAL_GPIO_ReadPin(photogate_port[i], photogate_pin[i]);
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

void RoboChassis::swerve3_calc(Vector2D robovel, float w)
{
    // scan_photogate();

    scan_photogate();

    target.swerve_motor_target[0].x = robovel.x - w * 0.44f;
    target.swerve_motor_target[0].y = robovel.y;

    if (target.swerve_motor_target[0].x != 0.0f | target.swerve_motor_target[0].y != 0.0f)
    {
        target.swerve_motor_angle[0] = atan2f(target.swerve_motor_target[0].x, target.swerve_motor_target[0].y) * 57.296f;
    }

    if (target.swerve_motor_angle[0] >= -90.0f && target.swerve_motor_angle[0] <= 90.0f)
    { // 无需劣弧优化

        motors[0]->set_rpm(v_2_rpm(target.swerve_motor_target[0].magnitude()));
        dmotors[0]->set_pos(target.swerve_motor_angle[0]);
    }
    else if (target.swerve_motor_angle[0] > 90.0f && target.swerve_motor_angle[0] <= 180.0f)
    {
        dmotors[0]->set_pos(target.swerve_motor_angle[0] - 180.0f);
        motors[0]->set_rpm(v_2_rpm(-target.swerve_motor_target[0].magnitude()));
    }
    else if (target.swerve_motor_angle[0] >= -180.0f && target.swerve_motor_angle[0] < -90.0f)
    {
        dmotors[0]->set_pos(target.swerve_motor_angle[0] + 180.0f);
        motors[0]->set_rpm(v_2_rpm(-target.swerve_motor_target[0].magnitude()));
    }

    target.swerve_motor_target[1].x = robovel.x + w * 0.38735f;
    target.swerve_motor_target[1].y = robovel.y - w * 0.38735f;

    if (target.swerve_motor_target[1].x != 0.0f | target.swerve_motor_target[1].y != 0.0f)
    {
        target.swerve_motor_angle[1] = atan2f(target.swerve_motor_target[1].x, target.swerve_motor_target[1].y) * 57.296f;
    }

    if (target.swerve_motor_angle[1] >= -90.0f && target.swerve_motor_angle[1] <= 90.0f)
    { // 无需劣弧优化

        motors[1]->set_rpm(v_2_rpm(-target.swerve_motor_target[1].magnitude()));
        dmotors[1]->set_pos(target.swerve_motor_angle[1]);
    }
    else if (target.swerve_motor_angle[1] > 90.0f && target.swerve_motor_angle[1] <= 180.0f)
    {
        dmotors[1]->set_pos(target.swerve_motor_angle[1] - 180.0f);
        motors[1]->set_rpm(v_2_rpm(target.swerve_motor_target[1].magnitude()));
    }
    else if (target.swerve_motor_angle[1] >= -180.0f && target.swerve_motor_angle[1] < -90.0f)
    {
        dmotors[1]->set_pos(target.swerve_motor_angle[1] + 180.0f);
        motors[1]->set_rpm(v_2_rpm(target.swerve_motor_target[1].magnitude()));
    }

    target.swerve_motor_target[2].x = robovel.x + w * 0.38735f;
    target.swerve_motor_target[2].y = robovel.y + w * 0.38735f;
    if (target.swerve_motor_target[2].x != 0.0f | target.swerve_motor_target[2].y != 0.0f)
    {
        target.swerve_motor_angle[2] = atan2f(target.swerve_motor_target[2].x, target.swerve_motor_target[2].y) * 57.296f;
    }

    if (target.swerve_motor_angle[2] >= -90.0f && target.swerve_motor_angle[2] <= 90.0f)
    { // 无需劣弧优化

        motors[2]->set_rpm(v_2_rpm(target.swerve_motor_target[2].magnitude()));
        dmotors[2]->set_pos(target.swerve_motor_angle[2]);
    }
    else if (target.swerve_motor_angle[2] > 90.0f && target.swerve_motor_angle[2] <= 180.0f)
    {
        dmotors[2]->set_pos(target.swerve_motor_angle[2] - 180.0f);
        motors[2]->set_rpm(v_2_rpm(-target.swerve_motor_target[2].magnitude()));
    }
    else if (target.swerve_motor_angle[2] >= -180.0f && target.swerve_motor_angle[2] < -90.0f)
    {
        dmotors[2]->set_pos(target.swerve_motor_angle[2] + 180.0f);
        motors[2]->set_rpm(v_2_rpm(-target.swerve_motor_target[2].magnitude()));
    }
}

void RoboChassis::add_photogate(GPIO_TypeDef *port1, uint16_t pin1, GPIO_TypeDef *port2, uint16_t pin2, GPIO_TypeDef *port3, uint16_t pin3, GPIO_TypeDef *port4, uint16_t pin4)
{

    photogate_port[0] = port1;
    photogate_pin[0] = pin1;
    photogate_port[1] = port2;
    photogate_pin[1] = pin2;
    photogate_port[2] = port3;
    photogate_pin[2] = pin3;
}

void RoboChassis::add_6_motors(power_motor *front_d_motor, power_motor *front_motor, power_motor *right_d_motor, power_motor *right_motor, power_motor *left_d_motor, power_motor *left_motor)
{
    dmotors[0] = front_d_motor;
    motors[0] = front_motor;
    dmotors[1] = right_d_motor;
    motors[1] = right_motor;
    dmotors[2] = left_d_motor;
    motors[2] = left_motor;
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
    if (mode != chassis_init)
    {
        mode = robotv;
    }

    target.target_robovel = robovel;
    return 1;
}

uint8_t RoboChassis::set_CWorldVel(Vector2D worldvel, uint8_t PriorityCode, chassis_user *user_)
{

    if (mode != chassis_init)
    {
        mode = worldv;
    }

    target.target_worldvel = worldvel;
    return 1;
}

uint8_t RoboChassis::set_CRobotW(float w, uint8_t PriorityCode, chassis_user *user_)
{

    yaw_mode = yaw_free;
    target.target_w = w;
    return 1;
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

    yaw_mode = yaw_TurnTo;
    target.target_yaw = yaw;
    return 1;
}
void RoboChassis::pointtrack_config(float kp, float ki, float kd, float integral_limit, float output_limit, float deadzone, float integral_separation_threshold)
{
    pointtracker.track_pid.ConfigAll(kp, ki, kd, integral_limit, output_limit, deadzone, integral_separation_threshold);
}

uint8_t RoboChassis::Cmove_to(Vector2D target_p, uint8_t PriorityCode, chassis_user *user_)
{
    if (mode != chassis_init)
    {
        mode = point_track;
    }

    target.target_point = target_p;
    return 1;
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