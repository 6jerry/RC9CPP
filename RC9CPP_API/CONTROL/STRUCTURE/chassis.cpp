/*******************************************************************************
 * @file chassis.cpp
 * @author 6Jerry (1517752988@qq.com)
 * @brief robot's chassis.
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
#include "chassis.h"
void chassis::switch_chassis_mode(Chassis_mode target_mode)
{
    chassis_mode = target_mode;
}
Chassis_mode chassis::get_mode()
{
    return chassis_mode;
}
bool chassis::setrobotv(float rx, float ry, float w)
{
    input_w = w;
    if (chassis_mode == remote_robotv)
    {
        input_rvx = rx;
        input_rvy = ry;
        return true;
    }
    else
    {
        return false; // 模式错误，设置失败
    }
}
bool chassis::setworldv(float wx, float wy, float w)
{
    input_w = w;
    if (chassis_mode == remote_worldv)
    {
        input_wvx = wx;
        input_wvy = wy;
        return true;
    }
    else
    {
        return false;
    }
}
bool chassis::setpoint(float x, float y)
{

    point_track_info.target_x = x;
    point_track_info.target_y = y;

    // chassis_mode = point_tracking;
}
void chassis::lock_to(float heading)
{
    if_lock_heading = true;
    target_heading_rad = heading;
}
void chassis::lock()
{
    if_lock_heading = true;
    if (if_first_lock == 0)
    {
        target_heading_rad = ACTION->pose_data.yaw_rad;
        if_first_lock = 1;
    }
}
void chassis::unlock()
{
    if_lock_heading = false;
    if_first_lock = 0;
}
float chassis::get_track_state()
{
    return 0;
}
void chassis::worldv_to_robotv()
{
    target_rvx = cos(ACTION->pose_data.yaw_rad) * input_wvx + sin(ACTION->pose_data.yaw_rad) * input_wvy;
    target_rvy = cos(ACTION->pose_data.yaw_rad) * input_wvy - sin(ACTION->pose_data.yaw_rad) * input_wvx;
}
void chassis::point_track_compute()
{
    float dis_vector_x = ACTION->pose_data.world_pos_x - point_track_info.target_x;
    float dis_vector_y = ACTION->pose_data.world_pos_y - point_track_info.target_y;

    point_track_info.distan_error = sqrt(dis_vector_x * dis_vector_x + dis_vector_y * dis_vector_y); // 求模

    // 单位化
    if (point_track_info.distan_error != 0)
    {
        point_track_info.direct_vector_x = dis_vector_x / point_track_info.distan_error;
        point_track_info.direct_vector_y = dis_vector_y / point_track_info.distan_error;

        distan_pid.setpoint = point_track_info.target_distan;
        float target_speed_vector = distan_pid.PID_Compute(point_track_info.distan_error);

        point_track_info.target_speed_x = target_speed_vector * point_track_info.direct_vector_x;
        point_track_info.target_speed_y = target_speed_vector * point_track_info.direct_vector_y;
    }
    else
    {
        point_track_info.target_speed_x = 0.0f;
        point_track_info.target_speed_y = 0.0f;
    }
}

chassis::chassis(ChassisType chassistype_, float Rwheel_, action *ACTION_, float headingkp, float headingki, float headingkd, float kp_, float ki_, float kd_) : chassistype(chassistype_), heading_pid(headingkp, headingki, headingkd, 100000.0f, 5.0f, 0.01f, 0.5f), ACTION(ACTION_), Rwheel(Rwheel_), distan_pid(kp_, ki_, kd_, 1000000.0f, 1.4f, 50.0f, 600.0f), pp_tracker(normalcontrol, 0.0057f, 0.0f, 0.0632f, 0.0f, 0.0f, 0.0f)
{
}

float chassis ::v_to_rpm(float v)
{
    float rpm = v / Rwheel / (2 * PI) * 60;
    return rpm;
}

swerve4 ::swerve4(action *ACTION_, float chassis_r_, float wheel_r_) : chassis(swerve4_, wheel_r_, ACTION_, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)
{
}

void swerve4 ::add_speed_motor(power_motor *right_front_, power_motor *right_back_, power_motor *left_back_, power_motor *left_front_)
{
    speed_motors[0] = right_front_;
    speed_motors[1] = right_back_;
    speed_motors[2] = left_back_;
    speed_motors[3] = left_front_;
}

void swerve4 ::add_heading_motor(power_motor *right_front_, power_motor *right_back_, power_motor *left_back_, power_motor *left_front_)
{
    heading_motors[0] = right_front_;
    heading_motors[1] = right_back_;
    heading_motors[2] = left_back_;
    heading_motors[3] = left_front_;
}

void swerve4 ::process_data()
{
    // 底盘内置小型状态机

    switch (chassis_mode)
    {
    case chassis_standby:
        target_rvx = 0.0f;
        target_rvy = 0.0f;

        break;
    case remote_robotv:

        target_rvx = input_rvx;
        target_rvy = input_rvy;
        break;
    case remote_worldv:

        // worldv_to_robotv();
        break;
    case line_tracking:

        // worldv_to_robotv();
        break;
    case point_tracking:
        point_track_compute();
        input_wvx = point_track_info.target_speed_x;
        input_wvy = point_track_info.target_speed_y;
        // worldv_to_robotv();
        break;

    case pp:
        // Vector2D wpos(ACTION->pose_data.world_pos_x, ACTION->pose_data.world_pos_y);

        // Vector2D tspeed = pp_tracker.pursuit(wpos);

        // input_wvx = tspeed.x;
        // input_wvy = tspeed.y;
        // worldv_to_robotv();
        break;
    }

    target_w = input_w;

    motorspeeds[0].x = target_rvx - target_w * 0.2739f * (0.7071f);
    motorspeeds[0].y = target_rvy - target_w * 0.2739f * (0.7071f);
    // 右前轮子
    if (motorspeeds[0].x != 0.0f | motorspeeds[0].y != 0.0f)
    {

        heading_motors[0]->set_pos(atan2f(motorspeeds[0].x, motorspeeds[0].y) * 57.296f);
    }
    speed_motors[0]->set_rpm(v_to_rpm(motorspeeds[0].magnitude()));

    motorspeeds[1].x = target_rvx + target_w * 0.2739f * (0.7071f);
    motorspeeds[1].y = target_rvy - target_w * 0.2739f * (0.7071f);
    // 右后轮子
    if (motorspeeds[1].x != 0.0f | motorspeeds[1].y != 0.0f)
    {

        heading_motors[1]->set_pos(atan2f(motorspeeds[1].x, motorspeeds[1].y) * 57.296f);
    }
    speed_motors[1]->set_rpm(v_to_rpm(motorspeeds[1].magnitude()));

    motorspeeds[2].x = target_rvx + target_w * 0.2739f * (0.7071f);
    motorspeeds[2].y = target_rvy + target_w * 0.2739f * (0.7071f);
    // 左后轮子
    if (motorspeeds[2].x != 0.0f | motorspeeds[2].y != 0.0f)
    {

        heading_motors[2]->set_pos(atan2f(motorspeeds[2].x, motorspeeds[2].y) * 57.296f);
    }
    speed_motors[2]->set_rpm(v_to_rpm(motorspeeds[2].magnitude()));

    motorspeeds[3].x = target_rvx - target_w * 0.2739f * (0.7071f);
    motorspeeds[3].y = target_rvy + target_w * 0.2739f * (0.7071f);
    // 左前轮子
    if (motorspeeds[3].x != 0.0f | motorspeeds[3].y != 0.0f)
    {

        heading_motors[3]->set_pos(atan2f(motorspeeds[3].x, motorspeeds[3].y) * 57.296f);
    }
    speed_motors[3]->set_rpm(v_to_rpm(motorspeeds[2].magnitude()));

    /*for (int i = 0; i < 4; i++)
    {

        switch (i) // 右前，右后，左后，左前
        {
        case 0:
            motorspeeds[i].x = target_rvx - target_w * 0.2739f * (0.7071f);
            motorspeeds[i].y = target_rvy - target_w * 0.2739f * (0.7071f);

            break;

        case 1:
            motorspeeds[i].x = target_rvx + target_w * 0.2739f * (0.7071f);
            motorspeeds[i].y = target_rvy - target_w * 0.2739f * (0.7071f);

            break;

        case 2:
            motorspeeds[i].x = target_rvx + target_w * 0.2739f * (0.7071f);
            motorspeeds[i].y = target_rvy + target_w * 0.2739f * (0.7071f);

            break;

        case 3:
            motorspeeds[i].x = target_rvx - target_w * 0.2739f * (0.7071f);
            motorspeeds[i].y = target_rvy + target_w * 0.2739f * (0.7071f);

            break;

        default:
            break;
        }

        if (motorspeeds[i].x != 0.0f | motorspeeds[i].y != 0.0f)
        {
            target_angle = atan2f(motorspeeds[i].x, motorspeeds[i].y) * 57.296f;
        }

        // headingerror = target_angle - heading_motors[i]->get_pos();

        /*if (abs(headingerror) > 90.0f)
        {
            // 需要劣弧优化
            if (headingerror > 0.0f)
            {

                setted_pos = target_angle - 180.0f;
                setted_rpm = v_to_rpm(-motorspeeds[i].magnitude());
            }
            else
            {
                setted_pos = target_angle + 180.0f;
                setted_rpm = v_to_rpm(-motorspeeds[i].magnitude());
            }
            if (if_adjust_heading) // 用来设置角度调整死区用的
            {
                heading_motors[i]->set_pos(setted_pos);
            }

            speed_motors[i]->set_rpm(setted_rpm);
        }
        // else
        //{
        //  无需劣弧优化
        setted_pos = target_angle;
        setted_rpm = v_to_rpm(motorspeeds[i].magnitude());
        heading_motors[i]->set_pos(setted_pos);

        speed_motors[i]->set_rpm(setted_rpm);
        //}
    }*/
}
omni3_unusual::omni3_unusual(power_motor *front_motor, power_motor *right_motor, power_motor *left_motor, float Rwheel_, action *ACTION_, float headingkp, float headingki, float headingkd, float point_kp, float point_ki, float point_kd) : chassis(omni3_unusual_, Rwheel_, ACTION_, headingkp, headingki, headingkd, point_kp, point_ki, point_kd)
{
    motors[0] = front_motor;
    motors[1] = right_motor;
    motors[2] = left_motor; // 顺时针安装
}
void omni3_unusual::process_data()
{
    // 底盘内置小型状态机

    switch (chassis_mode)
    {
    case chassis_standby:
        target_rvx = 0.0f;
        target_rvy = 0.0f;

        break;
    case remote_robotv:

        target_rvx = input_rvx;
        target_rvy = input_rvy;
        break;
    case remote_worldv:
        worldv_to_robotv();
        break;
    case line_tracking:

        break;
    case point_tracking:
        point_track_compute();
        input_wvy = point_track_info.target_speed_y;
        input_wvx = point_track_info.target_speed_x;
        worldv_to_robotv();
        break;
    default:

        break;
    }
    if (if_lock_heading)
    {
        heading_pid.setpoint = target_heading_rad;
        target_w = heading_pid.PID_Compute(ACTION->pose_data.yaw_rad);
    }
    else
    {
        target_w = input_w;
    }

    motors[0]->set_rpm(-v_to_rpm(-target_rvx + 0.20024f * target_w));
    motors[1]->set_rpm(v_to_rpm(target_w * 0.3149f + 0.6712f * target_rvx - target_rvy * 0.7412f));
    motors[2]->set_rpm(v_to_rpm(target_w * 0.3149f + 0.6712f * target_rvx + target_rvy * 0.7412f));
}

omni3::omni3(power_motor *front_motor, power_motor *right_motor, power_motor *left_motor, float Rwheel_, float CHASSIS_R_, action *ACTION_, float headingkp, float headingki, float headingkd, float point_kp, float point_ki, float point_kd) : chassis(omni3_unusual_, Rwheel_, ACTION_, headingkp, headingki, headingkd, point_kp, point_ki, point_kd)
{
    motors[0] = front_motor;
    motors[1] = right_motor;
    motors[2] = left_motor; // 顺时针安装

    CHASSIS_R = CHASSIS_R_;
}
void omni3::process_data()
{
    // 底盘内置小型状态机

    switch (chassis_mode)
    {
    case chassis_standby:
        target_rvx = 0.0f;
        target_rvy = 0.0f;

        break;
    case remote_robotv:

        target_rvx = input_rvx;
        target_rvy = input_rvy;
        break;
    case remote_worldv:

        worldv_to_robotv();
        break;
    case line_tracking:

        worldv_to_robotv();
        break;
    case point_tracking:
        point_track_compute();
        input_wvx = point_track_info.target_speed_x;
        input_wvy = point_track_info.target_speed_y;
        worldv_to_robotv();
        break;

    case pp:
        // Vector2D wpos(ACTION->pose_data.world_pos_x, ACTION->pose_data.world_pos_y);

        // Vector2D tspeed = pp_tracker.pursuit(wpos);

        // input_wvx = tspeed.x;
        // input_wvy = tspeed.y;
        worldv_to_robotv();
        break;
    }
    if (if_lock_heading)
    {
        heading_pid.setpoint = target_heading_rad;
        target_w = heading_pid.PID_Compute(ACTION->pose_data.yaw_rad);
    }
    else
    {
        target_w = input_w;
    }

    motors[0]->set_rpm(v_to_rpm(-target_rvx + CHASSIS_R * target_w));
    motors[1]->set_rpm(v_to_rpm(target_w * CHASSIS_R + 0.5f * target_rvx - target_rvy * 0.866f));
    motors[2]->set_rpm(v_to_rpm(target_w * CHASSIS_R + 0.5f * target_rvx + target_rvy * 0.866f));
}
omni4::omni4(power_motor *right_front_motor, power_motor *right_back_motor, power_motor *left_back_motor, power_motor *left_front_motor, float Rwheel_, float CHASSIS_R_, action *ACTION_, float headingkp, float headingki, float headingkd, float point_kp, float point_ki, float point_kd) : chassis(omni4_, Rwheel_, ACTION_, headingkp, headingki, headingkd, point_kp, point_ki, point_kd)
{
    motors[0] = right_front_motor;
    motors[1] = right_back_motor;
    motors[2] = left_back_motor;
    motors[3] = left_front_motor;

    CHASSIS_R = CHASSIS_R_;
}
void omni4::process_data()
{
    // 底盘内置小型状态机

    switch (chassis_mode)
    {
    case chassis_standby:
        target_rvx = 0.0f;
        target_rvy = 0.0f;

        break;
    case remote_robotv:
        target_rvx = input_rvx;
        target_rvy = input_rvy;
        break;
    case remote_worldv:
        worldv_to_robotv();
        break;
    case line_tracking:

        break;
    case point_tracking:
        point_track_compute();
        input_wvx = point_track_info.target_speed_x;
        input_wvy = point_track_info.target_speed_y;
        worldv_to_robotv();
        break;
    default:

        break;
    }
    if (if_lock_heading)
    {
        heading_pid.setpoint = target_heading_rad;
        target_w = heading_pid.PID_Compute(ACTION->pose_data.yaw_rad);
    }
    else
    {
        target_w = input_w;
    }

    motors[0]->set_rpm(v_to_rpm(-target_rvx * 0.70710678f - target_rvy * 0.70710678f + CHASSIS_R * target_w));
    motors[1]->set_rpm(v_to_rpm(target_w * CHASSIS_R + 0.70710678f * target_rvx - target_rvy * 0.70710678f));
    motors[2]->set_rpm(v_to_rpm(target_w * CHASSIS_R + 0.70710678f * target_rvx + target_rvy * 0.70710678f));
    motors[3]->set_rpm(v_to_rpm(target_w * CHASSIS_R - 0.70710678f * target_rvx + target_rvy * 0.70710678f));
}