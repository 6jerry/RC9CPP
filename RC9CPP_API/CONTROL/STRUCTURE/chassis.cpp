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
void chassis::line_track_compute()
{

    // 计算法向误差
    Vector2D now_point(ACTION->pose_data.world_pos_x, ACTION->pose_data.world_pos_y);
    line_track_info.now_dis = now_point - line_track_info.target_line_initpoint;

    line_track_info.projected = line_track_info.now_dis.project_onto(line_track_info.target_line);
    line_track_info.tangent_dis = line_track_info.projected.magnitude(); // 切向距离，追踪距离,指向下一点

    line_track_info.project_point = line_track_info.projected + line_track_info.target_line_initpoint;
    Vector2D normal_vector = line_track_info.project_point - now_point; // 法相距离，追踪误差,指向目标曲线
    line_track_info.normal_dis = normal_vector.magnitude();

    line_track_info.tangent_dir = line_track_info.target_line.normalize();
    line_track_info.normal_dir = normal_vector.normalize();

    float nor_speed = normal_control.PID_ComputeError(line_track_info.normal_dis);
    Vector2D normal_speed = nor_speed * line_track_info.normal_dir;

    Vector2D tangent_speed = line_track_info.tan_speed * line_track_info.tangent_dir;

    line_track_info.target_wspeed = normal_speed + tangent_speed;
}
void chassis::set_pursuit_speed(float Pspeed)
{
    line_track_info.tan_speed = -Pspeed;
}
void chassis::pure_pursuit_compute()
{

    // 由点计算得到向量
    Vector2D now_track = pure_pursuit_info.path[pure_pursuit_info.tracking_index] - pure_pursuit_info.path[pure_pursuit_info.tracking_index + 1];
    line_track_info.target_line_initpoint = pure_pursuit_info.path[pure_pursuit_info.tracking_index + 1];
    line_track_info.target_line = now_track;

    line_track_compute();
    // 判断是否需要切换点
    if (line_track_info.tangent_dis < pure_pursuit_info.change_point)
    {
        if (pure_pursuit_info.tracking_index < pure_pursuit_info.point_sum - 2)
        {
            pure_pursuit_info.tracking_index++;
        }
        else
        {
            if (pure_pursuit_info.if_loop)
            {
                // 环形轨迹
                pure_pursuit_info.tracking_index = 0;
            }
            else
            {
                line_track_info.target_wspeed.x = 0.0f;
                line_track_info.target_wspeed.y = 0.0f;
            }
        }
    }

    // 记录一下当前走过的路程并设置一下切向追踪速度，采用梯形速度规划，规划全程的追踪速度
}
chassis::chassis(ChassisType chassistype_, float Rwheel_, action *ACTION_, float headingkp, float headingki, float headingkd, float kp_, float ki_, float kd_) : chassistype(chassistype_), heading_pid(headingkp, headingki, headingkd, 100000.0f, 5.0f, 0.01f, 0.5f), ACTION(ACTION_), Rwheel(Rwheel_), distan_pid(kp_, ki_, kd_, 1000000.0f, 1.4f, 50.0f, 600.0f), normal_control(0.0062f, 0, 0.056f, 10000000.0f, 1.2f, 30.0f, 600.0f), tangential_control(0, 0, 0, 10000000.0f, 1.4f, 30.0f, 600.0f)
{
}

float chassis ::v_to_rpm(float v)
{
    float rpm = v / Rwheel / (2 * PI) * 60;
    return rpm;
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
        line_track_compute();
        input_wvx = line_track_info.target_wspeed.x;
        input_wvy = line_track_info.target_wspeed.y;
        worldv_to_robotv();
        break;
    case point_tracking:
        point_track_compute();
        input_wvx = point_track_info.target_speed_x;
        input_wvy = point_track_info.target_speed_y;
        worldv_to_robotv();

    case pure_pursuit:
        pure_pursuit_compute();

        input_wvx = line_track_info.target_wspeed.x;
        input_wvy = line_track_info.target_wspeed.y;
        worldv_to_robotv();
        break;

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