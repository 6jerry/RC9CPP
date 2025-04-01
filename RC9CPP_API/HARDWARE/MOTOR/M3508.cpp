/*******************************************************************************
 * @file M3508.cpp
 * @author 6Jerry (1517752988@qq.com)
 * @brief m3508 motor type.
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
#include "M3508.h"
m3508p::m3508p(uint8_t can_id, CAN_HandleTypeDef *hcan_, bool enable_locate_, float gear_ratio) : CanDevice(M3508, hcan_, can_id), gear_ratio(gear_ratio), dji_motor(20000.0f, 16384, 8191), rpm_control(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f), enable_locate(enable_locate_) // 选择使用积分分离的话积分限幅就是无意义的，随便给个爆大的值就行
{
}
// 电机处理函数，根据控制模式执行相应计算出控制电流，由CanManager自动调用并将结果发送至电调
int16_t m3508p::motor_process()
{
    switch (work_mode)
    {
    case m3508_increPID_speed:
        return increPID_speed();
        break;
    case m3508_pid_speed:
        return pid_speed();
        break;
    case m3508_angle_pid:
        return angle_pid();
        break;
    case m3508_angle_speedplan:
        //return angle_speedplan();
        break;
    case m3508_distance_pid:
        return distance_pid();
        break;
    case m3508_distance_speedplan:
        return distance_speedplan();
        break;
    case m3508_F:
        return F();
        break;
    default:
        break;
    }
}

// 重写can_update函数，由中断自动调用
void m3508p::can_update(uint8_t can_RxData[8])
{
    uint16_t vangle = (can_RxData[0] << 8) | can_RxData[1];
    rangle = vangle_to_rangle(vangle);

    rpm = (can_RxData[2] << 8) | can_RxData[3];

    int16_t vcurrent = (can_RxData[4] << 8) | can_RxData[5];
    rcurrent = vcurrent_to_rcurrent(vcurrent); // 虚拟电流，mA

    if (enable_locate)
    {
        many_pos_locate();
    }
}
//增量式PID控速，返回电流控制值
int16_t m3508p::increPID_speed()
{
    rpm_control.increPID_setarget(target_rpm * gear_ratio);
    return rcurrent_to_vcurrent(rpm_control.increPID_Compute(rpm));
}
//位置式PID串联增量式pid，控制位置输出，返回电流控制值
int16_t m3508p::distance_pid()
{
    time_cnt++;
    if (time_cnt > 10)
    {
        distance_pid_control.setpoint = target_distance;
        target_rpm = distance_pid_control.PID_Compute(dis_sum); // 位置控制的控制频率要远小于速度控制
        time_cnt = 0;
    }

    return increPID_speed();
}

/**
 * @brief 设置T型速度规划
 * @param targetdis 目标距离
 * @param max_speed 最大速度
 * @param max_acc 最大加速度
 * @param max_dec 最大减速度
 * @param finalspeed 最终速度
 * @return true 规划成功，false 上次规划尚未完成
 */
bool m3508p::set_dis_speedplan(float targetdis, float max_speed, float max_acc, float max_dec, float finalspeed)
{
    if (dis_speed_plan.isFinished())
    {
        target_distance = targetdis;
        work_mode = m3508_distance_speedplan;

        if (abs((float)rpm) > min_start_rpm && (targetdis - dis_sum) * (float)rpm > 0.0f)//当前速度大于最小启动速度且目标距离与当前距离同向
        {
            dis_speed_plan.start_plan(max_acc, max_dec, max_speed, rpm_2_v(rpm), finalspeed, dis_sum, targetdis);
            return true;
        }
        else
        {
            // 速度规划的起始速度强制设为最小启动速度
            dis_speed_plan.start_plan(max_acc, max_dec, max_speed, rpm_2_v(min_start_rpm), finalspeed, dis_sum, targetdis);
            return true;
        }
    }
    else
    {
        return false;
    }
}
//重置速度规划
void m3508p::dis_speedplan_restart()
{
    dis_speed_plan.reset();
}
//T型速度规划，由增量式pid算得最终电流控制值
int16_t m3508p::distance_speedplan()
{

    if (enable_debug)
    {
        float send_data[3] = {rpm / gear_ratio, target_rpm, dis_sum};
        sendFloatData(1, send_data, 3);
    }
    if (abs(target_distance - dis_sum) < speed_plan_end_dis && dis_speed_plan.m_finalSpeed == 0.0f)
    {
        dis_speed_plan.reset();// 速度规划结束
        return distance_pid();
    }
    else
    {
        target_rpm = v_2_rpm(dis_speed_plan.plan(dis_sum));
        return increPID_speed();
    }
}

//位置式PID串联增量式PID，控制角度输出值，返回电流控制值
int16_t m3508p::angle_pid()
{

    time_cnt++;
    if (time_cnt > 10)
    {
        if (pos_sum * target_angle >= 0)
        {
            angle_error = target_angle - pos_sum;
        }
        else
        {
            if (pos_sum > 0 && target_angle < 0)
            {
                float positive = 180.0f - pos_sum + 180.0f + target_angle; // 正路径
                float negative = target_angle - pos_sum;
                if (abs(positive) <= abs(negative))
                {
                    angle_error = positive;
                }
                else
                {
                    angle_error = negative; // 选择一个较短的路径
                }
            }
            else if (pos_sum < 0 && target_angle > 0)
            {
                float positive = target_angle - pos_sum;
                float negative = -(360.0f + pos_sum - target_angle);
                if (abs(positive) <= abs(negative))
                {
                    angle_error = positive;
                }
                else
                {
                    angle_error = negative; // 选择一个较短的路径
                }
            }
        }

        if (enable_debug)
        {
            float send_data[3] = {target_angle, pos_sum, rpm / gear_ratio};
            sendFloatData(1, send_data, 3);
        }

        target_rpm = angle_pid_control.PID_ComputeError(angle_error);
        time_cnt = 0;
    }
    return increPID_speed();
}

float m3508p::get_pos()
{
    return pos_sum;
}

void m3508p::set_pos(float pos)
{
    target_angle = pos;
    work_mode = m3508_angle_pid;
}

void m3508p::relocate_pos(float angle)
{
    pos_sum = angle;
}

int16_t m3508p::pid_speed()
{
    return 0;
}

int16_t m3508p::F()
{
    return 0;
}

float m3508p::get_rpm()
{
    return (float)rpm / gear_ratio;
}

void m3508p::set_rpm(float power_motor_rpm)
{
    work_mode = m3508_increPID_speed;
    target_rpm = power_motor_rpm;
}
void m3508p::set_dis(float dis)
{
    work_mode = m3508_distance_pid;
    target_distance = dis;
}
void m3508p::set_F(float F_)
{
    work_mode = m3508_F;
    targrt_T = F_;
}
void m3508p::T_TO_C()
{
    Cff = (Tff / gear_ratio) * 64020.49;
}
void m3508p::set_fTff(float Tff_)
{
    Tff = Tff_;
}

// 差分定位计算3508多圈位置
void m3508p::many_pos_locate() 
{
    if (if_init)
    {
        init_cnt++;
        if (init_cnt > 6)
        {
            locate_restart();
            if_init = false;
        }
    }

    last_pos = now_pos;

    now_pos = rangle;

    temp_delta = now_pos - last_pos;

    if (now_pos > 300.0f)
    {
        if (last_pos < 60.0f)
        {
            delta_pos = -(360.0f - now_pos + last_pos);
        }
        else
        {
            delta_pos = temp_delta;
        }
    }
    else if (now_pos < 60.0f)
    {
        if (last_pos > 300.0f)
        {
            delta_pos = now_pos + (360.0f - last_pos);
        }
        else
        {
            delta_pos = temp_delta;
        }
    }
    else
    {
        delta_pos = temp_delta;
    }

    if (pos_sum >= 180.0f)
    {
        pos_sum -= 360.0f;
    }
    else if (pos_sum < -180.0f)
    {
        pos_sum += 360.0f;
    }

    pos_sum += delta_pos / gear_ratio;
    dis_sum += (delta_pos / 360.0f) * wheel_perimeter;
}

void m3508p::locate_restart()
{
    pos_sum = 0.0f;
    dis_sum = 0.0f;
}

void m3508p::start_debug()
{
    enable_debug = true;
}
// 设置机械参数，使用v_2_rpm和rpm_2_v函数时需先调用，配置好参数
void m3508p::config_mech_param(float gear_ratio_, float wheel_D_)
{
    gear_ratio = gear_ratio_;
    wheel_perimeter = (wheel_D_ * PI) / gear_ratio; // 内圈屁股转一圈相当于外圈轮子转了多远

    wheel_R = wheel_D_ / 2.0f;

    v_2_rpm_k = 60.0f / (wheel_D_ * PI);
    rpm_2_v_k = (wheel_D_ * PI) / 60.0f;
}
// 速度转换为转速
float m3508p::v_2_rpm(float v)
{
    return v * v_2_rpm_k;
}
// 转速转换为速度
float m3508p::rpm_2_v(float rpm_)
{
    return rpm_ * rpm_2_v_k;
}

void m3508p::relocate_dis(float dis)
{
    dis_sum = dis;
}

float m3508p::get_dis()
{
    return dis_sum;
}