#include "TrapezoidalPlanner.h"

bool TrapezoidalPlanner::speed_pulse_plan_start(float max_speed, float slowdown_pos, float acc_, float dec_, float start_pos, float start_speed, float end_speed)
{
    if (now_state == plan_standby)
    {

        // 计算uniform_start_pos
        startspeed2 = start_speed * start_speed;
        uniform_speed2 = max_speed * max_speed;
        plan_info.uniform_start_pos = (uniform_speed2 - startspeed2) / (2 * acc_) + start_pos;

        // 检查一下上述规划结果是否合理
        if (plan_info.uniform_start_pos > slowdown_pos)
        {
            // plan_info.uniform_start_pos = 0.0f;
            return false;
        }
        plan_info.acc = acc_;
        plan_info.dec = dec_;
        plan_info.dec_start_pos = slowdown_pos;
        plan_info.uniform_speed = max_speed;
        plan_info.start_pos = start_pos;
        plan_info.start_speed = start_speed;
        plan_info.end_speed = end_speed;

        // 下一步计算 end_pos
        plan_info.end_pos = (plan_info.end_speed * plan_info.end_speed - plan_info.uniform_speed * plan_info.uniform_speed) / (2 * plan_info.dec) + plan_info.dec_start_pos;

        now_state = uniform_acc;
        return true;
    }
    else
    {
        return false;
    }
}

float TrapezoidalPlanner::speed_pulse_plan_setpos(float pos)
{
    if (pos >= plan_info.end_pos)
    {
        target_speed = 0.0f;
        now_state = plan_standby;
        return target_speed;
    }
    else if (pos >= plan_info.start_pos && pos < plan_info.uniform_start_pos) // 加速段
    {
        float32_t input = 2 * plan_info.acc * (pos - plan_info.start_pos) + startspeed2; // 输入值
        float32_t output;
        arm_sqrt_f32(input, &output);
        target_speed = output;
        now_state = uniform_acc;
        return target_speed;
    }
    else if (pos >= plan_info.uniform_start_pos && pos < plan_info.dec_start_pos) // 匀速段
    {

        target_speed = plan_info.uniform_speed;
        now_state = uniform;
        return target_speed;
    }
    else if (pos >= plan_info.dec_start_pos && pos < plan_info.end_pos) // 减速段
    {
        float32_t input = 2 * plan_info.dec * (pos - plan_info.dec_start_pos) + uniform_speed2; // 输入值
        float32_t output;
        arm_sqrt_f32(input, &output);
        target_speed = output;
        now_state = uniform_dec;
        return target_speed;
    }
    else
    {
        target_speed = 0.0f;
        now_state = plan_standby;
        return target_speed;
    }
}