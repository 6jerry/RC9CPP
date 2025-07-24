#include "R3_shooter.h"
photogate_shoot::photogate_shoot(gate_mode mode_, GPIO_TypeDef *port_, uint16_t pin_)
{
    mode = mode_;
    port = port_;
    pin = pin_;
}

void photogate_shoot::handleInterrupt()
{

    // 使用异或判断状态
    if (!(mode ^ cont))
    {

        rpm1 = motor1->get_rpm();
        rpm2 = motor2->get_rpm();
        motor1->send_rpm(0.0f);
        motor2->send_rpm(0.0f);
    }
    cont++;
    flag = 1;
}

void photogate_shoot::add_io_interrupt(GPIO_TypeDef *port_, uint16_t pin_)
{
}

void photogate_shoot::reset()
{
    flag = 0;
    cont = 0;
}
R3Shooter::R3Shooter()
{
}

void R3Shooter::init(power_motor *m1_, power_motor *m2_, Encoder *encoder_)
{
    m1 = m1_;
    m2 = m2_;
    encoder = encoder_;
}

void R3Shooter::process_data()
{
    get_data();
    switch (mode)
    {
    case Stop:
        m1->send_rpm(0.0f);
        m2->send_rpm(0.0f);

        gate->reset();
        gate_down->reset();

        break;
    case Hand:
        hand_adjust();

        gate->reset();
        gate_down->reset();
        break;

    case Auto:
        if (auto_adjust(info.auto_rpm))
        {
            mode = Lift;
            info.debug_dis = revert_dis;
            last_tick = HAL_GetTick();
        }

        break;

    case Lift:
        // lift_adjust(info.debug_dis);
        //        		if (HAL_GetTick() - last_tick > 500)
        if (info.real_dis <= 0.50f)
        {
            lift_adjust(info.debug_dis);
        }
        else
        {

            //   m1->send_rpm(-500.0f);
            //   m2->send_rpm(-500.0f);

            m1->send_rpm(-500.0f);
            m2->send_rpm(-500.0f);
        }

        break;
    default:
        m1->send_rpm(0.0f);
        m2->send_rpm(0.0f);
        gate->reset();
        gate_down->reset();
        break;
    }
}

void R3Shooter::lift_adjust(float dis)
{
    float error = (dis - info.real_dis);
    info.auto_rpm = dis_control.PID_ComputeError(error);
    m1->send_rpm(info.auto_rpm);
    // m2->send_rpm(info.auto_rpm);
    m2->set_current(0.0f);
}
void R3Shooter::hand_adjust()
{
    m1->send_rpm(info.hand_rpm);
    m2->send_rpm(info.hand_rpm);
}

void R3Shooter::set_hand(float rpm)
{

    if (mode != Auto)
    {
        mode = Hand;
        info.hand_rpm = rpm;
    }
}

void R3Shooter::set_shooter_mode(uint8_t mode_)
{

    mode = static_cast<R3Mode>(mode_);
}

void R3Shooter::set_auto_byrpm(uint8_t mode_, float rpm)
{
    // 已处于自动状态不可重复设置
    if (mode != Auto)
    {

        mode = static_cast<R3Mode>(mode_);
        // info.auto_mode = lift;
        info.auto_rpm = rpm;
        start_dis = info.real_dis;
    }
}

int R3Shooter::set_auto_byFitter(uint8_t mode_, float r)
{
    // 已处于自动状态不可重复设置
    if (mode != Auto)
    {
        mode = static_cast<R3Mode>(mode_);
        info.auto_rpm = calc(r);

        if (info.auto_rpm > max)
        {
            info.auto_rpm = max;
        }
        start_dis = info.real_dis;

        // 2 Auto 发射
        // 3 Lift 归位
        return mode;
    }
}

void R3Shooter::set_auto_byCameraFitter(float camera_r)
{
    // 已处于自动状态不可重复设置
    if (mode != Auto)
    {
        mode = Auto;
        info.auto_rpm = calc_camera(camera_r);

        if (info.auto_rpm > max)
        {
            info.auto_rpm = max;
        }
        start_dis = info.real_dis;
    }
}

void R3Shooter::set_lift(float dis)
{
    mode = Lift;
    info.debug_dis = dis;
}
void R3Shooter::allAuto_adjust()
{

    switch (info.auto_mode)
    {
    case lift:

        if (auto_adjust(info.auto_rpm))
        {
            last_tick = HAL_GetTick();
            info.auto_mode = shoot;
        }

        break;
    case shoot:
        m1->send_rpm(0.0f);
        m2->send_rpm(0.0f);
        if (HAL_GetTick() - last_tick > 300)
        {
            info.auto_mode = revert;
            start_dis = info.real_dis;
        }

        break;
    case revert:
        if (auto_adjust(-600.0f))
        {
            info.auto_mode = finish;
        }

        break;
    case finish:
        m1->send_rpm(0.0f);
        m2->send_rpm(0.0f);
        break;
    default:
        m1->send_rpm(0.0f);
        m2->send_rpm(0.0f);
        break;
    }
}
bool R3Shooter::auto_adjust(float target_rpm)
{
    //    s_dis = end_dis - start_dis;

    //    k = (target_rpm * target_rpm) / (2 * s_dis);

    //    c_dis = info.real_dis + 0.0001f - start_dis;

    //    test_rpm = sqrt(2 * k * c_dis);

    //    if (((s_dis - c_dis) / s_dis) < 0.5f)
    //    {
    //        test_rpm = target_rpm;
    //    }

    test_rpm = target_rpm;
    // v2 = 2ax;
    if (fabs(end_dis - info.real_dis) < zone || (info.real_dis > end_dis))
    {
        m1->send_rpm(0.0f);
        m2->send_rpm(0.0f);

        return true;
    }
    else
    {
        m1->send_rpm(test_rpm);
        m2->send_rpm(test_rpm);
    }

    // if (gate->flag)
    // {
    //     m1->send_rpm(0.0f);
    //     m2->send_rpm(0.0f);
    //     if (HAL_GetTick() - last_tick > 150)
    //     {
    //         m1->send_rpm(-1000.0f);
    //         m2->send_rpm(-1000.0f);
    //         if (gate_down->flag)
    //         {
    //             m1->send_rpm(0.0f);
    //             m2->send_rpm(0.0f);
    //             return true;
    //         }
    //     }
    // }
    // else
    // {
    //     // info.auto_rpm = info.auto_rpm + k;

    //     if (info.auto_rpm > max)
    //     {
    //         info.auto_rpm = max;
    //     }
    //     last_tick = HAL_GetTick();
    //     gate_down->flag = 0;
    //     m1->send_rpm(info.auto_rpm);
    //     m2->send_rpm(info.auto_rpm);
    // }

    return false;
}

void R3Shooter::add_gate(photogate_shoot *gate_, photogate_shoot *gate_down_)
{

    gate = gate_;
    gate_down = gate_down_;
}

float R3Shooter::calc(float r)
{
    //---------------------多项式拟合-----------------
    //    const float coeffs[] = {
    //            0.0169f, // r³ 系数
    //            -0.1134f,  // r² 系数
    //            0.2847f, // r 系数
    //            -0.0660f   // 常数项
    //        };

    //    float v = coeffv[0];
    //    v = v * r + coeffs[1];
    //    v = v * r + coeffs[2];
    //    v = v * r + coeffs[3];
    //-----------------------------------------------

    //---------------------幂、指数、直线拟合-----------------
    // v = a * pow(r, b) + c * logf(r + 1);
    v = a * pow(r, b) + c;
    // v = a*exp(b*r) + c ;
    // v=0.0321*r+0.0772;
    // v = a*(1-exp(b*r)) + c;
    //-----------------------------------------------

    // v = a * r * r * (1 + b * r) + c;

    return v + offset;
}

float R3Shooter::calc_camera(float camera_r)
{
    // v = a * pow(camera_r, b) + c;

    return v += camera_offset;
}

void R3Shooter::get_data()
{
    info.real_dis = encoder->get_distance();
}