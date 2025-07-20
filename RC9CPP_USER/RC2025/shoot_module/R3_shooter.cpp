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
    // 获取编码器距离
    // get_data();

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
        if (auto_adjust())
        {
            mode = Stop;
        }

        break;

    default:
        m1->send_rpm(0.0f);
        m2->send_rpm(0.0f);
        gate->reset();
        gate_down->reset();
        break;
    }

    // 检查射球扳机
    // check_shooter();
}
void R3Shooter::hand_adjust()
{
    m1->set_rpm(info.hand_rpm);
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
        info.auto_rpm = rpm;
    }
}
bool R3Shooter::auto_adjust()
{

    m1->send_rpm(info.auto_rpm);
    m2->send_rpm(info.auto_rpm);

    if (gate->flag)
    {
        m1->send_rpm(0.0f);
        m2->send_rpm(0.0f);

        if (HAL_GetTick() - last_tick > 150)
        {
            m1->set_rpm(-500.0f);
            m2->set_rpm(-500.0f);
            if (gate_down->flag)
            {
                m1->send_rpm(0.0f);
                m2->send_rpm(0.0f);
                return true;
            }
        }
    }
    else
    {
        last_tick = HAL_GetTick();
        gate_down->flag = 0;
        m1->send_rpm(info.auto_rpm);
        m2->send_rpm(info.auto_rpm);
    }

    return false;
}

void R3Shooter::add_gate(photogate_shoot *gate_, photogate_shoot *gate_down_)
{

    gate = gate_;
    gate_down = gate_down_;
}