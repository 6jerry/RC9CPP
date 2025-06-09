#include "auto_shooter.h"
/**
 * auto_shooter
 * author: yanqy
 * 2025/4/10
 */
AutoShooter::AutoShooter()
{
}
void AutoShooter::process_data()
{
    // 获取拉伸距离和俯仰角度
    get_data();

    fitter->fitAll(r, d, n);

    switch (shooter_mode)
    {
    case shooter_stop:
        shooter_motor->set_rpm(0.0f);
        break;
    case shooter_hand:
        hand_adjust();
        break;

    case shooter_auto:
        allAuto_adjust(shooter_info.shoot_dis);
        break;
    case shooter_lift:
        lift_adjust(0.17f);
        break;

    case shooter_move:

        shooter_hand_move();

        break;

    default:
        break;
    }

    // 检查射球扳机
    check_shooter();
}

void AutoShooter::hand_adjust()
{

    // 触发光电门
    // if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5) && shooter_motor->get_rpm() > 0.0f)
    // {
    //     shooter_info.hand_shooter_rpm = 0.0f;
    // }

    shooter_motor->send_rpm(shooter_info.hand_shooter_rpm);
}

void AutoShooter::shooter_hand_move()
{
    lift_adjust(hand_move_dis);
}

void AutoShooter::set_shooter_rpm(float rpm)
{
    shooter_info.hand_shooter_rpm = rpm;
    shooter_mode = shooter_hand;
}

void AutoShooter::set_shooter_dis(float dis)
{
    hand_move_dis = dis;
    shooter_mode = shooter_move;
}

void AutoShooter::lift_adjust(float shoot_dis)
{

    dis_control.setpoint = shoot_dis;
    // float error = (shoot_dis - shooter_info.shoot_disdance);
    shooter_info.auto_shooter_rpm = dis_control.PID_Compute(shooter_info.shoot_disdance);
    shooter_motor->set_rpm(-shooter_info.auto_shooter_rpm);
}

void AutoShooter::allAuto_adjust(float lifter_dis)
{

    switch (shooter_info.shooter_status)
    {
    case auto_lift:

        if (auto_adjust(lifter_dis))
        {
            shooter_info.shooter_status = auto_shoot;
        }

        break;
    case auto_shoot:
        // shooter_motor->set_rpm(0.0f);
        timecnt++;
        shooter_flag = 1;

        if (timecnt > 5)
        {
            timecnt = 0;
            shooter_info.shooter_status = auto_revert;
            shooter_info.start_dis = shooter_info.shoot_disdance;
            shooter_flag = 0;
        }

        break;
    case auto_revert:
        if (auto_adjust(0.017f))
        {
            shooter_info.shooter_status = auto_finish;
        }

        break;
    case auto_finish:
        shooter_motor->set_rpm(0.0f);
        break;
    default:
        break;
    }
}
bool AutoShooter::TP_adjust(float lifter_dis)
{

    // 使用梯形规划
    planer.reset();
    planer.start_plan(plan_info.max_acc, plan_info.max_dcc,
                      plan_info.max_speed, plan_info.inital_speed, plan_info.final_speed,
                      shooter_info.start_dis, lifter_dis);

    shooter_info.auto_shooter_rpm = -planer.plan(shooter_info.shoot_disdance);
    shooter_motor->set_rpm(shooter_info.auto_shooter_rpm);
    planer.reset();
    // 触发光电门
    //		if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5) && shooter_motor->get_rpm() > 0.0f)
    //   {
    //        shooter_motor->set_rpm(0.0f);
    //    }

    // 到达终点锁住
    if (fabs(shooter_info.shoot_disdance - lifter_dis) < 0.002f)
    {
        return true;
    }

    return false;
}
bool AutoShooter::

    auto_adjust(float lifter_dis)
{
    if (shooter_flag == 1)
    {
        shooter_flag = 0;
    }

    float error = (lifter_dis - shooter_info.shoot_disdance);

    switch (shooter_info.lift_mode)
    {
    case TP:
        TP_adjust(lifter_dis);
        break;

    case PID:
        shooter_info.auto_shooter_rpm = -dis_control.PID_ComputeError(error);
        shooter_motor->set_rpm(shooter_info.auto_shooter_rpm);
        break;

    case TP_PID:
        if (fabs(error) < 0.05f)
        {
            shooter_info.auto_shooter_rpm = -dis_control.PID_ComputeError(error);
            shooter_motor->set_rpm(shooter_info.auto_shooter_rpm);
        }
        else
        {

            TP_adjust(lifter_dis);
        }
        break;
    }

    if (fabs(error) < 0.0015f && fabs(shooter_motor->get_rpm()) < 10.0f)
    {
        test_dis = shooter_info.shoot_disdance;

        return true;
    }

    //	 if(count > 10)
    //   {
    //
    //	   count = 0;
    //
    //		 return true;
    //
    //	 }

    return false;
}

void AutoShooter::calc_fitter()
{

    fitter->fitAll(r, d, n);
}

void AutoShooter::add_fitter(PolynomialFitter *fitter_)
{

    fitter = fitter_;
}
void AutoShooter::add_encoder(Encoder *encoder_)
{
    encoder = encoder_;
}
void AutoShooter::add_trigger(GPIO_TypeDef *stop_port_, uint8_t stop_pin_, GPIO_TypeDef *shooter_port_, uint16_t shooter_pin_)
{

    shooter_port = shooter_port_;
    shooter_pin = shooter_pin_;
    stop_port = stop_port_;
    stop_pin = stop_pin_;
}
void AutoShooter::add_motor(power_motor *shooter_motor_)
{
    shooter_motor = shooter_motor_;
}

void AutoShooter::add_plan_info(float max_acc_, float max_dcc_, float max_speed_, float inital_speed_, float final_speed_)
{

    plan_info.max_acc = max_acc_;
    plan_info.max_dcc = max_dcc_;
    plan_info.max_speed = max_speed_;
    plan_info.inital_speed = inital_speed_;
    plan_info.final_speed = final_speed_;
}
// 根据编码器获取拉伸距离
void AutoShooter::get_data()
{
    shooter_info.shoot_disdance = encoder->get_absolute_distance();
}

// 读取GPIO状态
uint32_t AutoShooter::Read_GPIO_State(void)
{
    return HAL_GPIO_ReadPin(stop_port, stop_pin);
}

bool AutoShooter::isfinish()
{
    return shooter_info.shooter_status == auto_finish;
}

void AutoShooter::check_shooter()
{
    HAL_GPIO_WritePin(shooter_port, shooter_pin, shooter_flag == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void AutoShooter::set_auto(uint8_t mode, float r)
{

    if (shooter_info.shooter_status == auto_finish)
    {
        shooter_mode = shooter_auto;
        shooter_info.lift_mode = static_cast<liftMode>(mode);

        float d = fitter->evalLinear(r);
        if (d > 0.23f)
        {
            d = 0.23f;
        }
        if (d < 0.05f)
        {
            d = 0.05f;
        }
        shooter_info.shoot_dis = d;
        shooter_info.start_dis = shooter_info.shoot_disdance;
        shooter_info.shooter_status = auto_lift;
    }
}

void AutoShooter::set_shooter_mode(uint8_t mode)
{

    shooter_mode = static_cast<shooterMode>(mode);
}

float AutoShooter::get_t_dis()
{
    return hand_move_dis;
}

float AutoShooter::get_shooter_rpm()
{
    return shooter_motor->get_rpm();
}

float AutoShooter::get_shooter_dis()
{
    return encoder->get_absolute_distance();
}