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

    switch (shoot_mode)
    {
    case shooter_stop:
        shooter_motor->set_rpm(0.0f);
        break;
    case shooter_hand:
        hand_adjust();
        break;

    case shooter_auto:
        allAuto_adjust(shoot_info.target_dis);
        break;
    case shooter_lift:
        lift_adjust(shoot_info.target_dis);
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

    shooter_motor->set_rpm(shoot_info.hand_rpm);
}

void AutoShooter::lift_adjust(float shoot_dis)
{
    float error = (shoot_dis - shoot_info.real_dis);
    shoot_info.auto_rpm = -dis_control.PID_ComputeError(error);
    shooter_motor->set_rpm(shoot_info.auto_rpm);
}

void AutoShooter::allAuto_adjust(float lifter_dis)
{

    switch (shoot_info.shoot_status)
    {
    case auto_lift:

        if (auto_adjust(lifter_dis))
        {
            shoot_info.shoot_status = auto_shoot;
        }

        break;
    case auto_shoot:
        // shooter_motor->set_rpm(0.0f);
        timecnt++;
        shooter_flag = 1;

        if (timecnt > 15)
        {
            timecnt = 0;
            shoot_info.shoot_status = auto_revert;
            shoot_info.start_dis = shoot_info.real_dis;
            shooter_flag = 0;
        }

        break;
    case auto_revert:
        if (auto_adjust(0.0145f))
        {
            shoot_info.shoot_status = auto_finish;
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
                      shoot_info.start_dis, lifter_dis);

    shoot_info.auto_rpm = -planer.plan(shoot_info.real_dis);
    shooter_motor->set_rpm(shoot_info.auto_rpm);
    planer.reset();
    // 触发光电门
    //		if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5) && shooter_motor->get_rpm() > 0.0f)
    //   {
    //        shooter_motor->set_rpm(0.0f);
    //    }

    // 到达终点锁住
    if (fabs(shoot_info.real_dis - lifter_dis) < 0.002f)
    {
        return true;
    }

    return false;
}
bool AutoShooter::auto_adjust(float lifter_dis)
{
    if (shooter_flag == 1)
    {
        shooter_flag = 0;
    }

    float error = (lifter_dis - shoot_info.real_dis);

    switch (shoot_info.lift_mode)
    {
    case TP:
        TP_adjust(lifter_dis);
        break;

    case PID:
        shoot_info.auto_rpm = -dis_control.PID_ComputeError(error);
        shooter_motor->set_rpm(shoot_info.auto_rpm);
        break;

    case TP_PID:
        if (fabs(error) < 0.05f)
        {
            shoot_info.auto_rpm = -dis_control.PID_ComputeError(error);
            shooter_motor->set_rpm(shoot_info.auto_rpm);
        }
        else
        {

            TP_adjust(lifter_dis);
        }
        break;
    }

    if (fabs(error) < target_error && fabs(shooter_motor->get_rpm()) < target_rpm)
    {

        test_dis = shoot_info.real_dis;

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
    shoot_info.real_dis = encoder->get_absolute_distance();
}

// 读取GPIO状态
uint32_t AutoShooter::Read_GPIO_State(void)
{
    return HAL_GPIO_ReadPin(stop_port, stop_pin);
}

bool AutoShooter::isfinish()
{
    return shoot_info.shoot_status == auto_finish;
}

void AutoShooter::check_shooter()
{
    HAL_GPIO_WritePin(shooter_port, shooter_pin, shooter_flag == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void AutoShooter::set_auto_byFitter(uint8_t mode, float r)
{

    if (shoot_info.shoot_status == auto_finish)
    {
        shoot_mode = shooter_auto;
        shoot_info.lift_mode = static_cast<liftMode>(mode);
        // float d = fitter->evalLinear(r);
        //  float d = fitter->evalCubic(r);
        float d = calc(r);
        //        if (d > 0.23f)
        //        {
        //            d = 0.23f;
        //        }
        if (d < 0.05f)
        {
            d = 0.05f;
        }
        shoot_info.target_dis = d;
        shoot_info.start_dis = shoot_info.real_dis;
        shoot_info.shoot_status = auto_lift;
    }
}

void AutoShooter::set_auto_byDis(uint8_t mode, float shoot_dis)
{

    if (shoot_info.shoot_status == auto_finish)
    {

        //        if (shoot_dis > 0.23f)
        //        {
        //            shoot_dis = 0.23f;
        //        }
        if (shoot_dis < 0.05f)
        {
            shoot_dis = 0.05f;
        }

        shoot_mode = shooter_auto;
        shoot_info.lift_mode = static_cast<liftMode>(mode);

        shoot_info.target_dis = shoot_dis;
        shoot_info.start_dis = shoot_info.real_dis;
        shoot_info.shoot_status = auto_lift;
    }
}

void AutoShooter::set_lift(float dis)
{

    shoot_mode = shooter_lift;
    shoot_info.target_dis = dis;
}
void AutoShooter::set_hand(float rpm)
{

    shoot_mode = shooter_hand;
    shoot_info.hand_rpm = rpm;
}

void AutoShooter::set_shooter_mode(uint8_t mode)
{

    shoot_mode = static_cast<shooterMode>(mode);
}
float AutoShooter::calc(float r)
{

    //    const float coeffs[] = {
    //        0.0183f, // r³ 系数
    //        -0.1171f,  // r² 系数
    //        0.2796f, // r 系数
    //        -0.0659f   // 常数项
    //    };

    // 使用霍纳法则进行高效计算
    // s = ((-0.0116*r + 0.0804)*r + (-0.1364))*r + 0.2153

    // float s = coeffs[0]; // 从最高次系数开始

    // 循环展开，效率最高
    // s = s * r + coeffs[1];
    // s = s * r + coeffs[2];
    // s = s * r + coeffs[3];

    s = a * pow(r, b) + c * logf(r + 1);
    // offest;
    return s;
}
