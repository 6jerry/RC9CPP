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

    switch (shooter_mode)
    {
    case shooter_stop:
        shooter_motor->set_rpm(0.0f);
        break;
    case shooter_hand:
        hand_adjust();
        break;
    case shooter_halfAuto:
        auto_adjust(shooter_info.shoot_dis);
        break;
    case shooter_allAuto:

        allAuto_adjust(shooter_info.shoot_dis);
        break;
    case shooter_debug:
        // 隐藏模式：
        // 开启debug后，手动改变worknode为4，即可进入调试模式，
        // 修改debug_dis,即可调整拉伸距离
        auto_adjust(shooter_info.debug_dis);
        break;

    default:
        break;
    }

    // switch (pitcher_mode)
    // {
    // case pitcher_stop:
    //     pitcher_motor->set_rpm(0.0f);
    //     break;
    // case pitcher_inital:
    //     pitcher_adjust(inital_angle);
    //     break;
    // case pitcher_inside:
    //     pitcher_adjust(inside_angle);
    //     break;
    // case pitcher_hand:
    //     pitcher_motor->set_rpm(shooter_info.hand_pitcher_rpm);
    //     break;
    // default:
    //     break;
    // }

    // 检查扳机
    check_trigger();
    check_shooter();
}
void AutoShooter::pitcher_adjust(float pitch_angle)
{

    //    if (shooter_info.shoot_pitch_angle > pitch_angle)
    //    {
    //        pitcher_motor->set_rpm(200.0f);
    //    }
    //    else
    //    {
    //        pitcher_motor->set_rpm(-200.0f);
    //    }

    //    if (shooter_info.shoot_pitch_angle > pitch_angle - 0.003f && shooter_info.shoot_pitch_angle < pitch_angle + 0.003f)
    //    {
    //        pitcher_mode = pitcher_stop;
    //        pitcher_motor->set_rpm(0.0f);
    //        shooter_info.pitcher_status = 1;
    //    }
}
void AutoShooter::hand_adjust()
{

    // 棘轮锁住后，电机无法动
    if (trigger_flag == 1)
    {
        shooter_info.hand_shooter_rpm = 0.0f;
    }

    // 触发光电门
    // if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5) && shooter_motor->get_rpm() > 0.0f)
    // {
    //     shooter_info.hand_shooter_rpm = 0.0f;
    // }

    shooter_motor->set_rpm(shooter_info.hand_shooter_rpm);
}

void AutoShooter::allAuto_adjust(float lifter_distance)
{

    switch (shooter_info.shooter_status)
    {
    case auto_lift:
        if (auto_adjust(lifter_distance))
        {

            shooter_info.shooter_status = auto_shoot;
            shooter_motor->set_rpm(0.0f);
        }
        break;
    case auto_shoot:
        shooter_motor->set_rpm(0.0f);
        timecnt++;
        trigger_flag = 1;
        if (timecnt > 5)
        {
            shooter_flag = 1;
            if (timecnt > 20)
            {
                timecnt = 0;
                shooter_info.shooter_status = auto_revert;
                // trigger_flag = 0;
                // shooter_flag = 0;
            }
        }

        break;
    case auto_revert:
        if (auto_adjust(dis_data[0]))
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
bool AutoShooter::auto_adjust(float lifter_distance)
{

    if (trigger_flag == 1)
    {
        trigger_flag = 0;
    }

    if (shooter_flag == 1)
    {
        shooter_flag = 0;
    }
    // 使用梯形规划
    if (planer.isFinished())
    {
        planer.start_plan(plan_info.max_acc, plan_info.max_dcc,
                          plan_info.max_speed, plan_info.inital_speed, plan_info.final_speed,
                          shooter_info.shoot_disdance * 36000, lifter_distance * 36000);
    }

    shooter_motor->set_rpm(-planer.plan(shooter_info.shoot_disdance * 36000));

    // 触发光电门
    //		if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5) && shooter_motor->get_rpm() > 0.0f)
    //   {
    //        shooter_motor->set_rpm(0.0f);
    //    }

    // 到达终点锁住
    if (shooter_info.shoot_disdance > lifter_distance - 0.003f && shooter_info.shoot_disdance < lifter_distance + 0.003f)
    {
        return true;
    }

    return false;
}
void AutoShooter::add_imu(Encoder *encoder_, wit_gyro *wit_imu_)
{
    encoder = encoder_;
    wit_imu = wit_imu_;
}
void AutoShooter::add_trigger(GPIO_TypeDef *stop_port_, uint8_t stop_pin_, GPIO_TypeDef *trigger_port_, uint16_t trigger_pin_, GPIO_TypeDef *shooter_port_, uint16_t shooter_pin_)
{
    trigger_port = trigger_port_;
    trigger_pin = trigger_pin_;
    shooter_port = shooter_port_;
    shooter_pin = shooter_pin_;
    stop_port = stop_port_;
    stop_pin = stop_pin_;
}
void AutoShooter::add_motor(power_motor *shooter_motor_, power_motor *pithcer_motor_)
{
    shooter_motor = shooter_motor_;
    pitcher_motor = pithcer_motor_;
}

void AutoShooter::add_plan_info(float max_acc_, float max_dcc_, float max_speed_, float inital_speed_, float final_speed_)
{

    plan_info.max_acc = max_acc_;
    plan_info.max_dcc = max_dcc_;
    plan_info.max_speed = max_speed_;
    plan_info.inital_speed = inital_speed_;
    plan_info.final_speed = final_speed_;
}
// 获取拉伸距离和俯仰角度
void AutoShooter::get_data()
{
    shooter_info.shoot_disdance = encoder->get_absolute_distance();
    wit_imu->Get_Data();
    shooter_info.shoot_pitch_angle = wit_imu->Pitch_angle;
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

void AutoShooter::check_trigger()
{
    if (trigger_flag == 0)
    {
        HAL_GPIO_WritePin(trigger_port, trigger_pin, GPIO_PIN_RESET);
    }
    else if (trigger_flag == 1)
    {
        HAL_GPIO_WritePin(trigger_port, trigger_pin, GPIO_PIN_SET);
    }
}
void AutoShooter::check_shooter()
{

    if (shooter_flag == 0)
    {
        HAL_GPIO_WritePin(shooter_port, shooter_pin, GPIO_PIN_RESET);
    }
    else if (shooter_flag == 1)
    {
        // 发射时记录当前数据
        // float send_data[2] = {shooter_info.shoot_disdance,shooter_info.shoot_pitch_angle};
        // sendFloatData(1,send_data,2);
        HAL_GPIO_WritePin(shooter_port, shooter_pin, GPIO_PIN_SET);
    }
}

void AutoShooter::set_allAuto(uint8_t index)
{
    shooter_mode = shooter_allAuto;
    shooter_info.shoot_dis = lidar_data[index];
    shooter_info.shooter_status = auto_lift;
}

void AutoShooter::set_halfAuto(uint8_t index)
{
    shooter_mode = shooter_halfAuto;
    shooter_info.shoot_dis = lidar_data[index];
}
void AutoShooter::set_shooter_mode(uint8_t mode)
{
    shooter_mode = static_cast<shooterMode>(mode);
}
void AutoShooter::set_pitcher_mode(uint8_t mode)
{
    pitcher_mode = static_cast<pitcherMode>(mode);
}
