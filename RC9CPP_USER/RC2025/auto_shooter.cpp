#include "auto_shooter.h"
/**
 * @file auto_shooter
 * @author: yanqy
 * @brief：自动射球
 * 手动模式（shooter_hand）：
 * - 用户通过外部输入直接控制射球电机的转速。
 * - 在该模式下，系统会检测触发信号并执行相应的动作。
 *
 * 全自动模式（shooter_allAuto）：
 * - 系统根据预设数据自动完成拉伸、发射、复位等操作。
 * - 修改lidar_data中的预设数据，即可修改拉伸距离。单位为米。
 * - 包含四个状态：auto_lift（拉伸）、auto_shoot（发射）、auto_revert（复位）、auto_finish（停止）。
 * - 可通过 set_allAuto() 接口设置目标拉伸距离，并启动全自动流程。
 *
 * 俯仰目前已废弃，关于pitcher的代码都不用看
 * 2025/4/30
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
        allAuto_adjust();
        break;
     case shooter_pulldata:
        pulldata_adjust();
        break;
    case shooter_debug:
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
bool AutoShooter::pulldata_adjust()
{
     
     if(auto_adjust(shooter_info.shoot_dis ))
     {
         shooter_mode = shooter_stop;
         trigger_flag = 1;
         return true;
     }
     return false;

}
// 手动模式
void AutoShooter::hand_adjust()
{
    // 棘轮锁住后，电机不能动
    if (trigger_flag == 1)
    {
        shooter_info.hand_shooter_rpm = 0.0f;
    }

    // 触发光电门
    if (HAL_GPIO_ReadPin(stop_port, stop_pin) && shooter_motor->get_rpm() > 0.0f)
    {
        shooter_info.hand_shooter_rpm = 0.0f;
    }

    shooter_motor->set_rpm(shooter_info.hand_shooter_rpm);
}
// 全自动模式
void AutoShooter::allAuto_adjust()
{

    switch (shooter_info.shooter_status)
    {
    case auto_lift:
        if (auto_adjust(shooter_info.shoot_dis))
        {

            shooter_info.shooter_status = auto_shoot;
            shooter_motor->set_rpm(0.0f);
        }
        break;
    case auto_shoot:
        shooter_motor->set_rpm(0.0f);
        timecnt++;
        trigger_flag = 1;
        if (timecnt > 2)
        {
            shooter_flag = 1;
            if (timecnt > 5)
            {
                timecnt = 0;
                shooter_info.shooter_status = auto_revert;
            }
        }

        break;
    case auto_revert:
        if (auto_adjust(revert_dis))
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
// 自动拉伸
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
    if (plan_flag == 0)
    {
        planer.start_plan(plan_info.max_acc, plan_info.max_dcc,
                          plan_info.max_speed, plan_info.inital_speed, plan_info.final_speed,
                          shooter_info.shoot_disdance * 36000, lifter_distance * 36000);
        plan_flag = 1;
    }

    shooter_motor->set_rpm(-planer.plan(shooter_info.shoot_disdance * 36000));

    // 触发光电门
    if (HAL_GPIO_ReadPin(stop_port, stop_pin) && shooter_motor->get_rpm() >= 0.0f)
    {
        shooter_motor->set_rpm(0.0f);
    }

    // 到达终点锁住
    if (abs(shooter_info.shoot_disdance - lifter_distance) < 0.003f)
    {
        plan_flag = 0;
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
// 添加梯形规划信息
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
    // wit_imu->Get_Data();
    // shooter_info.shoot_pitch_angle = wit_imu->Pitch_angle;
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
    HAL_GPIO_WritePin(trigger_port, trigger_pin, (trigger_flag == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void AutoShooter::check_shooter()
{
    HAL_GPIO_WritePin(shooter_port, shooter_pin, (shooter_flag == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    if (shooter_flag == 1)
    {
        // 发射时记录当前数据
        // float send_data[2] = {shooter_info.shoot_disdance,shooter_info.shoot_pitch_angle};
        // sendFloatData(1,send_data,2);
    }
}

void AutoShooter::set_Auto(uint8_t index, uint8_t mode, uint8_t type)
{
    shooterMode temp_mode = static_cast<shooterMode>(mode);
    shooter_mode = temp_mode;
    
    if( type == 0)
    {
         shooter_info.shoot_dis = circle_data[index];
    }
    else if( type == 1)
    {
        shooter_info.shoot_dis =  data[index];

    }
    
    

    if (temp_mode = shooter_allAuto)
    {
        shooter_info.shooter_status = auto_lift;
    }

    
}
void AutoShooter::set_shooter_mode(uint8_t mode)
{
    shooter_mode = static_cast<shooterMode>(mode);
}
void AutoShooter::set_pitcher_mode(uint8_t mode)
{
    pitcher_mode = static_cast<pitcherMode>(mode);
}
