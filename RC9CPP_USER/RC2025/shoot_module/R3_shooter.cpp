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
        flag = 1;
    }
    cont++;

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
            //info.debug_dis = revert_dis;
            //last_tick = HAL_GetTick();
        }

        break;

    case Auto_2:
        // 二分投篮模式
//        if (fabs(info.real_dis - two_dis) < 0.01f)
//        {
//            if (auto_adjust(info.auto_rpm))
//            {
//                mode = Lift;
//                info.debug_dis = revert_dis;
//                last_tick = HAL_GetTick();
//            }
//        }
//        else
//        {
//            lift_adjust(two_dis);
//        }
        break;

    case Lift:
        // lift_adjust(info.debug_dis);
//        if (info.real_dis <= 0.50f)
//        {
//            if (lift_adjust(info.debug_dis))
//            {
//                mode = Keep;
//            }
//        }
//        else
//        {

//            //   m1->send_rpm(-500.0f);
//            //   m2->send_rpm(-500.0f);

//            m1->send_rpm(-800.0f);
//            m2->send_rpm(-800.0f);
//        }

        if(lift_adjust())
        {
              mode = Keep;
          
        }
    
        break;

    case Keep:
       // keep_adjust(revert_dis);
        m1->send_rpm(0.0f);
        m2->send_rpm(0.0f);
        gate->reset();
        gate_down->reset();
        break;
    default:
        m1->send_rpm(0.0f);
        m2->send_rpm(0.0f);
        gate->reset();
        gate_down->reset();
        break;
    }
}

void R3Shooter::keep_adjust(float dis)
{
//    float error = (dis - info.real_dis);
//    info.auto_rpm = dis_control.PID_ComputeError(error);
//    m1->send_rpm(info.auto_rpm);
//    // m2->send_rpm(info.auto_rpm);
//    m2->set_current(0.0f);
}
bool R3Shooter::lift_adjust()
{

//    float error = (dis - info.real_dis);

//    if (fabs(error) < 0.008f)
//    {
//        return true;
//    }
//    else
//    {
//        info.auto_rpm = dis_control.PID_ComputeError(error);
//        m1->send_rpm(info.auto_rpm);
//        // m2->send_rpm(info.auto_rpm);
//        m2->set_current(0.0f);
//        return false;
//    }
    
     if (gate_down->flag)
   {
       m1->send_rpm(0.0f);
       m2->send_rpm(0.0f);

       return true;
   }
   else
   {
       m1->send_rpm(-500.0f);
       m2->send_rpm(-500.0f);
   }
   return false;
}
void R3Shooter::hand_adjust()
{
    m1->send_rpm(test_rpm);
    //m2->send_rpm(test_rpm);
    m2->set_current(0.0f);
}

void R3Shooter::set_hand(float rpm)
{

    mode = Hand;
    test_rpm = rpm;
}

void R3Shooter::set_shooter_mode(uint8_t mode_)
{

    mode = static_cast<R3Mode>(mode_);
}

void R3Shooter::set_auto_byrpm(uint8_t mode_, float rpm)
{
    // 已处于自动状态不可重复设置
    if (mode != Auto || mode != Lift && mode != Auto_2)
    {

        mode = static_cast<R3Mode>(mode_);
        // info.auto_mode = lift;
        info.auto_rpm = rpm;
        start_dis = info.real_dis;
    }
}

int R3Shooter::set_auto_byFitter(uint8_t mode_, float r)
{
    // 自动状态结束才可重新设置
    if (mode != Auto && mode != Lift && mode != Auto_2)
    {
        mode = static_cast<R3Mode>(mode_);
        info.auto_rpm = calc(r);

        if (info.auto_rpm > max)
        {
            info.auto_rpm = max;
        }
    }
    // 0 Stop 停止
    // 2 Auto 发射
    // 3 Lift 归位
    return mode;
}

int R3Shooter::set_auto_byCameraFitter(uint8_t mode_, float camera_r)
{
    // 已处于自动状态不可重复设置
    if (mode != Auto && mode != Lift && mode != Auto_2)
    {
        mode = static_cast<R3Mode>(mode_);
        info.auto_rpm = calc_camera(camera_r);

        if (info.auto_rpm > max)
        {
            info.auto_rpm = max;
        }

   
    }

    // 0 Stop 停止
    // 2 Auto 发射
    // 3 Lift 归位

    return mode;
}

int R3Shooter::set_autoPass_byFitter(uint8_t mode_, float r)
{
    // 自动状态结束才可重新设置
    if (mode != Auto && mode != Lift && mode != Auto_2)
    {
        mode = static_cast<R3Mode>(mode_);
        info.auto_rpm = calc_pass(r);

        if (info.auto_rpm > max)
        {
            info.auto_rpm = max;
        }
     
    }

    // 0 Stop 停止
    // 2 Auto 发射
    // 3 Lift 归位
    return mode;
}

void R3Shooter::set_lift(float dis)
{
    mode = Lift;
    info.debug_dis = dis;
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

    //编码器发射
//      test_rpm = target_rpm;
//      // v2 = 2ax;
//      if (fabs(end_dis - info.real_dis) < zone || (info.real_dis > end_dis))
//      {
//          m1->send_rpm(0.0f);
//          m2->send_rpm(0.0f);

//          return true;
//      }
//      else
//      {
//          m1->send_rpm(test_rpm);
//          m2->send_rpm(test_rpm);
//      }

    // 光电门发射
   test_rpm = target_rpm;
   //test_c = target_rpm;
   if (gate->flag)
   {
       m1->send_rpm(0.0f);
       m2->send_rpm(0.0f);

       return true;
   }
   else
   {
       m1->send_rpm(test_rpm);
       m2->send_rpm(test_rpm);
       
   //    m1->set_current(test_c);
   //    m2->set_current(test_c);
   }

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
//        6.2000f,  // r³ 系数
//        -71.8172f,  // r² 系数
//        514.4328f, // r 系数
//        468.7465f  // 常数项
//    };

//    v = coeffs[0];
//    v = v * r + coeffs[1];
//    v = v * r + coeffs[2];
//    v = v * r + coeffs[3];
    //-----------------------------------------------

    //---------------------幂、指数、直线拟合-----------------
    // v = a * pow(r, b) + c * logf(r + 1);
     v = 501.1644f * pow(r,0.7165f) + 432.1382f;
    // v = a*exp(b*r) + c ;
    //v = 255.7915f * r + 766.6459f
    // v = a*(1-exp(b*r)) + c;
    //-----------------------------------------------

    // v = a * r * r * (1 + b * r) + c;

    return v + offset;
}

float R3Shooter::calc_camera(float camera_r)
{
    // v = a * pow(camera_r, b) + c;
    // v = a*exp(b*camera_r) + c;
    //---------------------多项式拟合-----------------
/*     const float coeffs[] = {
        0.0f,  // r³ 系数
        56.3513f,  // r² 系数
        -598.8639f, // r 系数
        3066.4466f  // 常数项
    };

    v = coeffs[0];
    v = v * camera_r + coeffs[1];
    v = v * camera_r + coeffs[2];
    v = v * camera_r + coeffs[3]; */
    //v = -500.3285f*logf(0.6645f*camera_r + 1) + 1705.5149f;
    v = 2317.0704f * pow(camera_r,-0.9365f) + 958.9916f;
    return v += camera_offset;
}

float R3Shooter::calc_pass(float pass_r)
{
    v = 750.6998 * pow(pass_r, 0.5571);
    return v += pass_offset;
}
void R3Shooter::get_data()
{
    info.real_dis = encoder->get_distance();

    //    flag = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);

    //    if (flag == 0)
    //    {
    //        if (HAL_GetTick() - last_tick2 > 100)
    //        {
    //            encoder->send_reset();
    //        }
    //    }
    //    else
    //    {
    //        last_tick2 = HAL_GetTick();
    //    }
}

float R3Shooter::CalculateDistance(float camera_r)
{
    const float coeffs[] = {
        0.0f,  // r³ 系数
        0.2582f,  // r² 系数
        -2.6388f, // r 系数
        9.5685f  // 常数项
    };
    camera_to_dis = coeffs[0];
    camera_to_dis = camera_to_dis * camera_r + coeffs[1];
    camera_to_dis = camera_to_dis * camera_r + coeffs[2];
    camera_to_dis = camera_to_dis * camera_r + coeffs[3];
    return camera_to_dis;
}