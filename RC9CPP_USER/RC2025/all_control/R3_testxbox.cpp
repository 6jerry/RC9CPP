#include "R3_testxbox.h"

R3_xbox::R3_xbox(imu *imu_ptr_)
{
	imu_ptr = imu_ptr_;
	center_point.x = -5.222f;
	center_point.y = 0.287490f;

	// -5.2320
	// -2.3530
	// center_point.x = 3.000f;
	// center_point.y = 14.355f;
}
void R3_xbox ::calc_error()
{
	Vector2D now_point;
	now_point.x = get_world_x();
	now_point.y = get_world_y();
	Vector2D dis = {0, 0};

	dis = center_point - now_point;
	dis_2_center = dis.magnitude();

	nor_dir = dis.normalize();

	center_heading = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;

	center_heading += 180.0f;
	if (center_heading > 180.0f)
	{
		center_heading -= 360.0f;
	}
}
float R3_xbox::calc_rpm(float dis)
{

	const float coeffs[] = {
		91.4439f,	// r³ 系数
		-580.3618f, // r² 系数
		1437.0013f, // r 系数
		-38.8659f	// 常数项
	};
	dis += offest;
	target_rpm = coeffs[0];
	target_rpm = target_rpm * dis + coeffs[1];
	target_rpm = target_rpm * dis + coeffs[2];
	target_rpm = target_rpm * dis + coeffs[3];
	// target_rpm = a * exp(b * dis) + c;

	if (target_rpm > 2300.0f)
	{
		target_rpm = 2300.0f;
	}
	return target_rpm;
}

void R3_xbox::mode_1()
{
	calc_error();

	Vector2D target(0.0f, 0.0f);
	set_RobotVel(target, 0);

	if (abs(center_heading - get_yaw()) < limit_yaw_error && abs(get_chassis_yaw_speed()) < limit_yaw_speed)
	{
		set_RobotW(0.0f, 0);
		Shoot(calc_rpm(dis_2_center));
		// Shoot(target_rpm);
		// mode_flag = 2;
	}
	else
	{
		yaw_TurnTo(center_heading, 0);
	}
}

void R3_xbox::mode_2()
{

	Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));

	set_RobotVel(tvel_, 0);
	set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);

	gate.flag = 0;
	gate_down.flag = 0;
	gate.cont = 0;
	gate_down.cont = 0;
	shoot_motor_1->send_rpm(move_rpm * xbox_msgs.joyRVert_map);
	shoot_motor_2->send_rpm(move_rpm * xbox_msgs.joyRVert_map);
}

void R3_xbox::mode_3()
{
	Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));
	set_RobotVel(tvel_, 0);

	calc_error();

	Vector2D target(0.0f, 0.0f);
	set_RobotVel(target, 0);

	if (abs(center_heading - get_yaw()) < limit_yaw_error && abs(get_chassis_yaw_speed()) < limit_yaw_speed)
	{
		set_RobotW(0.0f, 0);
		mode_flag = 2;
	}
	else
	{
		yaw_TurnTo(center_heading, 0);
	}
}

void R3_xbox::not_start()
{
	shoot_motor_1->send_rpm(0.0f);
	shoot_motor_2->send_rpm(0.0f);
	Vector2D zero(0, 0);
	set_RobotVel(zero, 0);
	set_RobotW(0, 0);
}

void R3_xbox::init(power_motor *shoot_motor_1_, power_motor *shoot_motor_2_, Encoder *encoder_)
{
	shoot_motor_1 = shoot_motor_1_;
	shoot_motor_2 = shoot_motor_2_;
	encoder = encoder_;
	gate.set_motors(shoot_motor_1_, shoot_motor_2_);
	gate_down.set_motors(shoot_motor_1_, shoot_motor_2_);
}

void R3_xbox::xbox_on()
{
	imu_ptr->imu_relocate(0.0f, 0.0f, 0.0f);
}
photogate_shoot::photogate_shoot()
{
}

photogate_shoot_down::photogate_shoot_down()
{
}

void photogate_shoot::handleInterrupt()
{
	if (cont++ == 0)
	{
		rpm1 = motor1->get_rpm();
		rpm2 = motor2->get_rpm();
		motor1->send_rpm(0.0f);
		motor2->send_rpm(0.0f);
	}
	flag = 1;
}

void photogate_shoot_down::handleInterrupt()
{
	if (cont++)
	{
		motor1->send_rpm(0.0f);
		motor2->send_rpm(0.0f);
	}
	flag = 1;
}

void photogate_shoot::add_io_interrupt(GPIO_TypeDef *port, uint16_t pin)
{
	port_ = port;
	pin_ = pin;
}

void photogate_shoot_down::add_io_interrupt(GPIO_TypeDef *port, uint16_t pin)
{
	port_ = port;
	pin_ = pin;
}

void R3_xbox::Shoot(float rpm)
{

	if (gate.flag)
	{
		shoot_motor_1->send_rpm(0.0f);
		shoot_motor_2->send_rpm(0.0f);

		if (HAL_GetTick() - last_tick > 150)
		{
			shoot_motor_1->send_rpm(-1000.0f);
			shoot_motor_2->send_rpm(-1000.0f);
			if (gate_down.flag)
			{
				shoot_motor_1->send_rpm(0.0f);
				shoot_motor_2->send_rpm(0.0f);
				mode_flag = 2;
			}
		}
	}
	else
	{
		last_tick = HAL_GetTick();
		gate_down.flag = 0;
		shoot_motor_1->send_rpm(rpm);
		shoot_motor_2->send_rpm(rpm);
	}
}