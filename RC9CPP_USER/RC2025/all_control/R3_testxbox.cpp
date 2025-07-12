#include "R3_testxbox.h"

R3_xbox::R3_xbox()
{
	center_point.x = 5.8f;
	center_point.y = 0.73f;
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
	; // 角度对准圆心
	center_heading += 180.0f;
	if (center_heading > 180.0f)
	{
		center_heading -= 360.0f;
	}
}

void R3_xbox::mode_1()
{

	if (gate.flag == 1)
	{
		shoot_motor_1->send_rpm(0.0f);
		shoot_motor_2->send_rpm(0.0f);
		rpm1 = shoot_motor_1->get_rpm();
		rpm2 = shoot_motor_2->get_rpm();

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
		shoot_motor_1->send_rpm(target_rpm);
		shoot_motor_2->send_rpm(target_rpm);
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

	dis = encoder->get_distance();
	gate.flag = 0;
	if (abs(dis - 0.0397f) < 0.005f)
	{
		test_flag = 1;
	}
}

void R3_xbox::mode_3()
{

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
