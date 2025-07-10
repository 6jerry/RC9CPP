#include "R3_testxbox.h"


void R3_xbox ::calc_error()
{
	Vector2D now_point;
	now_point.x = get_world_x();
	now_point.y = get_world_y();
	Vector2D dis = {0, 0};

	if (cnt_flag == 0)
	{
		dis = center_point - now_point;
		dis_2_center = dis.magnitude();
	}
	else
	{
		dis = robot_point - now_point;
		dis_2_center = dis.magnitude() - pass_correct_distance;
	}

	nor_dir = dis.normalize();

	center_heading = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;
	; // 角度对准圆心
	center_heading += 180.0f;
	if (center_heading > 180.0f)
	{
		center_heading -= 360.0f;
	}
	center_heading -= offest;
}

void R3_xbox::mode_1()
{

	//	shoot_motor_1->set_current(c);
	//	shoot_motor_2->set_current(c);
	//  shoot_motor_1->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
	//   shoot_motor_2->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
//	if (gate.is_finish() == true)
//	{
//		shoot_motor_1->send_rpm(0.0f);
//		shoot_motor_2->send_rpm(0.0f);
//		//	shoot_motor_1->set_rpm(0.0f);
//		// shoot_motor_2->set_current(shoot_motor_1->get_target_current());
//		mode_flag = 2;
//	}
//	else
//	{
//		shoot_motor_1->send_rpm(target_rpm);
//		shoot_motor_2->send_rpm(target_rpm);
//		//	shoot_motor_2->set_current(shoot_motor_1->get_target_current());
//	}
}

void R3_xbox::mode_2()
{

//	Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));

//	set_RobotVel(tvel_, 0);
//	set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
//	gate.flag = 0;
	 target_rpm = move_rpm * xbox_msgs.joyLVert_map;
	 shoot_motor_1->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
	 shoot_motor_2->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
	// shoot_motor_1->set_rpm(move_rpm * xbox_msgs.joyLVert_map);
	// shoot_motor_2->set_current(shoot_motor_1->get_target_current());

	// shoot_motor_1->send_rpm(0.0f);
	// shoot_motor_2->send_rpm(0.0f);
}

void R3_xbox::mode_3()
{

	calc_error();

	Vector2D target(0.0f, 0.0f);
	set_RobotVel(target, 0);

	if (abs(center_heading - get_yaw()) < limit_yaw_error && abs(get_chassis_yaw_speed()) < limit_yaw_speed)
	{
		set_RobotW(0.0f, 0);
		cnt_flag = 0;
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
}

photogate_shoot::photogate_shoot()
{
}

void photogate_shoot::handleInterrupt()
{

	flag = 1;
}
void photogate_shoot::add_io_interrupt(GPIO_TypeDef *port, uint16_t pin)
{
	port_ = port;
	pin_ = pin;
}
bool photogate_shoot::is_finish()
{
	if (flag)
	{
		flag = 0;
		return true;
	}
	else
	{
		return false;
	}
}