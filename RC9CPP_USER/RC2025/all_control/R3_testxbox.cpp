#include "R3_testxbox.h"

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
	//	//	shoot_motor_1->set_rpm(0.0f);
	//	 // shoot_motor_2->set_current(shoot_motor_1->get_target_current());
	//		mode_flag = 2;
	//	}
	//	else
	//	{
	//		shoot_motor_1->send_rpm(target_rpm);
	//		shoot_motor_2->send_rpm(target_rpm);
	//	//	shoot_motor_2->set_current(shoot_motor_1->get_target_current());
	//	}
}

void R3_xbox::mode_2()
{

	//	 Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));

	//	 set_RobotVel(tvel_, 0);
	//	 set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
	//	gate.flag = 0;
	target_rpm = move_rpm * xbox_msgs.joyLVert_map;
	shoot_motor_1->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
	shoot_motor_2->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
	// shoot_motor_1->set_rpm(move_rpm * xbox_msgs.joyLVert_map);
	// shoot_motor_2->set_current(shoot_motor_1->get_target_current());

	// shoot_motor_1->send_rpm(0.0f);
	// shoot_motor_2->send_rpm(0.0f);
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