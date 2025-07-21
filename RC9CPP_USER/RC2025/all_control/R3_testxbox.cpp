#include "R3_testxbox.h"

R3_xbox::R3_xbox(imu *imu_ptr_)
{
	// imu_ptr = imu_ptr_;
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

void R3_xbox::mode_1()
{
	//	calc_error();

	//	Vector2D target(0.0f, 0.0f);
	//	set_RobotVel(target, 0);

	//	if (abs(center_heading - get_yaw()) < limit_yaw_error && abs(get_chassis_yaw_speed()) < limit_yaw_speed)
	//	{
	// set_RobotW(0.0f, 0);
	// Shoot(calc_rpm(dis_2_center));
	//  Shoot(target_rpm);
	//  mode_flag = 2;
	//	}
	//	else
	//	{
	//		yaw_TurnTo(center_heading, 0);
	//	}

	if (DirRight_flag == 1)
	{
		shooter->set_auto_byrpm(Auto, target_rpm);
		DirRight_flag = 0;
	}
}

void R3_xbox::mode_2()
{

	Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));

	set_RobotVel(tvel_, 0);
	set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);

	shooter->set_hand(move_rpm * xbox_msgs.joyRVert_map);
	// shoot_motor_1->send_rpm(move_rpm * xbox_msgs.joyRVert_map);
	// shoot_motor_2->send_rpm(move_rpm * xbox_msgs.joyRVert_map);
}

void R3_xbox::mode_3()
{
	/*Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));
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
	}*/
	/*Shoot(target_rpm_test);
	auto_yunball_ptr->control_put_motor(xbox_msgs.joyRVert_map);*/
	if (DirRight_flag)
	{
		auto_yunball_ptr->start_putball();
		DirRight_flag = 0;
	}
	if (DirLeft_flag)
	{
		auto_yunball_ptr->start_yunball();
		DirLeft_flag = 0;
	}
}

void R3_xbox::not_start()
{
	shooter->set_shooter_mode(Stop);
	Vector2D zero(0, 0);
	set_RobotVel(zero, 0);
	set_RobotW(0, 0);
}

void R3_xbox::add_autoyunball(AutoYunballR3 *auto_yunball_)
{
	auto_yunball_ptr = auto_yunball_;
}
void R3_xbox::add_R3shooter(R3Shooter *shooter_)
{

	shooter = shooter_;
}

void R3_xbox::xbox_on()
{
	imu_ptr->imu_relocate(0.0f, 0.0f, 0.0f);
}

void R3_xbox::xbox_on()
{
	// post->imu_rst();
	imu_ptr->imu_rst();
	ros_imu->imu_rst();
}

void R3_xbox::xbox_share()
{
	center_point.x = get_world_x();
	center_point.y = get_world_y();
}