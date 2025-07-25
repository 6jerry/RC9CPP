#include "R3_testxbox.h"

R3_xbox::R3_xbox(imu *imu_ptr_, imu *ros_imu_ptr_, CameraOperation *camera_ops_)
{
	imu_ptr = imu_ptr_;
	ros_imu = ros_imu_ptr_;
	camera_ops = camera_ops_;
	center_point.x = 5.73;
	center_point.y = -0.25f;
    
    //3.5663
    //-14.0976
}
void R3_xbox ::calc_error()
{
	Vector2D now_point;
	now_point.x = get_world_x();
	now_point.y = get_world_y();

	Vector2D dis = center_point - now_point;
	dis_2_center = dis.magnitude();

	nor_dir = dis.normalize();

	center_heading = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;

	//	center_heading += 180.0f;
	//	 	if (center_heading > 180.0f)
	//		{
	//			center_heading -= 360.0f;
	//		}
}

void R3_xbox::mode_1()
{
    calc_error();
	Vector2D target(0.0f, 0.0f);
	set_RobotVel(target, 0);

 	if (abs(center_heading - get_yaw()) < limit_yaw_error && abs(get_chassis_yaw_speed()) < limit_yaw_speed)
	{
		set_RobotW(0.0f, 0);
	}
	else
	{
		yaw_TurnTo(center_heading, 0);
	}

//	if (camera_ops->camera_ready == true)
//	{
//		set_RobotW(0.0f, 0);
//	}
//	else
//	{
//		camera_ops->camera_on();
//	}

	if (DirRight_flag)
	{
		//shoot_dis = camera_ops->camera_Y / 1000.0f;
		// auto_shooter_ptr->camera_auto_byFitter(PID, shoot_dis);

		shooter->set_auto_byrpm(Auto, target_rpm);
		//shooter->set_auto_byFitter(Auto, dis_2_center);

		osDelay(400);
		DirRight_flag = 0;
		//mode_flag = 2;
		camera_ops->camera_off();
	}
}

void R3_xbox::mode_2()
{
	calc_error();
	Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));
	//set_WorldVel_ACCLE(tvel_, 2.5f); // 以后再改
    set_RobotVel_ACCLE(tvel_, 3.0f); 
    camera_ops->camera_off();
	if (btn_select_flag)
	{
		set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
	}
	else
	{
		yaw_TurnTo(center_heading, 0);
	}

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
	if (btn_start_flag)
	{
		reset_swerve();
		btn_start_flag = 0;
	}
}

void R3_xbox::mode_3()
{
	shooter->set_hand(move_rpm * xbox_msgs.joyRVert_map);
	auto_yunball_ptr->control_put_motor(xbox_msgs.joyLVert_map);
	auto_yunball_ptr->control_claw(rb_flag);
	auto_yunball_ptr->control_push(lb_flag);
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
	imu_ptr->imu_rst();
	ros_imu->imu_rst();
}

void R3_xbox::xbox_share()
{
	center_point.x = get_world_x();
	center_point.y = get_world_y();
}