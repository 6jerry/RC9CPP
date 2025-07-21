 #include "lock_xbox.h"

lock_xbox::lock_xbox(imu *imu_ptr_, ros_sensor *ros_ptr_, Camera *camera_ptr_)
{
    imu_ptr = imu_ptr_;
    ros_ptr = ros_ptr_;
    camera_ptr = camera_ptr_;

    center_point.x = 3.75f;
    center_point.y = -13.9f;

}

void lock_xbox::calc_error()
{
    Vector2D now_point;
    now_point.x = get_world_x();
    now_point.y = get_world_y();

    Vector2D dis = center_point - now_point;

    nor_dir = dis.normalize();
    tan_dir = Vector2D(nor_dir.y, -nor_dir.x).normalize();

    dis_2_center = dis.magnitude();

    center_heading = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;// 角度对准圆心

}

void lock_xbox::mode_1()
{   
	rb_flag = 0;
    DirRight_flag = 0;

    Vector2D tvel_((3.0f * xbox_msgs.joyLHori_map), (3.0f * xbox_msgs.joyLVert_map));
    set_RobotVel(tvel_, 0);

    lock_vol = camera_ptr->lock_basket_vol();
    
    set_RobotW(lock_vol, 0);
    
    if(rb_flag){
    
    set_RobotW(0.0f, 0);
    
//    shoot_dis = dis;
//    auto_shooter_ptr->set_auto_byDis(PID, shoot_dis);
	
	shoot_dis =  camera_ptr->camera_info.vertial_plane_deviation.y / 1000.0f;
	auto_shooter_ptr->camera_auto_byFitter(PID, shoot_dis);
    osDelay(400);
    rb_flag = 0;
    }
}

void lock_xbox::mode_2()
{
    calc_error();
    Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));
    set_RobotVel(tvel_, 2.5f);
   
    if(DirRight_flag){
        yaw_TurnTo(center_heading, 0);
    }
    else{
        set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
    }
}

void lock_xbox::xbox_on()
{
    imu_ptr->imu_relocate(0.0f, 0.0f, 0.0f);
    ros_ptr->rst_radar = true;
	ros_ptr->imu_rst();
}

void lock_xbox::add_AutoShooter(AutoShooter* auto_shooter_)
{
    auto_shooter_ptr = auto_shooter_;
}

