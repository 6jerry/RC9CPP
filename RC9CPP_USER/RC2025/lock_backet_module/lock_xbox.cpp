 #include "lock_xbox.h"

lock_xbox::lock_xbox(imu *imu_ptr_, ros_sensor *ros_ptr_)
{
    imu_ptr = imu_ptr_;
    ros_ptr = ros_ptr_;
}

void lock_xbox::mode_2()
{   
    Vector2D tvel_((3.0f * xbox_msgs.joyLHori_map), (3.0f * xbox_msgs.joyLVert_map));
    set_RobotVel(tvel_, 0);

    lock_vol = lock_basket.PID_ComputeError(ros_ptr->camera_info.vertial_plane_deviation.x);
    
    if(camera_info.vertial_plane_deviation.x < 5.0f){
        set_RobotW(0, 0);
    }
    else{
        set_RobotW(lock_vol, 0);
    }
    
    if(rb_flag){
    
    shoot_dis = dis;
    auto_shooter_ptr->set_auto_byDis(PID, shoot_dis);
    
    rb_flag = 0;
    }
    
}

void lock_xbox::mode_1()
{
    Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));
    set_RobotVel(tvel_, 2.5f);
    set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
}

void lock_xbox::xbox_on()
{
	lock_basket.ConfigAll(0.080f, 0.045f, 0.040f, 0.03f, 0.25f, 18.0f, 20.0f);
    imu_ptr->imu_relocate(0.0f, 0.0f, 0.0f);
    ros_ptr->imu_rst();
}

void lock_xbox::add_AutoShooter(AutoShooter* auto_shooter_)
{
    auto_shooter_ptr = auto_shooter_;
}

