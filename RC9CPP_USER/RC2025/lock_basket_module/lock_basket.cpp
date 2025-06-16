#include "lock_basket.h"

LockBasket::LockBasket(ros_sensor *ros_ptr_)
{
    ros_ptr = ros_ptr_;
    lock_basket_pid.ConfigAll(0.080f, 0.045f, 0.040f, 0.03f, 0.25f, 18.0f, 20.0f);
}

float LockBasket::lock_basket_vol()
{
    lock_vol = lock_basket_pid.PID_ComputeError(ros_ptr->camera_info.vertial_plane_deviation.x);

    if(ros_ptr->camera_info.vertial_plane_deviation.x < 4.0f){
        return 0;
    }
    else{
        return lock_vol;
    }
    
}
