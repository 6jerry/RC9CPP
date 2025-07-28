#include "camera.h"

Camera::Camera()
{
	///错误码
    err_id = ERR_DEVICE_CAMERA; //0x08
}

void Camera::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{

	// 上一次变量存储
	static float previous_vertial_plane_deviation_x = 0.0f;
	static float previous_vertial_plane_deviation_y = 0.0f;

	//处理相机数据
	if(id == 1 && byteCount == 8){
		camera_info.vertial_plane_deviation.x = floatData[0];
		camera_info.vertial_plane_deviation.y = floatData[1];
	}
	else{
		return;
	}
	// 阈值限制
	static float max_change = 80.0f; // 像素
	if (fabsf(camera_info.vertial_plane_deviation.x - previous_vertial_plane_deviation_x) > max_change)
	{
		camera_info.vertial_plane_deviation.x = previous_vertial_plane_deviation_x +
												((camera_info.vertial_plane_deviation.x - previous_vertial_plane_deviation_x) > 0 ? 1 : -1) * max_change / 4;
	}
	if (fabsf(camera_info.vertial_plane_deviation.y - previous_vertial_plane_deviation_y) > max_change)
	{
		camera_info.vertial_plane_deviation.y = previous_vertial_plane_deviation_y +
												((camera_info.vertial_plane_deviation.y - previous_vertial_plane_deviation_y) > 0 ? 1 : -1) * max_change / 4;
	}
	previous_vertial_plane_deviation_x = camera_info.vertial_plane_deviation.x;
	previous_vertial_plane_deviation_y = camera_info.vertial_plane_deviation.y;
	
}

err_code Camera::check_error()
{
    if(camera_info.vertial_plane_deviation.x == -721.0f 
        && camera_info.vertial_plane_deviation.y == 540.0f){

            gaze_flag = false;
            return ERR_CODE_ABNORMAL;
        }
	else if(camera_info.vertial_plane_deviation.x == 0.0f 
        && camera_info.vertial_plane_deviation.y == 0.0f){

			gaze_flag = false;
            return ERR_CODE_CONNECT_FAIL;
		}
    else{

         gaze_flag = true;
        return ERR_CODE_WORK_SUCCESS;
    }  
}

CameraOperation::CameraOperation(Camera *camera_ptr_)
{
    lock_basket_pid.ConfigAll(0.080f, 0.045f, 0.0423f, 0.03f, 0.219f, 2.0f, 20.0f);
	camera_ptr = camera_ptr_;
}

void CameraOperation::process_data()
{
	switch (camera_mode)
    {
    case camera_suspend:
		camera_Y = 0.0f;
		camera_ready = false;
		
        break;
    case camera_start:
		set_RobotW(lock_basket_vol(), 0);

        break;

    case camera_finish:
        set_RobotW(0.0f, 0);
		camera_ready = true;

        break;

    default:
        break;
    }
}

void CameraOperation::camera_on()
{
	if(camera_mode == camera_suspend){
	camera_mode = camera_start;

	}
}

void CameraOperation::camera_off()
{
	if(camera_mode != camera_suspend){
	camera_mode = camera_suspend;

	}
}

float CameraOperation::lock_basket_vol()
{
    lock_vol = lock_basket_pid.PID_ComputeError(camera_ptr->camera_info.vertial_plane_deviation.x);
    //偏置改完相机位置记得改，现在锁0
    
    if(fabs(camera_ptr->camera_info.vertial_plane_deviation.x) < deadlock && lock_vol < 0.08f){			//R1
		camera_Y = camera_ptr->camera_info.vertial_plane_deviation.y;
		camera_mode = camera_finish;
        return 0;
    }
    else{
        return lock_vol;
    }
}