#include "ros_sensor.h"
#include "Vector2D.h"

//0值去除(传过来的float一定不为0)
static void zero_removal(float * input, const float last_input){
    *input = (*input == 0) ? last_input : *input;
}
void ros_sensor::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount){
		static uint8_t count = 0;
		// 处理相机,雷达数据
        if (byteCount == 20){
            camera_info.vertial_plane_deviation.x = floatData[0];
            camera_info.vertial_plane_deviation.y = floatData[1];  
			ros_radar_loaction.world_pos.x = floatData[2];
            ros_radar_loaction.world_pos.y = -floatData[3]; //把上位机坐标与追踪坐标方向对齐
            ros_radar_loaction.yaw_angle = -floatData[4];			
        }
        // 去除雷达异常值
        zero_removal(&ros_radar_loaction.world_pos.x, previous_world_pos_x); 
        zero_removal(&ros_radar_loaction.world_pos.y, previous_world_pos_y);
        zero_removal(&ros_radar_loaction.yaw_angle, previous_yaw_angle);
		// 阈值限制
        static float max_change = 120.0f; //像素
        if(fabsf(camera_info.vertial_plane_deviation.x - previous_vertial_plane_deviation_x) > max_change){
            camera_info.vertial_plane_deviation.x = previous_vertial_plane_deviation_x + \
                ((camera_info.vertial_plane_deviation.x - previous_vertial_plane_deviation_x) > 0 ? 1 : -1) * max_change / 4;
        }
        if(fabsf(camera_info.vertial_plane_deviation.y - previous_vertial_plane_deviation_y) > max_change){
            camera_info.vertial_plane_deviation.y = previous_vertial_plane_deviation_y + \
                ((camera_info.vertial_plane_deviation.y - previous_vertial_plane_deviation_y) > 0 ? 1 : -1) * max_change / 4;
        }
        previous_vertial_plane_deviation_x = camera_info.vertial_plane_deviation.x;
        previous_vertial_plane_deviation_y = camera_info.vertial_plane_deviation.y;        
        static float max_change_ = 3.0f; //m
        if(fabsf(ros_radar_loaction.world_pos.x - previous_world_pos_x) > max_change_){
            ros_radar_loaction.world_pos.x = previous_world_pos_x + \
                ((ros_radar_loaction.world_pos.x - previous_world_pos_x) > 0 ? 1 : -1) * max_change_ / 4;
        }
        if(fabsf(ros_radar_loaction.world_pos.y - previous_world_pos_y) > max_change_){
            ros_radar_loaction.world_pos.y = previous_world_pos_y + \
                ((ros_radar_loaction.world_pos.y - previous_world_pos_y) > 0 ? 1 : -1) * max_change_ / 4;
        }
        if(fabsf(ros_radar_loaction.yaw_angle - previous_yaw_angle) > max_change_ * 20){
            ros_radar_loaction.yaw_angle = previous_yaw_angle + ((ros_radar_loaction.yaw_angle - previous_yaw_angle) > 0 ? 1 : -1) * max_change_ * 5;
        }
        // 更新上次变量
        previous_world_pos_x = ros_radar_loaction.world_pos.x;
        previous_world_pos_y = ros_radar_loaction.world_pos.y;
        previous_yaw_angle = ros_radar_loaction.yaw_angle;
		coordinate_map(&ros_radar_loaction.world_pos, &real_radar_world_pos, ros_radar_loaction.yaw_angle);
		if(count++ > 25){
		    _action_->imu_relocate(real_radar_world_pos.x, real_radar_world_pos.y, ros_radar_loaction.yaw_angle);
			count = 0;
		}

}

Vector2D ros_sensor::get_world_pos(void){
        return real_radar_world_pos;
}

float ros_sensor::get_heading(void){
    return ros_radar_loaction.yaw_angle;
}

float ros_sensor::get_yaw_rad(){
	return ros_radar_loaction.yaw_angle * 0.01745329252f;
}

void ros_sensor::add_action(action * action_){
    _action_ = action_;
}