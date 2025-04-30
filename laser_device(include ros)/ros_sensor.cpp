#include "ros_sensor.h"

//0值去除(传过来的float一定不为0)
static void zero_removal(float * input, const float last_input){
    *input = (*input == 0) ? last_input : *input;
}
//差分定位
static void localize_with_diff(Vector2D* pos, Vector2D* origin){
    pos->x -= origin->x;
    pos->y -= origin->y;
}

void ros_sensor::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount){
	static uint8_t count = 0; // 计数用于隔段时间校准action
	// 上一次变量存储
	static float previous_vertial_plane_deviation_x = 0.0f;
    static float previous_vertial_plane_deviation_y = 0.0f;    
    static float previous_world_pos_x = 0.0f;
    static float previous_world_pos_y = 0.0f;
    static float previous_yaw_angle = 0.0f;
	
	// 处理相机,雷达数据
	if (byteCount == 20){
		camera_info.vertial_plane_deviation.x = floatData[0];
		camera_info.vertial_plane_deviation.y = floatData[1];  
		ros_radar_loaction.world_pos.x = floatData[2];
		ros_radar_loaction.world_pos.y = -floatData[3]; //把上位机坐标与追踪坐标方向对齐
		ros_radar_loaction.yaw_angle = -floatData[4];
		if(fabsf(floatData[2]) < 0.02f && fabsf(floatData[3]) < 0.05f){
			map_origin_init_flag = false; //重置映射原点
		}			
	}
	// 去除雷达异常值
	zero_removal(&ros_radar_loaction.world_pos.x, previous_world_pos_x); 
	zero_removal(&ros_radar_loaction.world_pos.y, previous_world_pos_y);
	zero_removal(&ros_radar_loaction.yaw_angle, previous_yaw_angle);
	// 阈值限制
	static float max_change = 80.0f; //像素
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
	// 自旋映射
	coordinate_map(&ros_radar_loaction.world_pos, &real_radar_world_pos, ros_radar_loaction.yaw_angle);
	// 差分原点标定
	if(!map_origin_init_flag){
		map_origin = real_radar_world_pos;
		map_origin_init_flag = true;
	}
	// 差分定位
	localize_with_diff(&real_radar_world_pos, &map_origin);

	if(count++ > 25){
		// _action_->imu_relocate(-real_radar_world_pos.x, real_radar_world_pos.y, _action_->action_info.pos_z_sum);
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