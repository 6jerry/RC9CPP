#include "ros_sensor.h"
#include "Vector2D.h"

void ros_sensor::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount){
    
    if(id == 1){ // 处理相机数据
        if (byteCount == 8){
            camera_deviation.vertial_plane_deviation.x = floatData[0];
            camera_deviation.vertial_plane_deviation.y = floatData[1];            
        }
        static float max_change = 120.0f;
        if(fabsf(camera_deviation.vertial_plane_deviation.x - previous_vertial_plane_deviation_x) > max_change){
            camera_deviation.vertial_plane_deviation.x = previous_vertial_plane_deviation_x + \
                ((camera_deviation.vertial_plane_deviation.x - previous_vertial_plane_deviation_x) > 0 ? 1 : -1) * max_change / 2;;
        }
        if(fabsf(camera_deviation.vertial_plane_deviation.y - previous_vertial_plane_deviation_y) > max_change){
            camera_deviation.vertial_plane_deviation.y = previous_vertial_plane_deviation_y + \
                ((camera_deviation.vertial_plane_deviation.y - previous_vertial_plane_deviation_y) > 0 ? 1 : -1) * max_change / 2;;
        }
        previous_vertial_plane_deviation_x = camera_deviation.vertial_plane_deviation.x;
        previous_vertial_plane_deviation_y = camera_deviation.vertial_plane_deviation.y;        
    }else if (id == 2){ //处理雷达数据
        if (byteCount == 12){
            ros_radar_loaction.world_pos.x = floatData[0];
            ros_radar_loaction.world_pos.y = -floatData[1]; //把上位机坐标与追踪坐标方向对齐
            ros_radar_loaction.yaw_angle = -floatData[2];
        }
        static float max_change = 4.0f;
        if(fabsf(ros_radar_loaction.world_pos.x - previous_world_pos_x) > max_change){
            ros_radar_loaction.world_pos.x = previous_world_pos_x +  \
                ((ros_radar_loaction.world_pos.y - previous_world_pos_y) > 0 ? 1 : -1) * max_change / 2;
        }
        if(fabsf(ros_radar_loaction.world_pos.y - previous_world_pos_y) > max_change){
            ros_radar_loaction.world_pos.y = previous_world_pos_y +  \ 
                ((ros_radar_loaction.world_pos.y - previous_world_pos_y) > 0 ? 1 : -1) * max_change / 2;
        }
        // 更新上次变量
        previous_world_pos_x = ros_radar_loaction.world_pos.x;
        previous_world_pos_y = ros_radar_loaction.world_pos.y;
        previous_yaw_angle = ros_radar_loaction.yaw_angle;
    }
}

Vector2D ros_sensor::get_world_pos(void){
    return ros_radar_loaction.world_pos;
}

float ros_sensor::get_heading(void){
    return ros_radar_loaction.yaw_angle;
}

float ros_sensor::get_yaw_rad(){
	return ros_radar_loaction.yaw_angle * 0.01745329252f;
}
