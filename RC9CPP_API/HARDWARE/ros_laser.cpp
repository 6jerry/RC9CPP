#include "ros_laser.h"
#include "Vector2D.h"

void ros_laser::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount){
    if (byteCount == 12){
        ros_laser_loaction.world_pos.x = floatData[0];
        ros_laser_loaction.world_pos.y = -floatData[1];
        ros_laser_loaction.yaw_angle = -floatData[2];
    }
    static float max_change = 4.0f;
    if(fabs(ros_laser_loaction.world_pos.x - previous_world_pos_x) > max_change){
			if((ros_laser_loaction.world_pos.x - previous_world_pos_x) > 0){
				ros_laser_loaction.world_pos.x = previous_world_pos_x + max_change / 2;
			}else{
				ros_laser_loaction.world_pos.x = previous_world_pos_x - max_change / 2;
			}
        
    }
    else if(fabs(ros_laser_loaction.world_pos.y - previous_world_pos_y) > max_change){
			if((ros_laser_loaction.world_pos.y - previous_world_pos_y) > 0){
       	 ros_laser_loaction.world_pos.y = previous_world_pos_y + max_change / 2;		
			}else{
				 ros_laser_loaction.world_pos.y = previous_world_pos_y - max_change / 2;		
			}
    }
    // 更新上次变量
    previous_world_pos_x = ros_laser_loaction.world_pos.x;
    previous_world_pos_y = ros_laser_loaction.world_pos.y;
    previous_yaw_angle = ros_laser_loaction.yaw_angle;

}

Vector2D ros_laser::get_world_pos(void){
		to_action_world_pos.x = ros_laser_loaction.world_pos.x - (1-cos((ros_laser_loaction.yaw_angle+theta) / 180.0f * 3.1415926535897f)) * r;
		to_action_world_pos.y = ros_laser_loaction.world_pos.y - sin((ros_laser_loaction.yaw_angle+theta) / 180.0f * 3.1415926535897f) * r;
    return to_action_world_pos;
}

// float ros_laser::get_yaw_angle(void){
//     return ros_laser_loaction.yaw_angle;
// }

float ros_laser::get_heading(void){
    return ros_laser_loaction.yaw_angle;
}

float ros_laser::get_yaw_rad(){
	return ros_laser_loaction.yaw_angle / 180.0f * 3.1415926535897f;
}
