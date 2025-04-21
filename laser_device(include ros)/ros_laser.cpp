#include "ros_laser.h"
#include "Vector2D.h"

void ros_laser::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount){
    if (byteCount == 12){
        ros_laser_loaction.world_pos.x = floatData[0];
        ros_laser_loaction.world_pos.y = -floatData[1]; //把上位机坐标与追踪对齐
        ros_laser_loaction.yaw_angle = -floatData[2];
    }
    static float max_change = 4.0f;
    if(fabs(ros_laser_loaction.world_pos.x - previous_world_pos_x) > max_change){
        ros_laser_loaction.world_pos.x = previous_world_pos_x + ((ros_laser_loaction.world_pos.x - previous_world_pos_x)  \
        / fabs(ros_laser_loaction.world_pos.x - previous_world_pos_x))  * max_change / 2;
    }
    else if(fabs(ros_laser_loaction.world_pos.y - previous_world_pos_y) > max_change){
        ros_laser_loaction.world_pos.y = previous_world_pos_y + ((ros_laser_loaction.world_pos.y - previous_world_pos_y)  \ 
        / fabs(ros_laser_loaction.world_pos.y - previous_world_pos_y)) * max_change / 2;
    }
    // 更新上次变量
    previous_world_pos_x = ros_laser_loaction.world_pos.x;
    previous_world_pos_y = ros_laser_loaction.world_pos.y;
    previous_yaw_angle = ros_laser_loaction.yaw_angle;

}

Vector2D ros_laser::get_world_pos(void){
    //自旋变换
    float theta = (angle_offset + ros_laser_loaction.yaw_angle) * 0.01745329252f;
    float x_offset = center_offset * (1 - cos(theta));
    float y_offset = -center_offset * sin(theta);
    ros_laser_loaction.world_pos.x += x_offset;
    ros_laser_loaction.world_pos.y += y_offset;
    return ros_laser_loaction.world_pos;
}

// float ros_laser::get_yaw_angle(void){
//     return ros_laser_loaction.yaw_angle;
// }

float ros_laser::get_heading(void){
    return ros_laser_loaction.yaw_angle;
}

float ros_laser::get_yaw_rad(){
	return ros_laser_loaction.yaw_angle * 0.01745329252f;
}
