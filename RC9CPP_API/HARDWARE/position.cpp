#include "position.h"

void position::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{	
	static float last_yaw = 0.0f;
    world_pos.x = floatData[0] / 1000.0f;
    world_pos.y = floatData[1] / 1000.0f;
    world_yaw = floatData[2];
    // global->x = x_offset + local->x * cos_y + local->y * sin_y;
    // global->y = y_offset - local->x * sin_y + local->y * cos_y;
    float deltaxx = map_plot.x * arm_cos_f32(get_yaw_rad()) - map_plot.y * arm_sin_f32(get_yaw_rad()); // 逆变换回imu位置
    float deltayy = map_plot.x * arm_sin_f32(get_yaw_rad()) + map_plot.y * arm_cos_f32(get_yaw_rad());

    real_world_pos.x = world_pos.x - deltaxx; // 逆变换回imu位置
    real_world_pos.y = world_pos.y - deltayy;

    /*
    // ????????????任
    Vector2D world_pos_ = world_pos;
    world_pos_.y = -world_pos_.y;
    tf_.coordinate_map(&world_pos, &real_world_pos, 0.225f, -get_yaw_rad(), map_plot); //??????
    // ???????
    if(!tf_.map_origin_init_flag){
        tf_.map_origin = real_world_pos;
        tf_.map_origin_init_flag = true;
    }
    tf_.localize_with_diff(&real_world_pos);
    // ????y????任????????
    real_world_pos.y = -real_world_pos.y;
    */
}

Vector2D position::get_world_pos()
{
    return real_world_pos;
}

float position::get_heading()
{
    return world_yaw;
}

float position::get_yaw_rad()
{
    return world_yaw * 0.0174532925f;
}

float position::get_world_pos_x()
{
    return real_world_pos.x;
}

float position::get_world_pos_y()
{
    return real_world_pos.y;
}

void position::imu_relocate(float x, float y, float angle)
{   

    float send_datas[2] = {x, y};

    sendFloatData(1, send_datas, 2);
}