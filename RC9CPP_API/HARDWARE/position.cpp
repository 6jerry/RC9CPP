#include "position.h"

void position::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    world_pos.x = floatData[0] / 1000.0f;
    world_pos.y = floatData[1] / 1000.0f;
    world_yaw = floatData[2];
	
	// 使用临时坐标系变换
	Vector2D world_pos_ = world_pos;
	world_pos_.y = -world_pos_.y;
	tf_.coordinate_map(&world_pos_, &real_world_pos, 0.204f,  -get_yaw_rad()); //映射到圆心
	// 映射后变换回原坐标系
	real_world_pos.y = -real_world_pos.y;
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