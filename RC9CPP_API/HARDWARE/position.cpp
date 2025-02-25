#include "position.h"

void position::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    world_pos.x = floatData[0] / 1000.0f;
    world_pos.y = floatData[1] / 1000.0f;
    world_yaw = floatData[2];
}

Vector2D position::get_world_pos()
{
    return world_pos;
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
    return world_pos.x;
}

float position::get_world_pos_y()
{
    return world_pos.y;
}