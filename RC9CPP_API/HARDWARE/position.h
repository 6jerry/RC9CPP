#ifndef POSITION_H
#define POSITION_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "RC9Protocol.h"
#include "imu.h"
#include "transformation_of_coordinates.h"
#include <arm_math.h>
#ifdef __cplusplus
}

#endif
#ifdef __cplusplus

class position : public imu, public RC9subscriber
{
public:
    tf tf_;
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;

    Vector2D get_world_pos() override;

    float get_heading() override;

    float get_yaw_rad() override;

    float get_world_pos_x() override;

    float get_world_pos_y() override;

    void imu_relocate(float x, float y, float angle) override;

public:
    Vector2D real_world_pos;                           // 映射后坐标
    Vector2D map_plot = Vector2D(0.20035f, -0.08f); // 原始数据中心偏移点
public:
    Vector2D world_pos;     // 单位m
    float world_yaw = 0.0f; // 单位度
};

#endif
#endif