#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "Serial_device.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class imu
{

public:
    virtual float get_world_pos_x() {};
    virtual float get_world_pos_y() {};

    virtual float get_world_vel_x() {};
    virtual float get_world_vel_y() {};

    virtual float get_world_acc_x() {};
    virtual float get_world_acc_y() {};

public:
    virtual float get_heading() {};
    virtual float get_acc_x() {};
    virtual float get_acc_y() {};

public:
    virtual void imu_rst() {};
    virtual void imu_relocate(float x, float y) {};
};

#endif
#endif
