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
    virtual float get_heading() {};
    virtual float get_acc_x() {};
    virtual float get_acc_y() {};

    virtual float get_yaw_rad() {};

    virtual void imu_rst() {};

    virtual void imu_reset_heading(float reheading) {};

    virtual uint32_t get_update_time() {};

    virtual float get_yaw_speed_rad() {};
};

#endif

#endif
