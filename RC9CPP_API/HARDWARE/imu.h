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
};

#endif

#endif
