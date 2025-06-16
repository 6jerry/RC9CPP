#ifndef LOCK_BASKET_H
#define LOCK_BASKET_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "TaskManager.h"
#include <arm_math.h>
#include "PID.h"
#include "ros_sensor.h"


#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class LockBasket
{
private:     
    ros_sensor *ros_ptr; // 导入雷达的指针
    float lock_vol;
    pid lock_basket_pid; // 锁框

public:
    LockBasket(ros_sensor *ros_ptr_);

    float lock_basket_vol ();
  
   
    
};

#endif
#endif