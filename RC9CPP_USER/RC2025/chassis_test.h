#ifndef CHASSIS_TEST_H
#define CHASSIS_TEST_H

#include "TaskManager.h"
#ifdef __cplusplus
extern "C"
{
#endif


#include "auto_lock.h"
#include "gpio.h"
#include "VESC.h"
#include "M3508.h"
#include "robot_chassis.h"
#include "position.h"
#include <cmsis_os2.h>
#include "RC9Protocol.h"
#include "ros_sensor.h"
    void chassis_move_test(void);
    void u8_adjust(void);
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class demo: public ITaskProcessor, public RC9subscriber{
    void process_data();
};
#endif
#endif
