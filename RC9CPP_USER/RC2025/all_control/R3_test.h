#ifndef __R3_TEST_H__
#define __R3_TEST_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "M3508.h"
#include "vesc.h"
#include "TaskManager.h"
#include "RC9Protocol.h"
#include "encoder.h"
#include "fdcan_device.h"
#include "gpio.h"
#include "robot_chassis.h"
#include "ros_sensor.h"
#include "position.h"
#include "R3_testxbox.h"

    void r3_setup();
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class demo : public ITaskProcessor ,public RC9subscriber
{
public:
    void process_data();

   
};
#endif
#endif
