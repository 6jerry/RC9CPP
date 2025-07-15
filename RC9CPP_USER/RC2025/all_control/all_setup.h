#ifndef ALL_SETUP_H
#define ALL_SETUP_H

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
#include "CrsfReceiver.h"
#include "all_controller.h"
#include "auto_shooter.h"

#include "auto_yunball.h"
    void all_setup();
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class demo : public ITaskProcessor
{
public:
    void process_data();
    time_counter test_cnt;

float test_v = 0.0f, test_c = 0.0f;
    float test_x = 0.0f, test_y = 0.0f, test_heading = 0.0f;
};
#endif
#endif
