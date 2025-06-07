#ifndef CHASSIS_DEBUG_SETUP_H
#define CHASSIS_DEBUG_SETUP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "M3508.h"
#include "vesc.h"
#include "TaskManager.h"
#include "RC9Protocol.h"
#include "chassis_debug.h"
#include "fdcan_device.h"
#include "gpio.h"
#include "robot_chassis.h"
#include "ros_sensor.h"
#include "position.h"
#include "test_laser.h"
    void chassis_debug_setup();
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus


class demo: public ITaskProcessor, public RC9subscriber{
	public:
		void process_data();
        void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override; // 数据回调函数
    float recive_data[4] = {0};

    Vector2D robot_pos;
    Vector2D robot_v;
};

/*class Robot_communication: public ITaskProcessor, public RC9subscriber{
    public:
        void process_data();
        void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override; // 数据回调函数

    Vector2D data_pos;
    Vector2D data_v;
}*/

#endif
#endif