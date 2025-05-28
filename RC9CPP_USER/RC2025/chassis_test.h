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
