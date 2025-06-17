#ifndef ALL_CONTROLLER_H
#define ALL_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "CrsfReceiver.h"
#include "RC9Protocol.h"
#include "imu.h"
#include "TaskManager.h"
#include "robot_chassis.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class AllController : public CrsfReceiver, public RC9subscriber, public ITaskProcessor, public chassis_user
{
public:
    AllController(UART_HandleTypeDef *huart);
    void process_data();
};

#endif
#endif