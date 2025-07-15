#ifndef SUPERYUNBALL_SETUP_H
#define SUPERYUNBALL_SETUP_H

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
#include "superyunball.h"
#include "ros_sensor.h"
#include "position.h"

    void yunball_setup();
#ifdef __cplusplus
}
#endif
#endif