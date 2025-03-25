//
// Created by 15828 on 2025/3/20.
//

#ifndef RC9CPP_WH_SETUP_H
#define RC9CPP_WH_SETUP_H
#ifdef __cplusplus
extern "C"
{
#endif

#include <cmsis_os2.h>
#include "RC9Protocol.h"
#include "TaskManager.h"
#include "robot_chassis.h"
#include "M3508.h"
#include "M6020.h"
#include "VESC.h"
#include "gpio.h"
#include "wh_xbox.h"

void wh_setup(void);
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus


#endif

#endif //RC9CPP_WH_SETUP_H
