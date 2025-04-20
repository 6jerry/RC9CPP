#ifndef CHASSIS_TEST_H
#define CHASSIS_TEST_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "debug_xbox.h"

#include "gpio.h"
#include "VESC.h"
#include "M3508.h"
#include "robot_chassis.h"
#include "Action.h"
#include <cmsis_os2.h>
#include "RC9Protocol.h"

    void chassis_move_test(void);
    void u8_adjust(void);
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

#endif
#endif
