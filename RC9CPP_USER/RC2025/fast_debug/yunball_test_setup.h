#ifndef YUNBALL_TEST_SETUP_H
#define YUNBALL_TEST_SETUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "M3508.h"
#include "vesc.h"
#include "TaskManager.h"
#include "RC9Protocol.h"
#include "chassis_debug.h"
#include "fdcan_device.h"
#include "yunball_test_xbox.h"
#include "auto_yunball.h"
#include "position.h"
//#include "robot_chassis.h"
void yunball_test_setup(void);

#ifdef __cplusplus
}
#endif

#endif /* SHOOTBALL_TEST_SETUP_H */