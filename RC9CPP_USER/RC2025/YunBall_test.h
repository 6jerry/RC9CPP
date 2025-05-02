#ifndef YUNBALL_TEST_H
#define YUNBALL_TEST_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <cmsis_os2.h>
#include "RC9Protocol.h"
#include "robot_chassis.h"
#include "M3508.h"
#include "M6020.h"
#include "VESC.h"
#include "rc_test_xbox.h"
#include "debug_xbox.h"
#include "auto_yunball.h"
#include "encoder.h"
#include "wit_gyro.h"
#include "serial_studio.h"
    void yunball_test_setup(void);
    void auto_shooter_setup(void);

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

#endif
#endif
