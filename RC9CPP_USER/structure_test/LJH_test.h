#ifndef LJH_TEST_H
#define LJH_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <cmsis_os2.h>
#include "TaskManager.h"
#include "RC9Protocol.h"
#include "M3508.h"
#include "m6020.h"
#include "vesc.h"
#include "LJH_xbox_test.h"

void LJH_test_setup(void);

#ifdef __cplusplus
}
#endif
#endif