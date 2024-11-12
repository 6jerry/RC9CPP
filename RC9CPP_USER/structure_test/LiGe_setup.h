#ifndef LIGE_SETUP_H
#define LIGE_SETUP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "task.h"
// #include <stdint.h>
#include <cmsis_os.h>
#include "RC9Protocol.h"
#include "usart.h"
#include "TaskManager.h"

#include "r2n_xbox.h"

#include "go1.h"

    void lige_setup(void);
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

#endif
#endif
