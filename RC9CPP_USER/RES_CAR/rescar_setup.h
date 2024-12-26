#ifndef RESCAR_SETUP_H
#define RESCAR_SETUP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <cmsis_os2.h>
#include "RC9Protocol.h"

#include "TaskManager.h"

#include "chassis.h"
#include "tb6612.h"
#include "fdi.h"

    void rescar_setup(void);
#ifdef __cplusplus
}

#endif
#ifdef __cplusplus

class shit : public ITaskProcessor
{
public:
    void process_data();
};

#endif
#endif