//
// Created by 15828 on 2025/3/20.
//

#ifndef RC9CPP_WH_XBOX_H
#define RC9CPP_WH_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "xbox.h"
#include "motor.h"
#include "TaskManager.h"
#include "chassis.h"
#include "wh_setup.h"
#include <arm_math.h>
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class wh_xbox : public xbox, public ITaskProcessor
{
private:
    uint8_t Stop_flag=0,Cathcer_flag=0,Speed_level=0,Shoot_flag=0;
public:
    void process_data();
    void xbox_init();
    void btn_scan();
};

#endif
#endif //RC9CPP_WH_XBOX_H
