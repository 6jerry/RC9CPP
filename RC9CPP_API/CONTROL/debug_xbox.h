#ifndef DEBUG_XBOX_H
#define DEBUG_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "xbox.h"
#include "motor.h"
#include "TaskManager.h"

#include "RC9Protocol.h"

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus


class xbox_debug_base : public xbox, public ITaskProcessor
{
public:
    uint8_t mode_flag = 2, start_flag = 0, lb_flag = 0, rb_flag = 0,shoot_title = 0, yunball_title = 0, DirLeft_flag = 0, DirRight_flag = 0, btn_start_flag = 0, btn_select_flag = 1;

    virtual void not_start() {};
    virtual void mode_0() {};
    virtual void mode_1() {};
    virtual void mode_2() {};
    virtual void mode_3() {};
    virtual void mode_4() {};

    virtual void lb_on() {};
    virtual void rb_on() {};
    virtual void lb_off() {};
    virtual void rb_off() {};

    virtual void xbox_on() {};
    virtual void xbox_share() {};

    void process_data();
    void btn_scan();
    void btnconfig_init();

    void btnXBOX_callback() override;
    void btnShare_callback() override;
    void btnY_callback() override;
    virtual void for_btnY();
    xbox_debug_base();
};

#endif
#endif