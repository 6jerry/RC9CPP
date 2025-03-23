

#include "wh_xbox.h"

void wh_xbox::xbox_init()
{
    btnBConfig = {
            &xbox_msgs.btnB,
            &xbox_msgs.btnB_last,
            &Stop_flag,
            1,
            ButtonActionType::Toggle,
            nullptr
    };

    btnXConfig = {
            &xbox_msgs.btnX,
            &xbox_msgs.btnX_last,
            &Cathcer_flag,
            1,
            ButtonActionType::Toggle,
            nullptr
    };

    btnYConfig = {
            &xbox_msgs.btnY,
            &xbox_msgs.btnY_last,
            &Shoot_flag,
            1,
            ButtonActionType::Toggle,
            nullptr
    };

    btnDirLeftConfig = {
            &xbox_msgs.btnDirLeft,
            &xbox_msgs.btnDirLeft_last,
            &Speed_level,
            2,
            ButtonActionType::Increment,
            nullptr
    };

    btnDirRightConfig = {
            &xbox_msgs.btnDirRight,
            &xbox_msgs.btnDirRight_last,
            &Speed_level,
            2,
            ButtonActionType::Decrement,
            nullptr
    };
}

void wh_xbox::btn_scan() {
    handleButton(btnAConfig);
    handleButton(btnYConfig);
    handleButton(btnDirLeftConfig);
    handleButton(btnDirRightConfig);
}

void wh_xbox::process_data() {
    btn_scan();
    joymap_compute();

}