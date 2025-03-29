
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
            &Catcher_flag,
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

void wh_xbox::load_motor(power_motor *lifter_motor_, power_motor *turn_motor_, power_motor *shooter_motor_, power_motor *pithcer_motor_)
{
    lifter_motor = lifter_motor_;
    turn_motor = turn_motor_;
    shooter_motor = shooter_motor_;
    pithcer_motor = pithcer_motor_;
}

void wh_xbox::load_pin(GPIO_TypeDef *Catcher_port_,GPIO_TypeDef *Shooter_port_,uint16_t Catcher_pin_,uint16_t Shooter_pin_)
{
    Catcher_port=Catcher_port_;
    Catcher_pin=Catcher_pin_;
    Shooter_port=Shooter_port_;
    Shooter_pin=Shooter_pin_;
}

void wh_xbox::btn_scan() {
    handleButton(btnBConfig);
    handleButton(btnXConfig);
    handleButton(btnYConfig);
    //handleButton(btnYConfig);
    handleButton(btnDirLeftConfig);
    handleButton(btnDirRightConfig);
}

void wh_xbox::process_data() {
    btn_scan();
    joymap_compute();

    switch (Speed_level) {
        case 0:Speed_map=0.4;break;
        case 1:Speed_map=0.8;break;
        case 2:Speed_map=1.2;break;
        default:Speed_map=0;break;
    }
    
    if(Stop_flag==1){
        lifter_motor->set_rpm(0);
        turn_motor->set_rpm(0);
        shooter_motor->set_rpm(0);
        pithcer_motor->set_rpm(0);
    }
    else if(Stop_flag==0){
        lifter_motor->set_rpm(Speed_map*xbox_msgs.joyRVert_map * max_lifter_speed);
        turn_motor->set_rpm(-Speed_map*xbox_msgs.joyRHori_map * max_turn_speed);

        shooter_motor->set_rpm(Speed_map*xbox_msgs.joyLVert_map * max_shooter_speed);
        pithcer_motor->set_rpm(-(Speed_map*xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * max_pithcer_speed);
    }

    if(Catcher_flag==1){
        HAL_GPIO_WritePin(Catcher_port,Catcher_pin,GPIO_PIN_SET);
    }
    else if(Catcher_flag==0){
        HAL_GPIO_WritePin(Catcher_port,Catcher_pin,GPIO_PIN_RESET);
    }

    if(Shoot_flag==1){
        HAL_GPIO_WritePin(Shooter_port,Shooter_pin,GPIO_PIN_SET);
    }
    else if(Shoot_flag==0){
        HAL_GPIO_WritePin(Shooter_port,Shooter_pin,GPIO_PIN_RESET);
    }



}