#ifndef __R3_TEST_XBOX_H__
#define __R3_TEST_XBOX_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include "debug_xbox.h"

#include "RC9Protocol.h"
#include "motor.h"
#include "IO_Interrupt.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class photogate_shoot : public GPIODevice
{
	
public:
	
    int flag =0;
    photogate_shoot();
    
    void handleInterrupt() override;
    void add_io_interrupt(GPIO_TypeDef *port, uint16_t pin) override;
    bool is_finish();
};


class R3_xbox : public xbox_debug_base
{
private:
public:
    power_motor *shoot_motor_1 = nullptr;
    power_motor *shoot_motor_2 = nullptr;
    float c = 0.0f;
    float rpm = 1000.0f;
    float move_rpm = 600.0f;
    void mode_1() override;
    void mode_2() override;
    void not_start() override;
    //void add_io(GPIO_TypeDef  *stop_port_, uint16_t  stop_pin_);
    bool read_io();
		photogate_shoot gate;
};



#endif
#endif
