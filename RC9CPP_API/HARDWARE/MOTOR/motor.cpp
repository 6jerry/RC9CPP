#include "motor.h"

float dji_motor::vcurrent_to_rcurrent(int16_t vc)
{

    return ((float)vc / (float)max_vcurrent) * (float)max_rcurrent; // mA
}

int16_t dji_motor::rcurrent_to_vcurrent(float rc)
{
    return (rc / (float)max_rcurrent) * max_vcurrent;
}

float dji_motor::vangle_to_rangle(uint32_t va)
{
    return ((float)va / (float)max_vangle) * 360.0f;
}

dji_motor::dji_motor(float max_rcurrent_, int16_t max_vcurrent_, uint16_t max_vangle_) : max_rcurrent(max_rcurrent_), max_vcurrent(max_vcurrent_), max_vangle(max_vangle_)
{
}

void power_motor::switch_mode(motor_mode target_mode)
{
    mode = target_mode;
}