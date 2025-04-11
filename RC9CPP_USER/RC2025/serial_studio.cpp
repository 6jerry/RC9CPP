#include "serial_studio.h"


void serial_studio::send_float_debuginfo(uint8_t id, float *data, uint8_t length)
{

    sendFloatData(id, data, length);
}

void serial_studio::add_IO(RC9Protocol *port_)
{
    addport(port_);
}
void serial_studio::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    for (int i = 0; i < byteCount / 4; i++)
    {
        temp_param[i] = floatData[i];
    }
}