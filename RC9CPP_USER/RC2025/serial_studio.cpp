#include "serial_studio.h"

serialStudio::serialStudio()
{
}


void serialStudio::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{

    if (id == 1)
    {
        command = byteData[0];
    }
    else if (id == 2)
    {

        cricle_R = floatData[0];
    }
}
