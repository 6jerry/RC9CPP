#ifndef AIR_GAGUE_H
#define AIR_GAGUE_H


#include <cstdint>
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include "Serial_device.h"
#include "TaskManager.h"
#include "imu.h"
#include "math.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**************must*************/
// BAUD_RATE: 19200
/******************************/

class Air_gague : public SerialDevice, public imu, public ITaskProcessor
{
public:
    union {
        uint8_t tmp[2];
        int16_t data;
    }air_data; // 是Mpa
    float air_pressure = 0.0f;
    Air_gague(UART_HandleTypeDef *huart, uart_type type);
    float get_air_presure(void) override;
    void handleReceiveData(uint8_t byte) override;
    void process_data() override;
    const uint8_t cmd[8] = {0x12, 0x04, 0x00, 0x01, 0x00, 0x01, 0x62, 0xA9};
	
};

#endif
#endif