#ifndef POSITION_SETUP_H
#define POSITION_SETUP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "TaskManager.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "RC9Protocol.h"
#include "Oencoder.h"
#include "HWT101CT.h"

    void position_setup();
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class demo : public ITaskProcessor, public RC9subscriber
{
private:
    float elapsedTime = 0.0f, test_data = 0.0f, send_data = 6.0f;
    uint32_t currentTick = 0;
    uint32_t previousTick = 0;

public:
    void process_data();
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
};

#endif
#endif