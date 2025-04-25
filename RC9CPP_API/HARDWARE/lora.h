#ifndef LORA_H
#define LORA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "RC9Protocol.h"
#include "TaskManager.h"
#include "imu.h"
#include "Vector2D.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Lora : public RC9subscriber, public ITaskProcessor, public imu
{
public:
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
    void process_data();
    void add_imu(imu *imu_);

    Vector2D lora_msgs;//接收坐标
    Vector2D own_msgs;//当前己方机器人坐标

private:
    imu *IMU = nullptr;

    float tx_frame_mat[2];
};

#endif
#endif
