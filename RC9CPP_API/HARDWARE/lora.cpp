#include "lora.h"

void Lora::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    if (byteCount == 8)
    {
        lora_msgs.x = byteData[0];
        lora_msgs.y = byteData[1];
    }
}

void Lora::process_data()
{
    own_msgs = IMU->get_world_pos();
    tx_frame_mat[0]=own_msgs.x;
    tx_frame_mat[1]=own_msgs.y;
    sendFloatData(0x01, tx_frame_mat, 2);
}

void Lora::add_imu(imu *imu_)
{
    IMU = imu_;
}