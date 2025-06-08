#ifndef HWT101CT_H
#define HWT101CT_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "Serial_device.h"
#include "imu.h"
#ifdef __cplusplus
}

#endif
#ifdef __cplusplus
#define FRAME_HEADER_1 0x55
#define FRAME_HEADER_2 0x53

typedef struct
{
    uint8_t reserved[4]; // 前4个保留字节
    uint8_t YawL;
    uint8_t YawH;
    uint8_t VL;
    uint8_t VH;
    uint8_t checksum;
} HWT101CT_Frame_t;

class HWT101CT : public SerialDevice, public imu
{
private:
    enum RxState
    {
        WAITING_FOR_HEADER_1,
        WAITING_FOR_HEADER_2,
        WAITING_FOR_RESERVED_1,
        WAITING_FOR_RESERVED_2,
        WAITING_FOR_RESERVED_3,
        WAITING_FOR_RESERVED_4,
        WAITING_FOR_YAWL,
        WAITING_FOR_YAWH,
        WAITING_FOR_VL,
        WAITING_FOR_VH,
        WAITING_FOR_CHECKSUM
    } rx_state;

    HWT101CT_Frame_t frame;
    uint8_t reserved_index;
    uint8_t calculated_checksum, init_count = 0;
    float orin_yaw = 0.0f, init_yaw = 0.0f,delta_angle = 0.0f,real_yaw=0.0f;

    float calculateYaw(uint8_t YawH, uint8_t YawL);

    uint8_t calculateChecksum();
    void yaw_tf(float nowyaw);

    bool if_init = true;

public:
    float get_heading() override;
    void handleReceiveData(uint8_t byte);
    void processDecodedData(float yaw);
    HWT101CT(UART_HandleTypeDef *huart_);
};

#endif
#endif