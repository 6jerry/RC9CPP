#ifndef HWT101CT_H
#define HWT101CT_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "Serial_device.h"
#include "imu.h"
#include "gpio.h"
#include "TaskManager.h"
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

class HWT101CT : public SerialDevice, public imu, public ITaskProcessor
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
    float orin_yaw = 0.0f, init_yaw = 0.0f, delta_angle = 0.0f, real_yaw = 0.0f, yaw_rad = 0.0f;

    float calculateYaw(uint8_t YawH, uint8_t YawL);

    uint8_t calculateChecksum();
    void yaw_tf(float nowyaw);

    bool if_init = true;

 

    GPIO_TypeDef *port_;
    uint16_t pin_;

    bool if_reset_io = false;
    uint8_t reset_cnt = 0;
    uint32_t last_update_time = 0, now_time = 0;
    float delta_time = 0.0f, yaw_speed_rad = 0.0f, last_yaw = 0.0f;

public:
    float
    get_heading() override;
    float get_yaw_rad() override;
    void handleReceiveData(uint8_t byte);
    void processDecodedData(float yaw);
    HWT101CT(UART_HandleTypeDef *huart_);

    void add_io(GPIO_TypeDef *port, uint16_t pin);
    void process_data();
    void set_io();
    void reset_io();

    void imu_rst() override;

    void imu_reset_heading(float reheading) override;

    uint32_t get_update_time() override;
    float get_yaw_speed_rad() override;
};

#endif
#endif