#ifndef FDI_H
#define FDI_H

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
#define CMD_IMU_FDI 0x40
#define CMD_AHRS_FDI 0x41
#define FRAME_HEAD_FDI 0xFC
#define FRAME_END_FDI 0xFD
#define PI 3.14159
#define MAX_DATA_LENGTH_FDI 16
typedef struct
{
    uint8_t frame_head_fdi;
    uint8_t command_fdi;
    uint8_t data_length_fdi;
    uint8_t flow_id_fdi;
    uint8_t frame_crc8_fdi;
    uint16_t crc_calculated_fdi;
    union
    {
        float msg_get_fdi[MAX_DATA_LENGTH_FDI];
        uint8_t buff_msg_fdi[MAX_DATA_LENGTH_FDI * 4];
    } data_fdi;
    union
    {
        uint16_t crc_code_fdi;
        uint8_t crc_buff_fdi[2];
    } check_code_fdi;
    uint8_t frame_end_fdi;
} serial_frame_t_fdi_;

typedef struct
{
    float Gyroscope_X_fdi;
    float Gyroscope_Y_fdi;
    float Gyroscope_Z_fdi;
    float Accelerometer_X_fdi;
    float Accelerometer_Y_fdi;
    float Accelerometer_Z_fdi;
    float Magnetometer_X_fdi;
    float Magnetometer_Y_fdi;
    float Magnetometer_Z_fdi;
    float IMU_Temperature_fdi;
    float Pressure_fdi;
    float Pressure_Temperature_fdi;

} IMU_Data_t_fdi_;

typedef struct
{
    float RollSpeed_fdi;
    float PitchSpeed_fdi;
    float HeadingSpeed_fdi;
    float Roll_fdi;
    float Pitch_fdi;
    float Heading_fdi;
    float Q1_fdi;
    float Q2_fdi;
    float Q3_fdi;
    float Q4_fdi;

} AHRS_Data_t_fdi_;

class fdi : public SerialDevice, public imu

{
private:
    enum RxState
    {
        WAITING_FOR_HEADER_FDI,
        WAITING_FOR_COMMAND_FDI,
        WAITING_FOR_LENGTH_FDI,
        WAITING_FOR_FLOW_ID_FDI,
        WAITING_FOR_CRC8_FDI,
        WAITING_FOR_DATA_FDI,
        WAITING_FOR_CRC_0_FDI,
        WAITING_FOR_CRC_1_FDI,
        WAITING_FOR_END_FDI
    } rx_state_fdi;
    serial_frame_t_fdi_ fdi_rx_frame_fdi;
    uint8_t rx_index_fdi = 0;
    float IMU_YawTransform(float yaw);
    float now_heading = 0.0f;
    float angle_read = 0.0f;
    float orin_heading = 0.0f;
    float head_div = 0.0f;
    float relative_heading = 0.0f;

public:
    IMU_Data_t_fdi_ imu_data_fdi;
    AHRS_Data_t_fdi_ ahrs_data_fdi;
    void handleReceiveData(uint8_t byte);
    fdi(UART_HandleTypeDef *huart_);

    float real_heading = 0.0f;
    float get_heading() override;
};

#endif

#endif