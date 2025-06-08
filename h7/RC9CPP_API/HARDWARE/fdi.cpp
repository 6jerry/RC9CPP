#include "fdi.h"

void fdi::handleReceiveData(uint8_t byte)
{
    switch (rx_state_fdi)
    {
    case WAITING_FOR_HEADER_FDI:
        if (byte == FRAME_HEAD_FDI)
        {
            fdi_rx_frame_fdi.frame_head_fdi = byte;
            rx_state_fdi = WAITING_FOR_COMMAND_FDI;
        }
        break;

    case WAITING_FOR_COMMAND_FDI:
        if (byte != CMD_IMU_FDI && byte != CMD_AHRS_FDI)
        {
            rx_state_fdi = WAITING_FOR_HEADER_FDI;
        }
        else
        {
            fdi_rx_frame_fdi.command_fdi = byte;
            rx_state_fdi = WAITING_FOR_LENGTH_FDI;
        }
        break;

    case WAITING_FOR_LENGTH_FDI:
        fdi_rx_frame_fdi.data_length_fdi = byte;
        rx_index_fdi = 0;
        rx_state_fdi = WAITING_FOR_FLOW_ID_FDI;
        break;

    case WAITING_FOR_FLOW_ID_FDI:
        fdi_rx_frame_fdi.flow_id_fdi = byte; // 接收流水序号
        rx_state_fdi = WAITING_FOR_CRC8_FDI;
        break;

    case WAITING_FOR_CRC8_FDI:
        fdi_rx_frame_fdi.frame_crc8_fdi = byte; // 接收CRC8校验
        // 计算并验证帧头CRC8
        // uint8_t crc8_data_fdi[4] = {fdi_rx_frame_fdi.frame_head_fdi, fdi_rx_frame_fdi.command_fdi, fdi_rx_frame_fdi.data_length_fdi, fdi_rx_frame_fdi.flow_id_fdi};
        // fdi_crc8_calculated_fdi = CRC8_Table(crc8_data_fdi, 4);

        rx_state_fdi = WAITING_FOR_CRC_0_FDI;

        break;

    case WAITING_FOR_CRC_0_FDI:
        fdi_rx_frame_fdi.check_code_fdi.crc_buff_fdi[0] = byte;
        rx_state_fdi = WAITING_FOR_CRC_1_FDI;
        break;

    case WAITING_FOR_CRC_1_FDI:
        fdi_rx_frame_fdi.check_code_fdi.crc_buff_fdi[1] = byte;
        rx_state_fdi = WAITING_FOR_DATA_FDI;
        break;

    case WAITING_FOR_DATA_FDI:
        fdi_rx_frame_fdi.data_fdi.buff_msg_fdi[rx_index_fdi++] = byte;
        if (rx_index_fdi >= fdi_rx_frame_fdi.data_length_fdi)
        {
            fdi_rx_frame_fdi.crc_calculated_fdi = 0;

            rx_state_fdi = WAITING_FOR_END_FDI;
        }
        break;

    case WAITING_FOR_END_FDI:
        if (byte == FRAME_END_FDI)
        {
            fdi_rx_frame_fdi.frame_end_fdi = byte;

            if (fdi_rx_frame_fdi.command_fdi == CMD_IMU_FDI)
            {
                imu_data_fdi.Gyroscope_X_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[0];
                imu_data_fdi.Gyroscope_Y_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[1];
                imu_data_fdi.Gyroscope_Z_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[2];
                imu_data_fdi.Accelerometer_X_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[3];
                imu_data_fdi.Accelerometer_Y_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[4];
                imu_data_fdi.Accelerometer_Z_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[5];
            }
            else if (fdi_rx_frame_fdi.command_fdi == CMD_AHRS_FDI)
            {
                ahrs_data_fdi.RollSpeed_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[0];
                ahrs_data_fdi.PitchSpeed_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[1];
                ahrs_data_fdi.HeadingSpeed_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[2];
                ahrs_data_fdi.Roll_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[3];
                ahrs_data_fdi.Pitch_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[4];
                ahrs_data_fdi.Heading_fdi = fdi_rx_frame_fdi.data_fdi.msg_get_fdi[5];

                real_heading = IMU_YawTransform(ahrs_data_fdi.Heading_fdi) ;
            }
        }
        rx_state_fdi = WAITING_FOR_HEADER_FDI;
        break;

    default:
        rx_state_fdi = WAITING_FOR_HEADER_FDI;
        break;
    }
}

float fdi::IMU_YawTransform(float yaw)
{
    now_heading = yaw;
    if (angle_read == 0)
    {
        orin_heading = now_heading;
        angle_read = 1;
    }
    head_div = now_heading - orin_heading;
    if (head_div >= PI)
    {
        relative_heading = 2 * PI - now_heading + orin_heading;
        return relative_heading;
    }
    if (head_div < 0 && head_div > -PI)
    {
        relative_heading = orin_heading - now_heading;
        return relative_heading;
    }
    if (head_div <= -PI)
    {
        relative_heading = -(2 * PI - orin_heading + now_heading);
        return relative_heading;
    }
    if (head_div >= 0 && head_div < PI)
    {
        relative_heading = orin_heading - now_heading;
        return relative_heading;
    }
    return relative_heading;
}

fdi::fdi(UART_HandleTypeDef *huart_) : SerialDevice(huart_)
{
}

float fdi::get_heading()
{
    return -real_heading;
}