#include "HWT101CT.h"

HWT101CT::HWT101CT(UART_HandleTypeDef *huart_) : rx_state(WAITING_FOR_HEADER_1), reserved_index(0), calculated_checksum(0), SerialDevice(huart_) {}

float HWT101CT::calculateYaw(uint8_t YawH, uint8_t YawL)
{
    int16_t raw_yaw = (YawH << 8) | YawL;
    return ((float)raw_yaw / 32768.0f) * 180.0f;
}

uint8_t HWT101CT::calculateChecksum()
{
    return FRAME_HEADER_1 + FRAME_HEADER_2 + frame.reserved[0] + frame.reserved[1] +
           frame.reserved[2] + frame.reserved[3] + frame.YawL + frame.YawH + frame.VL + frame.VH;
}

void HWT101CT::handleReceiveData(uint8_t byte)
{
    switch (rx_state)
    {
    case WAITING_FOR_HEADER_1:
        if (byte == FRAME_HEADER_1)
        {
            rx_state = WAITING_FOR_HEADER_2;
        }
        break;

    case WAITING_FOR_HEADER_2:
        if (byte == FRAME_HEADER_2)
        {
            rx_state = WAITING_FOR_RESERVED_1;
            reserved_index = 0;
        }
        else
        {
            rx_state = WAITING_FOR_HEADER_1;
        }
        break;

    case WAITING_FOR_RESERVED_1:
    case WAITING_FOR_RESERVED_2:
    case WAITING_FOR_RESERVED_3:
    case WAITING_FOR_RESERVED_4:
        frame.reserved[reserved_index++] = byte;
        if (reserved_index >= 4)
        {
            rx_state = WAITING_FOR_YAWL;
        }
        else
        {
            rx_state = static_cast<RxState>(rx_state + 1);
        }
        break;

    case WAITING_FOR_YAWL:
        frame.YawL = byte;
        rx_state = WAITING_FOR_YAWH;
        break;

    case WAITING_FOR_YAWH:
        frame.YawH = byte;
        rx_state = WAITING_FOR_VL;
        break;

    case WAITING_FOR_VL:
        frame.VL = byte;
        rx_state = WAITING_FOR_VH;
        break;

    case WAITING_FOR_VH:
        frame.VH = byte;
        rx_state = WAITING_FOR_CHECKSUM;
        break;

    case WAITING_FOR_CHECKSUM:
        frame.checksum = byte;
        calculated_checksum = calculateChecksum();

        if (calculated_checksum == frame.checksum)
        {
            orin_yaw = -calculateYaw(frame.YawH, frame.YawL);

            processDecodedData(orin_yaw);
        }

        rx_state = WAITING_FOR_HEADER_1;
        break;

    default:
        rx_state = WAITING_FOR_HEADER_1;
        break;
    }
}

void HWT101CT::processDecodedData(float yaw)
{
    if (if_init)
    {
        init_count++;
        if (init_count > 6)
        {
            if_init = false;
            init_yaw = yaw;
        }
    }
    else
    {
        yaw_tf(yaw);
    }
}
void HWT101CT::yaw_tf(float nowyaw)
{
    delta_angle = nowyaw - init_yaw;
    if (delta_angle >= 180.0f)
    {
        delta_angle -= 360.0f;
    }
    else if (delta_angle < -180.0f)
    {
        delta_angle += 360.0f;
    }

    real_yaw = delta_angle;
}
float HWT101CT::get_heading()
{
    return real_yaw;
}