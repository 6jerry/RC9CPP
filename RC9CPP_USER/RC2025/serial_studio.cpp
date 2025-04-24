#include "serial_studio.h"

serialStudio::serialStudio()
{
}

void serialStudio::process_data()
{

    if (upper_info.enable_flag == true)
    {
        float send_data[5] = {upper_info.shooter_motor->get_rpm(),
                              upper_info.lifter_motor->get_rpm(),
                              upper_info.turner_motor->get_rpm(),
                              upper_info.encoder->get_absolute_distance(),
                              upper_info.wit_imu->Pitch_angle};
        sendFloatData(upper_info.id, send_data, upper_info.length);
    }
}

void serialStudio::add_upper_info(power_motor *shooter_motor_, power_motor *lifter_motor_, power_motor *turner_motor_,
                                   Encoder *encoder_, wit_gyro *wit_imu_)
{
    upper_info.enable_flag = true;
    upper_info.shooter_motor = shooter_motor_;
    upper_info.lifter_motor = lifter_motor_;
    upper_info.turner_motor = turner_motor_;
    upper_info.encoder = encoder_;
    upper_info.wit_imu = wit_imu_;
}

void serialStudio::send_float_debuginfo(uint8_t id, float *data, uint8_t length)
{

    sendFloatData(id, data, length);
}

void serialStudio::add_IO(RC9Protocol *port_)
{
    addport(port_);
}
void serialStudio::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    for (int i = 0; i < byteCount / 4; i++)
    {
        temp_param[i] = floatData[i];
    }
}