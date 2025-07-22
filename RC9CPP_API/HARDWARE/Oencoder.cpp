#include "Oencoder.h"

void Oencoder::process_data()
{
    uint32_t current_time = HAL_GetTick();
    if (previous_time != 0)
    {
        delta_time = (float)(current_time - previous_time) / 1000.0f;
    }

    left_wheel.current_count = __HAL_TIM_GET_COUNTER(left_wheel.ecoder_tim);
    right_wheel.current_count = __HAL_TIM_GET_COUNTER(right_wheel.ecoder_tim);

    test_left = (float)left_wheel.current_count;
    test_right = (float)right_wheel.current_count;

    calc_delta_count(&left_wheel);
    calc_delta_count(&right_wheel);

    left_wheel.delta_count = -left_wheel.delta_count;
    right_wheel.delta_count = -right_wheel.delta_count;

    robot_delta_x = (double)left_wheel.delta_count * pos_2_rpm * wheel_p * (-COS45) - (double)right_wheel.delta_count * pos_2_rpm * wheel_p * COS45;
    robot_delta_y = (double)left_wheel.delta_count * pos_2_rpm * wheel_p * COS45 - (double)right_wheel.delta_count * pos_2_rpm * wheel_p * (COS45);

    world_delta_x = robot_delta_x * (double)(arm_cos_f32(IMU->get_yaw_rad())) - robot_delta_y * (double)(arm_sin_f32(IMU->get_yaw_rad()));

    world_delta_y = robot_delta_x * (double)(arm_sin_f32(IMU->get_yaw_rad())) + robot_delta_y * (double)(arm_cos_f32(IMU->get_yaw_rad()));

    world_pos_x += world_delta_x;
    world_pos_y += world_delta_y;

    data_send[0] = (float)world_pos_x;
    data_send[1] = (float)world_pos_y;
    data_send[2] = (float)IMU->get_heading();
    data_send[3] = (float)IMU->get_yaw_speed_rad();
    data_send[4] = robot_delta_x / delta_time;
    data_send[5] = robot_delta_y / delta_time;
    check_if_error();
		sendFloatData(1, data_send, 6);
//    if (left_wheel_ec == 0 && right_wheel_ec == 0 && imu_ec == 0) // 只有position全部模块都正常才会发送定位数据
//    {
//        sendFloatData(1, data_send, 6);
//    }
//    else
//    {
//        uint8_t send_errors[3] = {left_wheel_ec, right_wheel_ec, imu_ec};
//        sendByteData(2, send_errors, 3); // 错误帧的id为2
//    }

    previous_time = current_time;
}

void Oencoder::check_if_error()
{
    if (abs(IMU->get_yaw_speed_rad()) > 0.05f) // position正在转动
    {
        if (left_wheel.delta_count == 0)
        {
            left_wheel_ec = 1;
        }
        else
        {
            left_wheel_ec = 0;
        }

        if (right_wheel.delta_count == 0)
        {
            right_wheel_ec = 1;
        }
        else
        {
            right_wheel_ec = 0;
        }
    }
    if (IMU->get_update_time() == 0)
    {
        if (!detected_imu_error)
        {
            find_imu_error = HAL_GetTick();
            detected_imu_error = true;
        }

        if (detected_imu_error)
        {
            if (HAL_GetTick() - find_imu_error > 500) // 超时时间为500ms
            {
                imu_ec = 1;
            }
        }
    }
    if (IMU->get_update_time() != 0)
    {
        uint32_t now_time = HAL_GetTick();
        if (now_time - IMU->get_update_time() > 500) // 超时时间为500ms
        {
            imu_ec = 1;
        }
        else
        {
            imu_ec = 0;
            detected_imu_error = false;
            find_imu_error = 0;
        }
    }
}

void Oencoder::calc_delta_count(encoderwheel_t *wheel)
{
    if (wheel->last_count < overflow_min && wheel->current_count > overflow_max)
    {
        wheel->delta_count = -(wheel->last_count + max_count - wheel->current_count);
    }
    else if (wheel->last_count > overflow_max && wheel->current_count < overflow_min)
    {
        wheel->delta_count = wheel->current_count + max_count - wheel->last_count;
    }
    else
    {
        wheel->delta_count = wheel->current_count - wheel->last_count;

        wheel->delta_count = -wheel->delta_count;
    }
    wheel->last_count = wheel->current_count;
}
void Oencoder::add_imu(imu *IMU_)
{
    IMU = IMU_;
}

Oencoder::Oencoder(TIM_HandleTypeDef *left_encoder_tim, TIM_HandleTypeDef *right_encoder_tim, float wheel_perimeter_)
{
    left_wheel.ecoder_tim = left_encoder_tim;
    right_wheel.ecoder_tim = right_encoder_tim;
}

void Oencoder::pos_tf()
{
}

void Oencoder::init()
{
    HAL_TIM_Encoder_Start(left_wheel.ecoder_tim, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(right_wheel.ecoder_tim, TIM_CHANNEL_ALL);
}

void Oencoder::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    if (id == 1)
    {
        world_pos_x = (double)(floatData[0] * 1000.0f);

        world_pos_y = (double)(floatData[1] * 1000.0f);

        // IMU->imu_reset_heading((float)floatData[2]);
    }
    if (id == 2)
    {

        world_pos_x = (double)(floatData[0] * 1000.0f);

        world_pos_y = (double)(floatData[1] * 1000.0f);
        IMU->imu_reset_heading((float)floatData[0]);
    }

    if (id == 3)
    {
        IMU->imu_rst();
        world_pos_x = (double)(floatData[0] * 1000.0f);

        world_pos_y = (double)(floatData[1] * 1000.0f);
    }
}