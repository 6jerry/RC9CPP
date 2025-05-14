#include "motor.h"

dji_motor *dji_motor::m3508_instances_can1[MAX_INSTANCES] = {nullptr};
dji_motor *dji_motor::m3508_instances_can2[MAX_INSTANCES] = {nullptr};
dji_motor *dji_motor::m3508_instances_can3[MAX_INSTANCES] = {nullptr};

uint8_t dji_motor::instanceCount_m3508_can1 = 0;
uint8_t dji_motor::instanceCount_m3508_can2 = 0;
uint8_t dji_motor::instanceCount_m3508_can3 = 0;

dji_motor *dji_motor::m6020_instances_can1[MAX_INSTANCES] = {nullptr};

dji_motor *dji_motor::m6020_instances_can2[MAX_INSTANCES] = {nullptr};

dji_motor *dji_motor::m6020_instances_can3[MAX_INSTANCES] = {nullptr};

uint8_t dji_motor::instanceCount_m6020_can1 = 0;
uint8_t dji_motor::instanceCount_m6020_can2 = 0;
uint8_t dji_motor::instanceCount_m6020_can3 = 0;

float dji_motor::vcurrent_to_rcurrent(int16_t vc)
{

    return ((float)vc / (float)max_vcurrent) * (float)max_rcurrent; // mA
}

int16_t dji_motor::rcurrent_to_vcurrent(float rc)
{
    return (rc / (float)max_rcurrent) * max_vcurrent;
}

float dji_motor::vangle_to_rangle(uint32_t va)
{
    return ((float)va / (float)max_vangle) * 360.0f;
}

dji_motor::dji_motor(float max_rcurrent_, int16_t max_vcurrent_, uint16_t max_vangle_, DjiMotorType type_, uint32_t can_id_, FDCAN_HandleTypeDef *hcan_) : max_rcurrent(max_rcurrent_), max_vcurrent(max_vcurrent_), max_vangle(max_vangle_), type(type_)
{
    if (type_ == M3508_M2006)
    {
        if (hcan_ == &hfdcan1)
        {
            if (can_id_ >= dji_id_1 && can_id_ <= dji_id_8)
            {
                m3508_instances_can1[can_id_ - dji_id_1] = this;
                instanceCount_m3508_can1++;
            }
        }
        else if (hcan_ == &hfdcan2)
        {
            if (can_id_ >= dji_id_1 && can_id_ <= dji_id_8)
            {
                m3508_instances_can2[can_id_ - dji_id_1] = this;
                instanceCount_m3508_can2++;
            }
        }
        else if (hcan_ == &hfdcan3)
        {
            if (can_id_ >= dji_id_1 && can_id_ <= dji_id_8)
            {
                m3508_instances_can3[can_id_ - dji_id_1] = this;
                instanceCount_m3508_can3++;
            }
        }
    }
    else if (type_ == M6020)
    {
        if (hcan_ == &hfdcan1)
        {
            if (can_id_ >= dji_id_5 && can_id_ <= dji_id_11)
            {
                m6020_instances_can1[can_id_ - dji_id_5] = this;
                instanceCount_m6020_can1++;
            }
        }
        else if (hcan_ == &hfdcan2)
        {
            if (can_id_ >= dji_id_5 && can_id_ <= dji_id_11)
            {
                m6020_instances_can2[can_id_ - dji_id_5] = this;
                instanceCount_m6020_can2++;
            }
        }
        else if (hcan_ == &hfdcan3)
        {
            if (can_id_ >= dji_id_5 && can_id_ <= dji_id_11)
            {
                m6020_instances_can3[can_id_ - dji_id_5] = this;
                instanceCount_m6020_can3++;
            }
        }
    }
}

void power_motor::switch_mode(motor_mode target_mode)
{
    mode = target_mode;
}

void dji_motor_handle::process_data()
{
    if (dji_motor::instanceCount_m3508_can1 > 0)
    {
        uint8_t send_buf[8] = {0};
        uint32_t send_id = 0x200;
        uint8_t if_have = 0;
        for (int i = 0; i < 4; i++)
        {
            if (dji_motor::m3508_instances_can1[i] != nullptr)
            {
                int16_t temp_vcurrent = dji_motor::m3508_instances_can1[i]->motor_process();
                send_buf[2 * i] = (uint8_t)(temp_vcurrent >> 8);
                send_buf[2 * i + 1] = (uint8_t)temp_vcurrent;
                if_have = 1;
            }
        }
        if (if_have == 1)
        {
            CanDevice::CAN_Send(send_id, 0, send_buf, &hfdcan1);
        }

        uint8_t send_buf2[8] = {0};
        uint32_t send_id2 = 0x1FF;
        uint8_t if_have2 = 0;
        for (int i = 4; i < 8; i++)
        {
            if (dji_motor::m3508_instances_can1[i] != nullptr)
            {
                int16_t temp_vcurrent = dji_motor::m3508_instances_can1[i]->motor_process();
                send_buf2[2 * (i - 4)] = (uint8_t)(temp_vcurrent >> 8);
                send_buf2[2 * (i - 4) + 1] = (uint8_t)temp_vcurrent;
                if_have2 = 1;
            }
        }
        if (if_have2 == 1)
        {
            CanDevice::CAN_Send(send_id2, 0, send_buf2, &hfdcan1);
        }
    }
    if (dji_motor::instanceCount_m3508_can2 > 0)
    {
        uint8_t send_buf[8] = {0};
        uint32_t send_id = 0x200;
        uint8_t if_have = 0;
        for (int i = 0; i < 4; i++)
        {
            if (dji_motor::m3508_instances_can2[i] != nullptr)
            {
                int16_t temp_vcurrent = dji_motor::m3508_instances_can2[i]->motor_process();
                send_buf[2 * i] = (uint8_t)(temp_vcurrent >> 8);
                send_buf[2 * i + 1] = (uint8_t)temp_vcurrent;
                if_have = 1;
            }
        }
        if (if_have == 1)
        {
            CanDevice::CAN_Send(send_id, 0, send_buf, &hfdcan2);
        }

        uint8_t send_buf2[8] = {0};
        uint32_t send_id2 = 0x1FF;
        uint8_t if_have2 = 0;
        for (int i = 4; i < 8; i++)
        {
            if (dji_motor::m3508_instances_can2[i] != nullptr)
            {
                int16_t temp_vcurrent = dji_motor::m3508_instances_can2[i]->motor_process();
                send_buf2[2 * (i - 4)] = (uint8_t)(temp_vcurrent >> 8);
                send_buf2[2 * (i - 4) + 1] = (uint8_t)temp_vcurrent;
                if_have2 = 1;
            }
        }
        if (if_have2 == 1)
        {
            CanDevice::CAN_Send(send_id2, 0, send_buf2, &hfdcan2);
        }
    }
    if (dji_motor::instanceCount_m3508_can3 > 0)
    {
        uint8_t send_buf[8] = {0};
        uint32_t send_id = 0x200;
        uint8_t if_have = 0;
        for (int i = 0; i < 4; i++)
        {
            if (dji_motor::m3508_instances_can3[i] != nullptr)
            {
                int16_t temp_vcurrent = dji_motor::m3508_instances_can3[i]->motor_process();
                send_buf[2 * i] = (uint8_t)(temp_vcurrent >> 8);
                send_buf[2 * i + 1] = (uint8_t)temp_vcurrent;
                if_have = 1;
            }
        }
        if (if_have == 1)
        {
            CanDevice::CAN_Send(send_id, 0, send_buf, &hfdcan3);
        }

        uint8_t send_buf2[8] = {0};
        uint32_t send_id2 = 0x1FF;
        uint8_t if_have2 = 0;
        for (int i = 4; i < 8; i++)
        {
            if (dji_motor::m3508_instances_can3[i] != nullptr)
            {
                int16_t temp_vcurrent = dji_motor::m3508_instances_can3[i]->motor_process();
                send_buf2[2 * (i - 4)] = (uint8_t)(temp_vcurrent >> 8);
                send_buf2[2 * (i - 4) + 1] = (uint8_t)temp_vcurrent;
                if_have2 = 1;
            }
        }
        if (if_have2 == 1)
        {
            CanDevice::CAN_Send(send_id2, 0, send_buf2, &hfdcan3);
        }
    }
}