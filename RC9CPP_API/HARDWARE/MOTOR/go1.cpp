
#include "go1.h"

int go1::modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;

    //		SATURATE(motor_s->id,   0,    15);
    //		SATURATE(motor_s->mode, 0,    7);
    SATURATE(motor_s->K_P, 0.0f, 25.599f);
    SATURATE(motor_s->K_W, 0.0f, 25.599f);
    SATURATE(motor_s->T, -127.99f, 127.99f);
    SATURATE(motor_s->W, -804.00f, 804.00f);
    SATURATE(motor_s->Pos, -411774.0f, 411774.0f);

    motor_s->motor_send_data.mode.id = motor_s->id;
    motor_s->motor_send_data.mode.status = motor_s->mode;
    motor_s->motor_send_data.comd.k_pos = motor_s->K_P / 25.6f * 32768;
    motor_s->motor_send_data.comd.k_spd = motor_s->K_W / 25.6f * 32768;
    motor_s->motor_send_data.comd.pos_des = motor_s->Pos / 6.2832f * 32768;
    motor_s->motor_send_data.comd.spd_des = motor_s->W / 6.2832f * 256;
    motor_s->motor_send_data.comd.tor_des = motor_s->T * 256;
    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}

go1::go1(UART_HandleTypeDef *huart) : SerialDevice(huart)
{
}

void go1::process_data()
{
    cmd.id = 0; // 给电机控制指令结构体赋值
    cmd.mode = 1;
    cmd.T = 0;
    cmd.W = 20;
    cmd.Pos = 0;
    cmd.K_P = 0;
    cmd.K_W = 0.05;
    modify_data(&cmd);
    HAL_UART_Transmit(huart_, (uint8_t *)&cmd, sizeof(cmd.motor_send_data), 10);
}
void go1::handleReceiveData(uint8_t byte)
{
}
