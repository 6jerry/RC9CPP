#ifndef FREERTOS_TASK_H
#define FREERTOS_TASK_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
// 任务函数声明
void create_freertos_tasks(); // 创建所有任务
#define FRAME_HEAD_0_MAT 0xFC
#define FRAME_HEAD_1_MAT 0xFB
#define FRAME_ID_MAT 0x01 // 示例数据帧ID
#define FRAME_END_0_MAT 0xFD
#define FRAME_END_1_MAT 0xFE
#define MAX_DATA_LENGTH_MAT 32

typedef struct serial_frame_mat
{
    uint8_t data_length;
    uint8_t frame_head[2];
    uint8_t frame_id;
    uint16_t crc_calculated;
    union data
    {
        float msg_get[MAX_DATA_LENGTH_MAT];
        uint8_t buff_msg[MAX_DATA_LENGTH_MAT * 4];
    } data;
    union check_code
    {
        uint16_t crc_code;
        uint8_t crc_buff[2];
    } check_code;
    uint8_t frame_end[2];
} serial_frame_mat_t;

uint8_t handle_serial_data_mat(uint8_t byte);
void send_serial_frame_mat(uint8_t frame_id, uint8_t data_length, uint8_t *data);

extern serial_frame_mat_t rx_frame_mat;

#endif // FREERTOS_TASK_H
