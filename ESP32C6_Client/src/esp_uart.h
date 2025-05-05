#ifndef _ESP_UART_H
#define _ESP_UART_H
#pragma once
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "udp_client.h"
#include "esp_timer.h"
#include "esp_pm.h"

#ifdef __cplusplus
extern "C" {
#endif
// 协议相关定义
#define FRAME_HEAD_0_RC9  0xFC
#define FRAME_HEAD_1_RC9  0xFB
#define FRAME_END_0_RC9   0xFD
#define FRAME_END_1_RC9   0xFE
#define MAX_RC9_DATA_LEN  64

// 解析状态机状态定义
typedef enum {
    RC9_STATE_WAIT_HEAD1,
    RC9_STATE_WAIT_HEAD2,
    RC9_STATE_READ_ID,
    RC9_STATE_READ_LEN,
    RC9_STATE_READ_DATA,
    RC9_STATE_READ_CRC1,
    RC9_STATE_READ_CRC2,
    RC9_STATE_READ_END1,
    RC9_STATE_READ_END2
} rc9_parser_state_t;


// 数据包结构体（对外暴露）
typedef struct {
    uint8_t frame_id;
    uint8_t data_length;
    uint8_t data[MAX_RC9_DATA_LEN];
    uint16_t crc;
} rc9_frame_t;

// 回调函数类型定义
typedef void (*rc9_callback_t)(const rc9_frame_t* frame);

void init_uart(uart_port_t uart_num,int baud_rate, int tx_pin, int rx_pin);
void process_rc9_frame(const rc9_frame_t *frame);
void rc9_parser_init(rc9_callback_t callback,uart_port_t uart_num);
void rc9_reset_parser(void);
void parse_rc9_byte(uint8_t byte);
void init_udp_client(int sock_, struct sockaddr_in *server_addr__);

// 解析器私有结构体
typedef struct {
    rc9_parser_state_t state;
    rc9_frame_t current_frame;
    uint8_t data_index;
    rc9_callback_t user_callback;
    uart_port_t uart_num;      // UART端口号
} rc9_parser_t;

extern rc9_parser_t parser;
extern bool update_xbox;
extern int sent_fail_count;
extern bool fail_flag;
extern int64_t start;

#ifdef __cplusplus
}
#endif
#endif