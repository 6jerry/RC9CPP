// udp_client.h
#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include "wifi_connection.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "esp_system.h"  // 必须包含头文件
#include "esp_uart.h"
#include "esp_event.h"
#include "esp_wifi.h"

void udp_client_task(void *pvParameters);
void process_udp_data(int sock);
bool udp_communication_test_loop(int sock, struct sockaddr_in *server_addr);
char* hex_to_string(const uint8_t* data, size_t length, char delimiter);

#endif