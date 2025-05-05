#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H
#include "wifi_connection.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"

void tcp_client_task(void *pvParameters);

#endif