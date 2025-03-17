#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void tcp_server_task(void *pvParameters);

#endif