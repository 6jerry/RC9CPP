#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "device_info.h"

#define PORT 3333
#define MAX_CLIENTS 5

void tcp_server_start(void);

#endif