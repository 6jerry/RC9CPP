#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "device_info.h"

#define PORT 3333

void udp_server_start(void);

#endif