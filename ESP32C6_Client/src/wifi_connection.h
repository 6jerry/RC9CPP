#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

void wifi_init_sta(TaskHandle_t task_handle);

#endif