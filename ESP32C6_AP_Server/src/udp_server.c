#include "udp_server.h"

#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#define PORT 3333
#define MAX_CLIENTS 5

static const char *TAG = "UDP_SERVER";

static void udp_server_task(void *pvParameters)
{
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket listening on port %d", PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        char buffer[128];
        int len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&client_addr, &client_addr_len);
        if (len > 0) {
            buffer[len] = '\0';
            ESP_LOGI(TAG, "Received: %s", buffer);
            sendto(sock, buffer, len, 0, (struct sockaddr *)&client_addr, client_addr_len);
        }
    }

    close(sock);
    vTaskDelete(NULL);
}

void udp_server_start(void)
{
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
}