#include "tcp_server.h"

#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#define PORT 3333
#define MAX_CLIENTS 5

static const char *TAG = "TCP_SERVER";

static void client_handler_task(void *pvParameters)
{
    int client_sock = *(int *)pvParameters;
    char buffer[128];
    int len;

    while ((len = recv(client_sock, buffer, sizeof(buffer) - 1, 0)) > 0) {
        buffer[len] = '0';
        ESP_LOGI(TAG, "Received: %s", buffer);
        send(client_sock, buffer, len, 0);
    }

    if (len < 0) {
        ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
    }

    ESP_LOGI(TAG, "Client disconnected");
    close(client_sock);
    vTaskDelete(NULL);
}

// Declare tcp_server_task before tcp_server_start
static void tcp_server_task(void *pvParameters);

void tcp_server_start(void)
{
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}

static void tcp_server_task(void *pvParameters)
{
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    err = listen(listen_sock, MAX_CLIENTS);
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket listening on port %d", PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            continue;
        }

        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_AP, mac);
        ESP_LOGI(TAG, "New client connected, MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        xTaskCreate(client_handler_task, "client_handler", 4096, (void *)&client_sock, 5, NULL);
    }

    close(listen_sock);
    vTaskDelete(NULL);
}