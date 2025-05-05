#include "tcp_client.h"
#include "udp_client.h"
#include <stdbool.h>
#include "spi.h"
#include "esp_uart.h"
#include "nvs_flash.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_wifi.h"


//选择通讯模式
//#define USE_TCP
#define USE_UDP

bool spi_init_(void);
spi_device_handle_t spi_dev;

// spi接收回调函数
void spi_rx_handler(void *ctx, uint8_t *data, size_t len) {
    static const char *TAG = "SPI_RX";
    ESP_LOGI(TAG, "Received %d bytes\n", len);
}

void app_main(void) {

    // 初始化UART0（使用默认引脚：TX=GPIO1, RX=GPIO3）
    //init_uart(UART_NUM_0, 1, 3);

    // 若要初始化UART1
    // init_uart(UART_NUM_1,921600*4, 20, 21);
    init_uart(UART_NUM_1,921600*4, 12, 13);
    init_uart(UART_NUM_0, 115200, 16, 17);  // 使用默认日志UART0
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#ifdef USE_TCP
    xTaskCreate(tcp_client_task, "tcp_client_task", 4096, NULL, 5, NULL);
#elif defined(USE_UDP)
    // xTaskCreate(udp_client_task, "udp_client_task", 4096, NULL, 5, NULL);
    xTaskCreate(udp_client_task, "udp_client_task", 4096, NULL, configMAX_PRIORITIES-2, NULL);
    // 在初始化 WiFi 后调用
    esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AC | WIFI_PROTOCOL_11AX);  // 启用 11n/11ax
    esp_wifi_set_ps(WIFI_PS_NONE);
#else
    #error "Please define USE_TCP or USE_UDP"
#endif

}

bool spi_init_(void) {
    static const char *TAG = "SPI_INIT";

    // 主机初始化示例
    spi_config_t master_cfg = {
        .mode = SPI_MODE_MASTER,
        .host = SPI2_HOST,
        .mosi_pin = 11,
        .miso_pin = 13,
        .sclk_pin = 12,
        .max_transfer_size = 4092,
        .clock_speed = 1*1000*1000,
        .cs_pin = 10,
        .clk_src = SPI_CLK_SRC_DEFAULT,
        .rx_callback = spi_rx_handler,
        .callback_context = NULL,
        .queue_size = 7
    };

    // 从机初始化示例
    spi_config_t slave_cfg = {
        .mode = SPI_MODE_SLAVE,
        .host = SPI2_HOST,
        .mosi_pin = 13,  // 注意引脚方向
        .miso_pin = 12,  // 从机模式下可能需要交换
        .sclk_pin = 14,
        .max_transfer_size = 1024,
        .cs_pin = 15,
        .rx_callback = spi_rx_handler,
        .queue_size = 5
    };
    // 初始化SPI
    if (spi_init(&slave_cfg, &spi_dev) != ESP_OK) {
        ESP_LOGE(TAG, "SPI init failed\n");
        return false;
    } 
    ESP_LOGI(TAG, "SPI init success\n");
    return true;
}