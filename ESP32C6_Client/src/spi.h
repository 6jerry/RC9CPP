#ifndef _SPI_H
#define _SPI_H
#include "esp_intr_alloc.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "esp_err.h"

// 定义操作模式
typedef enum {
    SPI_MODE_MASTER,
    SPI_MODE_SLAVE
} spi_mode_t;

// 定义中断回调函数类型
typedef void (*spi_rx_callback_t)(void *context, uint8_t *data, size_t len);

typedef struct {
    spi_mode_t mode;               // 新增模式选择
    spi_host_device_t host;
    int mosi_pin;
    int miso_pin;
    int sclk_pin;
    int max_transfer_size;
    uint32_t clock_speed;
    int cs_pin;
    spi_clock_source_t clk_src;
    spi_rx_callback_t rx_callback;
    void *callback_context;
    int queue_size;                // 新增队列大小配置
} spi_config_t;

/**
 * @brief 初始化SPI总线并添加设备
 * @param config SPI配置参数
 * @param dev_handle 输出参数，SPI设备句柄
 * @return esp_err_t 初始化结果
 */
esp_err_t spi_init(const spi_config_t *config, spi_device_handle_t *dev_handle);
esp_err_t spi_transfer(spi_device_handle_t dev, 
    uint8_t *tx_data, 
    uint8_t *rx_data,
    size_t length) ;
    
#endif