#include "spi.h"
#include "esp_log.h"

static const char *TAG = "SPI Manager";
static spi_rx_callback_t user_rx_callback = NULL;
static void *user_callback_context = NULL;

static void IRAM_ATTR spi_post_transfer_callback(spi_transaction_t *trans);
static void IRAM_ATTR spi_post_slave_transfer_callback(spi_slave_transaction_t *trans);

esp_err_t spi_init(const spi_config_t *config, spi_device_handle_t *dev_handle) {
    esp_err_t ret;
    
    if (!config || !dev_handle) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->mode == SPI_MODE_MASTER) {
        // 主机模式初始化
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = config->mosi_pin,
            .miso_io_num = config->miso_pin,
            .sclk_io_num = config->sclk_pin,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = config->max_transfer_size,
            .flags = SPICOMMON_BUSFLAG_MASTER,
        };

        if ((ret = spi_bus_initialize(config->host, &bus_cfg, SPI_DMA_CH_AUTO)) != ESP_OK) {
            ESP_LOGE(TAG, "Bus initialize failed: %s", esp_err_to_name(ret));
            return ret;
        }

        spi_device_interface_config_t dev_cfg = {
            .mode = 0,
            .clock_speed_hz = config->clock_speed,
            .spics_io_num = config->cs_pin,
            .queue_size = config->queue_size ? config->queue_size : 7,
            .flags = SPI_DEVICE_NO_DUMMY,
            .post_cb = spi_post_transfer_callback,
            .clock_source = config->clk_src,
        };

        user_rx_callback = config->rx_callback;
        user_callback_context = config->callback_context;

        if ((ret = spi_bus_add_device(config->host, &dev_cfg, dev_handle)) != ESP_OK) {
            ESP_LOGE(TAG, "Add device failed: %s", esp_err_to_name(ret));
            spi_bus_free(config->host);
            return ret;
        }
    } else if (config->mode == SPI_MODE_SLAVE) {
        // 从机模式初始化
        spi_slave_interface_config_t slv_cfg = {
            .mode = 0,
            .spics_io_num = config->cs_pin,
            .queue_size = config->queue_size ? config->queue_size : 7,
            .flags = 0,
            .post_setup_cb = NULL,
            .post_trans_cb = spi_post_slave_transfer_callback,
        };

        spi_bus_config_t bus_cfg = {
            .mosi_io_num = config->mosi_pin,
            .miso_io_num = config->miso_pin,
            .sclk_io_num = config->sclk_pin,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = config->max_transfer_size,
        };

        user_rx_callback = config->rx_callback;
        user_callback_context = config->callback_context;

        if ((ret = spi_slave_initialize(config->host, &bus_cfg, &slv_cfg, SPI_DMA_CH_AUTO)) != ESP_OK) {
            ESP_LOGE(TAG, "Slave initialize failed: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        ESP_LOGE(TAG, "Invalid SPI mode");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "SPI %s initialized successfully", 
            config->mode == SPI_MODE_MASTER ? "Master" : "Slave");
    return ESP_OK;
}

static void spi_post_transfer_callback(spi_transaction_t *trans) {
    if (user_rx_callback && trans->rx_buffer) {
        uint8_t *rx_data = (uint8_t *)trans->rx_buffer;
        size_t data_len = trans->rxlength / 8;
        user_rx_callback(user_callback_context, rx_data, data_len);
    }
}

static void spi_post_slave_transfer_callback(spi_slave_transaction_t *trans) {
    if (user_rx_callback && trans->rx_buffer) {
        uint8_t *rx_data = (uint8_t *)trans->rx_buffer;
        size_t data_len = trans->trans_len / 8;
        user_rx_callback(user_callback_context, rx_data, data_len);
    }
}

esp_err_t spi_transfer(spi_device_handle_t dev, 
                      uint8_t *tx_data, 
                      uint8_t *rx_data,
                      size_t length) {
    spi_transaction_t trans = {
        .length = length * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    return spi_device_transmit(dev, &trans);
}