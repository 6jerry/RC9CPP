#include "esp_uart.h"
static const char *TAG = "uart_events";

#define PATTERN_CHR_NUM  (3)    // 用于检测的连续字符数量
#define BUF_SIZE         (2048)
#define RD_BUF_SIZE      (BUF_SIZE)

bool fail_flag = false; // 用于标记失败状态
int sent_fail_count = 0; // 用于计数
int64_t start = 0,uart_delay = 0,udp_delay = 0; // 用于计数;

rc9_parser_t parser;
bool update_xbox = false;
// 任务参数结构体
typedef struct {
    uart_port_t uart_num;      // UART端口号
    QueueHandle_t uart_queue;  // 事件队列句柄
} uart_task_params_t;

/**
 * @brief UART事件处理任务
 */
static void uart_event_task(void *pvParameters) {
    uart_task_params_t *params = (uart_task_params_t *)pvParameters;
    uart_port_t uart_num = params->uart_num;
    QueueHandle_t uart_queue = params->uart_queue;

    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
    
    for (;;) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            memset(dtmp, 0, RD_BUF_SIZE);
            //ESP_LOGI(TAG, "uart[%d] event:", uart_num);
            
            switch (event.type) {
                case UART_DATA:
                    //ESP_LOGI(TAG, "[UART DATA]: %d bytes received", event.size);
                    
                    // 读取数据
                    int read_len = uart_read_bytes(uart_num, dtmp, event.size, portMAX_DELAY);
                    if (read_len <= 0) {
                        ESP_LOGE(TAG, "Read error: %d", read_len);
                        break;
                    }
                        // 精简后的处理逻辑
                    for (int i = 0; i < read_len; i++) {
                        parse_rc9_byte(dtmp[i]);  // 核心解析函数调用
                    }
                    break;
                
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input(uart_num);
                    xQueueReset(uart_queue);
                    break;
                
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input(uart_num);
                    xQueueReset(uart_queue);
                    break;
                
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(uart_num, &buffered_size);
                    int pos = uart_pattern_pop_pos(uart_num);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    
                    if (pos == -1) {
                        uart_flush_input(uart_num);
                    } else {
                        uart_read_bytes(uart_num, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(uart_num, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    
    free(dtmp);
    free(params);
    vTaskDelete(NULL);
}

/**
 * @brief 初始化UART
 * @param uart_num UART端口号（UART_NUM_0或UART_NUM_1）
 * @param baud_rate 波特率
 * @param tx_pin TX引脚号
 * @param rx_pin RX引脚号
 */
void init_uart(uart_port_t uart_num,int baud_rate, int tx_pin, int rx_pin) {
    // UART配置参数
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 安装UART驱动
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(
        uart_num,
        BUF_SIZE * 4,
        BUF_SIZE * 4,
        1,
        &uart_queue,
        0
    ));
    
    // 配置参数和引脚
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(
        uart_num,
        tx_pin,
        rx_pin,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    // 配置模式检测
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(
        uart_num,
        '+',
        PATTERN_CHR_NUM,
        9, 0, 0
    ));
    ESP_ERROR_CHECK(uart_pattern_queue_reset(uart_num, 20));

    // 创建任务参数
    uart_task_params_t *params = (uart_task_params_t *)malloc(sizeof(uart_task_params_t));
    params->uart_num = uart_num;
    params->uart_queue = uart_queue;

    rc9_parser_init(process_rc9_frame,params->uart_num);

    esp_pm_lock_handle_t pm_lock;
    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, NULL, &pm_lock);
    esp_pm_lock_acquire(pm_lock);

    // 创建事件处理任务
    xTaskCreate(
        uart_event_task,
        "uart_event_task",
        4096,
        params,
        configMAX_PRIORITIES-1,
        NULL
    );
}
uint64_t pack_cnt=0;
uint64_t pack_err_cnt=0;
// 状态机处理函数（新增核心函数）
void parse_rc9_byte(uint8_t byte) {
    switch (parser.state) {
        case RC9_STATE_WAIT_HEAD1:
            if (byte == FRAME_HEAD_0_RC9) {
                parser.state = RC9_STATE_WAIT_HEAD2;
                pack_cnt++; 
            }
            break;
            
        case RC9_STATE_WAIT_HEAD2:
            if (byte == FRAME_HEAD_1_RC9) {
                parser.state = RC9_STATE_READ_ID;
                memset(&parser.current_frame, 0, sizeof(rc9_frame_t));
                pack_err_cnt++;
            } else {
                parser.state = RC9_STATE_WAIT_HEAD1;
            }
            break;
            
        case RC9_STATE_READ_ID:
            parser.current_frame.frame_id = byte;
            parser.state = RC9_STATE_READ_LEN;
            break;
            
        case RC9_STATE_READ_LEN:
            parser.current_frame.data_length = byte;
            parser.data_index = 0;
            parser.state = (byte > 0) ? RC9_STATE_READ_DATA : RC9_STATE_READ_CRC1;
            break;
            
        case RC9_STATE_READ_DATA:
            if (parser.data_index < parser.current_frame.data_length) {
                parser.current_frame.data[parser.data_index++] = byte;
            }
            if (parser.data_index >= parser.current_frame.data_length) {
                parser.state = RC9_STATE_READ_CRC1;
            }
            break;
            
        case RC9_STATE_READ_CRC1:
            parser.current_frame.crc = byte << 8;
            parser.state = RC9_STATE_READ_CRC2;
            break;
            
        case RC9_STATE_READ_CRC2:
            parser.current_frame.crc |= byte;
            parser.state = RC9_STATE_READ_END1;
            break;
            
        case RC9_STATE_READ_END1:
            if (byte == FRAME_END_0_RC9) {
                parser.state = RC9_STATE_READ_END2;
            } else {
                ESP_LOGI(TAG, "ERRO_DATA!");
                rc9_reset_parser();
                pack_err_cnt++;
            }
            break;
            
        case RC9_STATE_READ_END2:
            if (byte == FRAME_END_1_RC9) {
                // 完整帧接收完成，验证CRC
                // const uint16_t calc_crc = CRC16_Table(
                //     parser.current_frame.data,
                //     parser.current_frame.data_length
                // );
                
                // if (calc_crc == parser.current_frame.crc && parser.user_callback) {
                //     parser.user_callback(&parser.current_frame);
                // } else if (calc_crc != parser.current_frame.crc) {
                //     ESP_LOGE(TAG, "CRC Error: calc 0x%04X vs recv 0x%04X",
                //            calc_crc, parser.current_frame.crc);
                // }
                parser.user_callback(&parser.current_frame);
            }
            rc9_reset_parser();
            break;
    }
}
int sock;
bool init_ = false;
struct sockaddr_in *server_addr;
void init_udp_client(int sock_, struct sockaddr_in *server_addr__) {
    start = esp_timer_get_time();
    sock = sock_;
    server_addr = server_addr__;
    init_ = true;
}


// 重置解析器状态
void rc9_reset_parser(void) {
    parser.state = RC9_STATE_WAIT_HEAD1;
    memset(&parser.current_frame, 0, sizeof(rc9_frame_t));
    parser.data_index = 0;
}

// 初始化解析器
void  rc9_parser_init(rc9_callback_t callback,uart_port_t uart_num){
    rc9_reset_parser();
    parser.user_callback = callback;
    parser.uart_num = uart_num;
}

// 添加数据处理回调函数
void process_rc9_frame(const rc9_frame_t *frame){
    // 在这里处理解析完成的有效数据包
    // ESP_LOGI(TAG, "Valid frame received! ID:0x%02X Len:%d", 
    //         frame->frame_id, frame->data_length);

    //ESP_LOGI(TAG, "Packet loss rate: %0.2f%%", (float)pack_err_cnt/(float)pack_cnt);
    update_xbox = true;

    if(esp_timer_get_time() - uart_delay > 5000){
        // uart_write_bytes(UART_NUM_0, (const char*)frame->data,  frame->data_length);
        // uart_write_bytes(UART_NUM_0, "\r\n",  2);
        uart_delay = esp_timer_get_time();
        }
    //free(str);
    
    if(init_){
        if(esp_timer_get_time() - udp_delay > 25000){
            udp_delay = esp_timer_get_time();
            if (!udp_communication_test_loop(sock, server_addr)) {
                sent_fail_count++;
                if (sent_fail_count >= 10) { // 连续5次发送失败
                    ESP_LOGE(TAG, "UDP communication failed. Restarting...");
                    fail_flag = true; // 设置失败标志
                }
            }else if(sent_fail_count >= 0){
                    sent_fail_count = 0;
            }
        }

    }
}
