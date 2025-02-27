// laser_processor.cpp
#include "laser_processor.h"

#define max_distance 0.5f
#define min_distance 0.045f

//my_fabs()
float my_fabs(float x){
    return x < 0 ? -x : x;
}

// 校验和计算（与协议一致）
static uint8_t CalculateChecksum(const uint8_t* data, uint16_t len) {
    uint16_t sum = 0;
    for(uint16_t i=0; i<len; i++) {
        sum += data[i];
    }
    return static_cast<uint8_t>(~sum + 1);
}

float LaserProcessor::ProcessData(const uint8_t* data, uint16_t len) {
    if(!ValidatePacket(data, len)) {
        return -1.0f; // 无效数据标志
    }

    // 提取ASCII数据部分（假设数据从第4字节开始）
    const uint8_t* ascii_start = data + 3;
    uint16_t ascii_len = len - 4; // 排除包头3字节和校验1字节
    static float last_data = 0;

    // ASCII转浮点数
    float distance = ParseAsciiDistance(ascii_start, ascii_len);
    
    // 滑动窗口滤波
    filter_window_[window_index_] = distance;
    window_index_ = (window_index_ + 1) % FILTER_WINDOW_SIZE;
    
    // 更新有效数据计数
    if(data_count_ < FILTER_WINDOW_SIZE) {
        data_count_++;
    }

    // 计算滤波值（数据不足时使用当前有效数据）
    float sum = 0;
    for(uint8_t i=0; i<data_count_; i++) {
        sum += filter_window_[i];
    }
    float filter_data = sum / data_count_;
    if(my_fabs(filter_data - last_data) > max_distance){
        return last_data;
    }
    else if(filter_data < min_distance){
        last_data = filter_data;
        return min_distance;
    }
    last_data = filter_data;
    return filter_data;
}

bool LaserProcessor::ValidatePacket(const uint8_t* data, uint16_t len) {
    // 基础长度检查
    if(len < 7) return false; // 最小有效包长度
    
    // 校验包头（假设标准返回格式）
    if(data[0] != 0x80 || data[1] != 0x06 || data[2] != 0x83) {
        return false;
    }
    
    // 校验码验证
    uint8_t checksum = CalculateChecksum(data, len-1);
    return (checksum == data[len-1]);
}

float LaserProcessor::ParseAsciiDistance(const uint8_t* start, uint16_t bytes) {
    char buffer[16] = {0};
    const uint8_t max_len = (bytes > 15) ? 15 : bytes;

    // Copy valid ASCII characters
    for(uint8_t i = 0; i < max_len; ++i) {
        buffer[i] = static_cast<char>(start[i]);
    }

    int32_t integer_part = 0;
    int32_t fractional_part = 0;
    float divisor = 1.0f;
    bool negative = false;
    bool is_fraction = false;
    uint8_t pos = 0;

    // Handle negative sign
    if(buffer[0] == '-') {
        negative = true;
        pos = 1;
    }

    // Single pass parsing loop
    for(; pos < max_len; ++pos) {
        const char c = buffer[pos];
        
        if(c == '.') {
            is_fraction = true;
            continue;
        }

        if(c >= '0' && c <= '9') {
            if(is_fraction) {
                fractional_part = fractional_part * 10 + (c - '0');
                divisor *= 10.0f;
            } else {
                integer_part = integer_part * 10 + (c - '0');
            }
        }
    }

    float result = integer_part + (fractional_part / divisor);
    return negative ? -result : result;
}