
/*
 * @author loopgad
 * @contact 3280646246@qq.com
 * @license MIT License
 *
 * Copyright (c) 2024 loopgad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "laser_processor.h"
#include <cstring>

#define max_distance 0.5f
#define min_distance 0.045f



static float my_fabs(float x){
	return x < 0 ? -x : x;
}
LaserProcessor::LaserProcessor() : rx_length_(0), resolution_(2) {
    // 初始化滤波窗口
    memset(filter_window_, 0, sizeof(filter_window_));
}


float LaserProcessor::ProcessByte(uint8_t byte) {
    // 将字节存入缓冲区
    if (rx_length_ >= sizeof(rx_buffer_)) {
        rx_length_ = 0;  // 缓冲区满，重置
    }
    rx_buffer_[rx_length_++] = byte;

    // 检查是否可能形成有效包
    if (rx_length_ < 3) {
        return -1.0f;  // 包头未完整，继续接收
    }

    // 检查包头中的ADDR（默认0x80）
    if (rx_buffer_[0] != 0x80) {
        rx_length_ = 0;  // 非测量包，重置
        return -1.0f;
    }

    uint8_t cmd1 = rx_buffer_[1];
    uint8_t cmd2 = rx_buffer_[2];

    // 仅处理单次/连续测量返回包
    if ((cmd1 == 0x06 && (cmd2 == 0x82 || cmd2 == 0x83))) {
        // 根据分辨率确定数据长度
        uint16_t data_length = (resolution_ == 1) ? 7 : 8;
        uint16_t expected_length = 3 + data_length + 1;  // 包头3 + 数据 + 校验和1

        if (rx_length_ < expected_length) {
            return -1.0f;  // 数据未完整，继续接收
        }

        // 校验和验证
        uint8_t checksum = CalculateChecksum(rx_buffer_, expected_length - 1);
        if (checksum != rx_buffer_[expected_length - 1]) {
            rx_length_ = 0;
            return -1.0f;  // 校验失败
        }

        // 解析数据
        float distance = ParseAsciiDistance(rx_buffer_ + 3, data_length);
        rx_length_ = 0;  // 清空缓冲区

        // 更新滤波窗口
        filter_window_[window_index_] = distance;
        window_index_ = (window_index_ + 1) % FILTER_WINDOW_SIZE;
        if (data_count_ < FILTER_WINDOW_SIZE) {
            data_count_++;
        }

        // 计算滤波值
        float sum = 0;
        for (uint8_t i = 0; i < data_count_; i++) {
            sum += filter_window_[i];
        }
        float filtered = sum / data_count_;

        // 应用最大变化率限制
        if (my_fabs(filtered - last_data_) > max_distance) {
            last_data_ = filtered;
            return last_data_;
        } else if (filtered < min_distance) {
            last_data_ = min_distance;
            return min_distance;
        }

        last_data_ = filtered;
        return filtered;
    } else {
        rx_length_ = 0;  // 非测量包，重置
        return -1.0f;
    }
}
float LaserProcessor::get_distance(uint8_t byte){
	static float last_distance = 0.04f;
	float distance = ProcessByte(byte);
	distance =  distance < 0 ? last_distance : distance;
	if(my_fabs(distance-last_distance)> max_distance){
		return last_distance;
	}
	last_distance = distance;
	return distance;
}


uint8_t LaserProcessor::CalculateChecksum(const uint8_t* data, uint16_t len) {
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return static_cast<uint8_t>(~sum + 1);
}

float LaserProcessor::ParseAsciiDistance(const uint8_t* start, uint16_t bytes) {
    char buffer[16] = {0};
    uint8_t max_len = (bytes > 15) ? 15 : bytes;

    for (uint8_t i = 0; i < max_len; ++i) {
        buffer[i] = static_cast<char>(start[i]);
    }

    int32_t integer_part = 0;
    int32_t fractional_part = 0;
    float divisor = 1.0f;
    bool negative = false;
    bool is_fraction = false;
    uint8_t pos = 0;

    if (buffer[0] == '-') {
        negative = true;
        pos = 1;
    }

    for (; pos < max_len; ++pos) {
        char c = buffer[pos];
        if (c == '.') {
            is_fraction = true;
            continue;
        }
        if (c >= '0' && c <= '9') {
            if (is_fraction) {
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

