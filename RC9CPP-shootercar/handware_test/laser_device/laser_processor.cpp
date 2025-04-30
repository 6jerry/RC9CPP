
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

static float my_fabs(float x){
	return x < 0 ? -x : x;
}
// 协议参数
constexpr float max_distance = 0.5f;
constexpr float min_distance = 0.03f;
 
LaserProcessor::LaserProcessor(UART_HandleTypeDef *huart_) : 
    SerialDevice(huart_), rx_length_(0), resolution_(2), data_count_(0), 
    window_index_(0), last_data_(0){
    memset(filter_window_, 0, sizeof(filter_window_));
    memset(cmd_tracker_, 0, sizeof(cmd_tracker_));
}

uint8_t LaserProcessor::CalculateChecksum(const uint8_t* data, uint16_t len) {
    // 检查长度是否超过最大值
    if (len > 12) {
        return 0; // 返回一个错误值
    }

    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return static_cast<uint8_t>(~sum + 1);
}

float LaserProcessor::ProcessByte(uint8_t byte) {
    // 存入缓冲区并防止溢出
    if (rx_length_ < sizeof(rx_buffer_)) {
        rx_buffer_[rx_length_++] = byte;
    } else {
        rx_length_ = 0;  // 缓冲区满强制清空（协议最大包长保护）
    }

    // 实时地址校验（协议文档默认地址规则）
    if (rx_length_ >= 1 && rx_buffer_[0] != 0x80 && rx_buffer_[0] != 0xFA) {
        rx_length_ = 0;  // 非法地址立即清空（协议第3节地址规范）
        return -1.0f;
    }

    // 优先处理测量数据包（协议表第8-9行）
    if (rx_length_ >= 3 && rx_buffer_[0] == 0x80) {  // 测量包固定地址
        const uint8_t cmd1 = rx_buffer_[1];
        const uint8_t cmd2 = rx_buffer_[2];
        
        // 校验测量命令格式（协议表第8-9行命令码）
        if (cmd1 == 0x06 && (cmd2 == 0x82 || cmd2 == 0x83)) {
            const uint8_t data_len = (resolution_ == 1) ? 7 : 8;  // 协议表分辨率说明
            const uint8_t pkg_len = 3 + data_len + 1;  // ADDR+CMD+DATA+CS
            
            if (rx_length_ >= pkg_len) {
                // 校验和验证（协议第3节校验规则）
                if (CalculateChecksum(rx_buffer_, pkg_len-1) == rx_buffer_[pkg_len-1]) {
                    // ASCII数据解析（协议第4节数据格式）
                    float dist = ParseAsciiDistance(rx_buffer_+3, data_len);
                    
                    // 移动平均滤波（协议未要求，用户需求保留）
                    filter_window_[window_index_] = dist;
                    window_index_ = (window_index_ + 1) % FILTER_WINDOW_SIZE;
                    data_count_ = (data_count_+1  < FILTER_WINDOW_SIZE) ? data_count_+1 : FILTER_WINDOW_SIZE;
                    
                    // 计算滤波值
                    float filtered = 0;
                    for (uint8_t i=0; i<data_count_; ++i)
                        filtered += filter_window_[i];
                    filtered /= data_count_;
                    
                    // 变化率限制（根据协议ERR-26错误码设计）
                    if (my_fabs(filtered - last_data_) > max_distance) {
                        last_data_ = filtered;
                    } else if (filtered < min_distance) {
                        last_data_ = min_distance;
                    } else {
                        last_data_ = filtered;
                    }
                    
                    rx_length_ = 0;  // 成功处理清空缓冲区
                    return last_data_;
                }
                rx_length_ = 0;  // 校验失败清空缓冲区
                return -1.0f;
            }
            return -1.0f;  // 数据未接收完整
        }
        rx_length_ = 0;  // 非测量命令清空缓冲区
        return -1.0f;
    }

    // 次优先处理命令响应（协议表第3列返回代码）
    if (ValidatePacket(rx_buffer_, rx_length_)) {
        ProcessCmdResponse(rx_buffer_, rx_length_);
        rx_length_ = 0;  // 处理完成必须清空
        return -1.0f;
    }

    // 超长包保护（协议最大响应包长度检测）
    if (rx_length_ >= 5) {  // 协议错误响应最大长度FA 84 81 02 FF
        rx_length_ = 0;  // 超过最大长度仍未通过验证
    }

    return -1.0f;
}
 
bool LaserProcessor::ValidatePacket(const uint8_t* data, uint16_t len) {
    // 基本验证
    if(len <4 || (data[0]!=0x80 && data[0]!=0xFA)) 
        return false;
    
    // 校验和验证
    const uint8_t calc_cs = CalculateChecksum(data, len-1);
    return (calc_cs == data[len-1]);
}
 
void LaserProcessor::ProcessCmdResponse(const uint8_t* data, uint8_t len) {
    for(int i=0; i<CMD_GROUP_SIZE; ++i) {
        const Command& cmd = init_commands_[i];
        if(len >=3 && memcmp(data, cmd.data, 3) == 0) {
            cmd_tracker_[i].status = (data[2] & 0x80) ? CMD_DEVICE_ERR : CMD_SUCCESS;
            break;
        }
    }
}
 

 
LaserProcessor::CmdStatus LaserProcessor::GetCmdStatus(int index) const {
    return (index >=0 && index<CMD_GROUP_SIZE) ? 
           cmd_tracker_[index].status : CMD_TIMEOUT;
}
 
// ASCII解析实现保持不变
float LaserProcessor::ParseAsciiDistance(const uint8_t* start, uint16_t bytes) {
    char buffer[16] = {0};
    const uint8_t max_len = (bytes > 15) ? 15 : bytes;
    memcpy(buffer, start, max_len);
    
    int32_t int_part = 0, frac_part = 0;
    float divisor = 1.0f;
    bool negative = false, is_frac = false;
    uint8_t pos = 0;
    
    if(buffer[0] == '-') { negative = true; pos = 1; }
    
    for(; pos<max_len; ++pos) {
        const char c = buffer[pos];
        if(c == '.') { is_frac = true; continue; }
        if(c >= '0' && c <= '9') {
            if(is_frac) {
                frac_part = frac_part*10 + (c-'0');
                divisor *= 10.0f;
            } else {
                int_part = int_part*10 + (c-'0');
            }
        }
    }
    return negative ? -(int_part + frac_part/divisor) : (int_part + frac_part/divisor);
}

//数据后处理
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

void LaserProcessor::handleReceiveData(uint8_t byte){
    laser_distance = get_distance(byte);
}
