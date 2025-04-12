
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

#ifndef LASER_PROCESSOR_H
#define LASER_PROCESSOR_H


#ifndef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <cstdint>


#define CMD_GROUP_SIZE 4
#define FILTER_WINDOW_SIZE 4



class LaserProcessor {
public:
    LaserProcessor();
// 新增命令描述结构体
    struct Command {
        const uint8_t* data;
        uint8_t length;
    };


    const Command* InitCommands() const {
        return init_commands_;
    }
 
    // 新增命令数量获取
    static constexpr int InitCommandCount() {
        return CMD_GROUP_SIZE;
    }
	
	float get_distance(uint8_t byte);
private:
	
	// 命令集定义（直接使用协议文档中的HEX值）
	const uint8_t INIT_COMMANDS[CMD_GROUP_SIZE][8] = {
		// 开启激光（协议表第15行）
		{0x80, 0x06, 0x05, 0x01, 0x74},           // 80 06 05 01 74
		// 0.1mm分辨率（协议表第13行）
		{0xFA, 0x04, 0x0C, 0x02, 0xF4},           // FA 04 0C 02 F4
		// 20Hz频率（协议表第11行扩展）
		{0xFA, 0x04, 0x0A, 0x14, 0xE4},           // FA 04 0A 14 E4
		// 连续测量（协议表第7行）
		{0x80, 0x06, 0x03, 0x77}                  // 80 06 03 77
	};

	const uint8_t CMD_LENGTHS[CMD_GROUP_SIZE] = {
		5,  // 8006050174 实际字节长度
		5,  // FA040C02F4
		5,  // FA040A14E4
		4   // 80060377
	};
	// 命令集存储改为结构体数组
    Command init_commands_[CMD_GROUP_SIZE] = {
        {INIT_COMMANDS[0], CMD_LENGTHS[0]},
        {INIT_COMMANDS[1], CMD_LENGTHS[1]},
        {INIT_COMMANDS[2], CMD_LENGTHS[2]},
        {INIT_COMMANDS[3], CMD_LENGTHS[3]}
    };
	
	
    // 接收缓冲区及状态
    uint8_t rx_buffer_[32];
    uint16_t rx_length_;
    uint8_t resolution_;  // 当前分辨率：1=1mm，2=0.1mm
    // 滤波窗口管理
    float filter_window_[FILTER_WINDOW_SIZE] = {0};
    uint8_t data_count_ = 0;
    uint8_t window_index_ = 0;
    float last_data_ = 0;
	
    

    // 协议解析方法
    bool ValidatePacket(const uint8_t* data, uint16_t len);
    float ParseAsciiDistance(const uint8_t* start, uint16_t bytes);
    uint8_t CalculateChecksum(const uint8_t* data, uint16_t len);
	
	// 处理逐字节接收的数据
    float ProcessByte(uint8_t byte);


};

extern LaserProcessor laser;  // 全局对象

#ifndef __cplusplus
#define __cplusplus
}
#endif

#endif