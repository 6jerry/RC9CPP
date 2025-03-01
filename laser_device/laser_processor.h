// laser_processor.h
#ifndef LASER_PROCESSOR_H
#define LASER_PROCESSOR_H

#ifndef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <cstdint>



class LaserProcessor {
public:
    LaserProcessor();
    const static uint8_t CMD_GROUP_SIZE  = 4;
	const static uint8_t FILTER_WINDOW_SIZE  = 4;
    // 保持原有公共接口
    struct Command {
        const uint8_t* data;
        uint8_t length;
    };
	
	const Command init_commands_[CMD_GROUP_SIZE] = {
		// 激光控制命令（协议表第15行）
		{ 
			.data = (const uint8_t[]){0x80, 0x06, 0x05, 0x01, 0x74}, // 80 06 05 01 74
			.length = 5  // 实际字节数：5
		},
		// 分辨率设置（协议表第13行）
		{ 
			.data = (const uint8_t[]){0xFA, 0x04, 0x0C, 0x02, 0xF4}, // FA 04 0C 02 F4
			.length = 5  // 包含完整校验位
		},
		// 频率设置（协议表第11行扩展）
		{ 
			.data = (const uint8_t[]){0xFA, 0x04, 0x0A, 0x14, 0xE4}, // FA 04 0A 14 E4
			.length = 5  // 文档中完整5字节命令
		},
		// 连续测量（协议表第7行）
		{ 
			.data = (const uint8_t[]){0x80, 0x06, 0x03, 0x77}, // 80 06 03 77
			.length = 4  // 文档明确显示4字节命令
		}
	};
	
	 // 新增错误状态查询接口
    enum CmdStatus {
        CMD_PENDING,    // 等待响应
        CMD_SUCCESS,    // 成功
        CMD_CHECKSUM_ERR, 
        CMD_TIMEOUT,    
        CMD_DEVICE_ERR  
    };
	
    // 命令跟踪结构
    struct {
        CmdStatus status = CMD_PENDING;
        uint8_t retries = 0;
        uint32_t sent_time = 0;
    } cmd_tracker_[CMD_GROUP_SIZE];
	
    const Command* InitCommands() const { return init_commands_; }
    static constexpr int InitCommandCount() { return CMD_GROUP_SIZE; }
    
    float get_distance(uint8_t byte);
    
    CmdStatus GetCmdStatus(int index) const;

private:
    // 原有私有成员
    uint8_t rx_buffer_[32];
    uint16_t rx_length_;
    uint8_t resolution_;
    float filter_window_[FILTER_WINDOW_SIZE];
    uint8_t data_count_;
    uint8_t window_index_;
    float last_data_;


    // 原有私有方法
    bool ValidatePacket(const uint8_t* data, uint16_t len);
    float ParseAsciiDistance(const uint8_t* start, uint16_t bytes);
    uint8_t CalculateChecksum(const uint8_t* data, uint16_t len);
    float ProcessByte(uint8_t byte);
    
    // 新增响应处理方法
    void ProcessCmdResponse(const uint8_t* data, uint8_t len);
};

extern LaserProcessor laser;

#ifndef __cplusplus
}
#endif

#endif