// laser_processor.h
#ifndef LASER_PROCESSOR_H
#define LASER_PROCESSOR_H

#include <cstdint>

#define CMD_GROUP_SIZE 4
#define FILTER_WINDOW_SIZE 4

class LaserProcessor {
public:
    // 初始化命令组访问接口
    const char* const* GetInitCommands() const {
        return init_commands_;
    }

    // 处理接收到的原始数据
    float ProcessData(const uint8_t* data, uint16_t len);

private:
    // 初始化命令组（十六进制字符串格式）
    const char* init_commands_[CMD_GROUP_SIZE] = {
        "8006050174",   // 开启激光
        "FA040C02F4",   // 0.1mm分辨率
        "FA040A14E4",   // 20Hz频率
        "80060377"      // 连续测量
    };

    // 滤波窗口管理
    float filter_window_[FILTER_WINDOW_SIZE] = {0};
    uint8_t data_count_ = 0;
    uint8_t window_index_ = 0;

    // 协议解析方法
    bool ValidatePacket(const uint8_t* data, uint16_t len);
    float ParseAsciiDistance(const uint8_t* start, uint16_t bytes);
};

#endif