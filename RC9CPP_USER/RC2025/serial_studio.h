#ifndef SERIAL_STUDIO_DEBUG_H
#define SERIAL_STUDIO_DEBUG_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "encoder.h"
#include "serial_studio.h"
#include "wit_gyro.h"
#include "TaskManager.h"
#include "TrapezoidalPlanner.h"
#include "motor.h"
#include "RC9Protocol.h"
#ifdef __cplusplus

    /**
     *  serial studio
     *  Author： yanqy
     *  Todo ： 基于DataReceivedCallback实现一套接收数据并处理（可能会做）
     */
}
#endif
#ifdef __cplusplus
// 上层信息
typedef struct upperInfo
{
    uint8_t id = 1;
    uint8_t length;
    bool enable_flag = false;
    power_motor *shooter_motor = nullptr;
    power_motor *lifter_motor = nullptr;
    power_motor *turner_motor = nullptr;
    Encoder *encoder = nullptr;
    wit_gyro *wit_imu = nullptr;
};

// 底盘信息(TODO)
typedef struct chassisInfo
{
};
class serialStudio : public RC9subscriber, public ITaskProcessor
{
public:
    serialStudio();
    void add_upper_info(power_motor *shooter_motor_, power_motor *lifter_motor_, power_motor *turner_motor_,
                        Encoder *encoder_, wit_gyro *wit_imu_);
    void process_data();
    void send_float_debuginfo(uint8_t id, float *data, uint8_t length);
    void add_IO(RC9Protocol *port_);

    float temp_param[6] = {0.0f}; // 暂存上位机传过来的参数
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;

private:
    upperInfo upper_info;
};

#endif
#endif