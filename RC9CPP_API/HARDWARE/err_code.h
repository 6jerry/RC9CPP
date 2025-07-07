#ifndef ERR_CODE_H
#define ERR_CODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "TaskManager.h"
#include "RC9Protocol.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief 错误码枚举类型，用于统一表示系统运行状态和错误信息。
 *
 * 所有函数/方法应返回该类型的值以表明执行结果。
 */
enum err_code {
    ERR_CODE_WORK_SUCCESS = 0,         ///< 操作成功完成
    ERR_CODE_NOT_INITIALIZED = -1,     ///< 模块未初始化
    ERR_CODE_FAIL_INITIALIZED = -2,    ///< 初始化失败
    ERR_CODE_TIMEOUT = -3,             ///< 操作超时
    ERR_CODE_CONNECT_FAIL = -4,        ///< 连接失败
    ERR_CODE_DISCONNECT = -5,          ///< 设备断开连接
};

/**
 * @brief 设备标识符枚举，用于区分不同硬件模块。
 *
 * 每种设备在系统中拥有唯一的 ID，用于错误日志记录、调试追踪等。
 */
#define DEVICE_CLASS_NUM 8 	// 设备种类数
enum device_id {
    ERR_DEVICE_NONE = 0x00,
    ERR_DEVICE_MOTOR = 0x01,       ///< 电机
    ERR_DEVICE_PHOTOGATE = 0x02,       ///< 光电门传感器
    ERR_DEVICE_ROS = 0x03,           ///< ros通讯模块(radar或camera)
    ERR_DEVICE_POSITION = 0x04,        ///< 定位模块
    ERR_DEVICE_LASER = 0x05,           ///< 激光测距模块
    ERR_DEVICE_ULTRASONIC = 0x06,      ///< 超声波传感器
    ERR_DEVICE_SOLENOID_VALVE = 0x07,   ///< 电磁阀
};

class error_check { //错误检测基类

public:
    device_id err_id = ERR_DEVICE_NONE; // 子类需要重写
	bool timeout_flag = true; // 超时标志位(需要在子类进行手动置位)
    uint8_t instance_id = 0;
    error_check(); //默认构造函数
    void init(); // 初始化各种信息
    static void registerInstance(error_check *instance);
    virtual err_code check_error() = 0; // 虚函数，需要重写，每个设备自定义自己的错误检测机制
    err_code check_timeout(); //超时检测
};

class error_checker : public RC9subscriber, public ITaskProcessor { // 错误检测器
public:
    void process_data() override;
};

#endif
#endif