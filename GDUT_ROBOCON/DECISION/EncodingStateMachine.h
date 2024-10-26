#ifndef ENCODING_STATE_MACHINE_H
#define ENCODING_STATE_MACHINE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "TaskManager.h" // 假设你需要这个头文件

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus



// 定义标志位配置结构体
struct FlagConfig
{
    uint8_t *flagPtr; // 指向标志位的指针
    uint8_t maxValue; // 标志位的最大值
    uint8_t bitWidth; // 位宽
    uint8_t offset;   // 偏移量
};

// 定义状态机类
class EncodingStateMachine
{
public:
    // 定义功能函数类型
    typedef void (*StateFunction)();

    // 构造函数，初始化标志位配置数组
    EncodingStateMachine(FlagConfig flagConfigs[], size_t numFlags);

    // 设置某个状态的功能函数
    void setStateFunction(uint16_t index, StateFunction func);

    // 根据当前标志位的值计算索引并执行相应状态的功能函数
    void executeCurrentState() const;

private:
    // 根据标志位的当前值计算索引
    uint16_t calculateIndex() const;

    FlagConfig *flagConfigs; // 标志位配置数组
    size_t numFlags;         // 标志位的数量

    static const size_t maxStates = 32;      // 假设最多支持32种状态
    StateFunction stateFunctions[maxStates]; // 功能函数表
};

#endif
#endif // ENCODING_STATE_MACHINE_H
