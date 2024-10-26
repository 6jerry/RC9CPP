#include "EncodingStateMachine.h"

// 构造函数：预先计算各标志位的位宽和偏移量
EncodingStateMachine::EncodingStateMachine(FlagConfig flagConfigs[], size_t numFlags)
    : flagConfigs(flagConfigs), numFlags(numFlags)
{
    uint8_t shiftAmount = 0;

    // 初始化功能函数表为空
    for (size_t i = 0; i < maxStates; ++i)
    {
        stateFunctions[i] = nullptr;
    }

    // 计算每个标志位的位宽和偏移量
    for (size_t i = 0; i < numFlags; ++i)
    {
        uint8_t bitWidth = 0;
        uint8_t maxValue = flagConfigs[i].maxValue;

        // 计算位宽
        while (maxValue > 0)
        {
            maxValue >>= 1;
            bitWidth++;
        }

        // 设置位宽和偏移量
        flagConfigs[i].bitWidth = bitWidth;
        flagConfigs[i].offset = shiftAmount;
        shiftAmount += bitWidth;
    }
}

// 设置某个状态的功能函数
void EncodingStateMachine::setStateFunction(uint16_t index, StateFunction func)
{
    if (index < maxStates)
    {
        stateFunctions[index] = func;
    }
}

// 根据标志位的当前值计算索引
uint16_t EncodingStateMachine::calculateIndex() const
{
    uint16_t index = 0;

    for (size_t i = 0; i < numFlags; ++i)
    {
        // 使用预计算的偏移量和位宽
        uint8_t value = *flagConfigs[i].flagPtr & ((1 << flagConfigs[i].bitWidth) - 1);
        index |= value << flagConfigs[i].offset;
    }

    return index;
}

// 执行当前状态的功能函数
void EncodingStateMachine::executeCurrentState() const
{
    uint16_t index = calculateIndex();
    if (index < maxStates && stateFunctions[index])
    {
        stateFunctions[index](); // 执行对应的功能函数
    }
}
