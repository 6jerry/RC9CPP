#include "EncodingStateMachine.h"

// 构造函数：初始化位宽和偏移量，设置状态表
EncodingStateMachine::EncodingStateMachine(const uint8_t *bitWidths, size_t numFlags)
    : numFlags(numFlags)
{
    if (numFlags > MAX_FLAGS)
    {
        numFlags = 0; // 错误处理，避免溢出
        return;
    }

    uint8_t shiftAmount = 0;
    for (size_t i = 0; i < numFlags; ++i)
    {
        this->bitWidths[i] = bitWidths[i];
        offsets[i] = shiftAmount;
        shiftAmount += bitWidths[i];
    }

    // 初始化状态表为无效状态
    for (uint16_t i = 0; i < maxIndex; ++i)
    {
        stateTable[i] = invalidState;
    }
}

// 设置状态到索引的映射
bool EncodingStateMachine::mapStateToIndices(uint8_t state, const uint16_t indices[], size_t numIndices)
{
    for (size_t i = 0; i < numIndices; ++i)
    {
        if (indices[i] >= maxIndex)
        {
            return false; // 索引超出范围
        }
        stateTable[indices[i]] = state;
    }
    return true;
}

// 计算索引
uint16_t EncodingStateMachine::calculateIndex(const uint8_t *flagValues) const
{
    uint16_t index = 0;
    for (size_t i = 0; i < numFlags; ++i)
    {
        uint8_t value = flagValues[i] & ((1 << bitWidths[i]) - 1);
        index |= value << offsets[i];
    }
    return index;
}

// 获取状态
uint8_t EncodingStateMachine::getState(const uint8_t *flagValues) const
{
    uint16_t index = calculateIndex(flagValues);
    if (index < maxIndex)
    {
        return stateTable[index];
    }
    return invalidState;
}
