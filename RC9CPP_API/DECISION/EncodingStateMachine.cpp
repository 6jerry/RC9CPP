/*******************************************************************************
 * @file EncodingStateMachine.cpp
 * @author 6Jerry (1517752988@qq.com)
 * @brief Multi-Flag Encoding State Machine aka MFESM
 * @version 1.0
 * @date 2024-10-27
 *
 * @copyright Copyright (c) 2024-10-27 6Jerry
 *
 * @license MIT
 *
 * @disclaimer This software is provided "as is", without warranty of any kind, express or implied,
 *             including but not limited to the warranties of merchantability, fitness for a
 *             particular purpose and noninfringement. In no event shall the authors be liable for any
 *             claim, damages or other liability, whether in an action of contract, tort or otherwise,
 *             arising from, out of or in connection with the software or the use or other dealings
 *             in the software.
 ******************************************************************************/
#include "EncodingStateMachine.h"
// 构造函数：预先计算各标志位的位宽和偏移量，并初始化状态表
EncodingStateMachine::EncodingStateMachine(FlagConfig flagConfigs[], size_t numFlags)
    : flagConfigs(flagConfigs), numFlags(numFlags)
{
    uint8_t shiftAmount = 0;

    // 初始化状态表为无效状态
    for (uint16_t i = 0; i < maxIndex; ++i)
    {
        stateTable[i] = invalidState;
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

// 设置某个状态的索引值映射
bool EncodingStateMachine::mapStateToIndices(uint8_t state, const uint16_t indices[], size_t numIndices)
{
    // 检查索引范围
    for (size_t i = 0; i < numIndices; ++i)
    {
        if (indices[i] >= maxIndex)
        {
            return false; // 索引超出范围
        }

        // 将索引值映射到对应的状态
        stateTable[indices[i]] = state;
    }
    return true;
}

// 根据当前标志位的值返回对应的状态值
uint8_t EncodingStateMachine::getState() const
{
    uint16_t index = calculateIndex();
    if (index < maxIndex)
    {
        return stateTable[index]; // 返回状态表中的状态值
    }
    return invalidState; // 返回无效状态
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
