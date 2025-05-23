/*******************************************************************************
 * @file EncodingStateMachine.h
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
#ifndef ENCODING_STATE_MACHINE_H
#define ENCODING_STATE_MACHINE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "TaskManager.h"

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
    static const uint16_t maxIndex = 128;    // 假设最大索引值为 64
    static const uint8_t invalidState = 255; // 无效状态定义为 255

    // 构造函数，初始化标志位配置数组
    EncodingStateMachine(FlagConfig flagConfigs[], size_t numFlags);

    // 设置某个状态的索引值映射
    bool mapStateToIndices(uint8_t state, const uint16_t indices[], size_t numIndices);

    // 根据当前标志位的值返回对应的状态值，如果找不到则返回 255 (表示无效状态)
    uint8_t getState() const;

    // 根据标志位的当前值计算索引
    uint16_t calculateIndex() const;

private:
    FlagConfig *flagConfigs; // 标志位配置数组
    size_t numFlags;         // 标志位的数量

    uint8_t stateTable[maxIndex]; // 用于快速查找状态的表，大小为 maxIndex
};

#endif
#endif // ENCODING_STATE_MACHINE_H
