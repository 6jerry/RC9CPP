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

class EncodingStateMachine
{
public:
    static const uint16_t maxIndex = 128;    // 最大索引值
    static const uint8_t invalidState = 255; // 无效状态
    static const size_t MAX_FLAGS = 10;      // 最大标志位数量

    // 构造函数：传入位宽数组和标志位数量
    EncodingStateMachine(const uint8_t *bitWidths, size_t numFlags);

    // 设置某个状态的索引值映射
    bool mapStateToIndices(uint8_t state, const uint16_t indices[], size_t numIndices);

    // 根据传入的标志位值返回状态
    uint8_t getState(const uint8_t *flagValues) const;

    // 根据传入的标志位值计算索引
    uint16_t calculateIndex(const uint8_t *flagValues) const;

private:
    uint8_t bitWidths[MAX_FLAGS]; // 存储各个标志位的位宽
    uint8_t offsets[MAX_FLAGS];   // 存储各个标志位的偏移量
    size_t numFlags;              // 标志位数量
    uint8_t stateTable[maxIndex]; // 状态查找表
};

#endif
#endif // ENCODING_STATE_MACHINE_H
