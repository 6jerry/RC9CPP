#ifndef FILTERS_H
#define FILTERS_H
#ifdef __cplusplus
extern "C"
{
#endif
#include <stdbool.h>
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class SimpleLowPassFilter
{
public:
    SimpleLowPassFilter(float alpha);
    float update(float rawData);
    void setAlpha(float newAlpha);

private:
    float alpha;      // 滤波系数（0~1）
    bool initialized; // 是否完成初始化
    float lastOutput; // 上一次滤波后的输出
    float output;     // 当前滤波输出
};
#endif
#endif