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

#define MAX_WINDOW_SIZE 32

class SimpleLowPassFilter
{
public:
    SimpleLowPassFilter(float alpha = 1.0f);
    float update(float rawData);
    void setAlpha(float newAlpha);

private:
    float alpha;      // 滤波系数（0~1）
    bool initialized; // 是否完成初始化
    float lastOutput; // 上一次滤波后的输出
    float output;     // 当前滤波输出
};

class SimpleMedianFilter
{
public:
    // 默认构造函数，不分配内存，只初始化缓冲区和参数
    SimpleMedianFilter();
    // 设置滑动窗口大小，newSize 必须小于等于 MAX_WINDOW_SIZE
    void setWindowSize(unsigned int newSize);
    // 更新滤波器，返回当前中值滤波结果
    float update(float rawData);

private:
    float window[MAX_WINDOW_SIZE]; // 固定大小的滑动窗口缓冲区
    unsigned int windowSize;       // 实际使用的窗口大小（<= MAX_WINDOW_SIZE）
    unsigned int count;            // 当前存储的数据个数
    unsigned int index;            // 当前写入位置（循环覆盖）
    // 辅助函数：对数组进行插入排序
    void sortArray(float *arr, unsigned int n);
};

// SimpleMeanFilter：均值滤波器，不使用动态内存分配
class SimpleMeanFilter
{
public:
    // 默认构造函数，初始化缓冲区和参数
    SimpleMeanFilter();
    // 设置滑动窗口大小，newSize 必须小于等于 MAX_WINDOW_SIZE
    void setWindowSize(unsigned int newSize);
    // 更新滤波器，返回当前均值滤波结果
    float update(float rawData);

private:
    float window[MAX_WINDOW_SIZE]; // 固定大小的滑动窗口缓冲区
    unsigned int windowSize;       // 实际使用的窗口大小（<= MAX_WINDOW_SIZE）
    unsigned int count;            // 当前存储的数据个数
    unsigned int index;            // 当前写入位置（循环覆盖）
};

#endif
#endif