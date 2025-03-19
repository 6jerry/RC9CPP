#include "filters.h"

SimpleLowPassFilter::SimpleLowPassFilter(float alpha)
    : alpha(alpha), initialized(false), lastOutput(0.0f), output(0.0f)
{
}

// 更新函数：输入原始数据，返回滤波后的数据
float SimpleLowPassFilter::update(float rawData)
{
    if (!initialized)
    {
        // 第一次调用时，直接将输出初始化为输入值
        output = rawData;
        lastOutput = rawData;
        initialized = true;
    }
    else
    {
        // 先保存上一次的输出
        lastOutput = output;
        // 一阶低通滤波公式: output = (1-alpha)*lastOutput + alpha*rawData
        output = (1.0f - alpha) * lastOutput + alpha * rawData;
    }
    return output;
}

// 设置滤波系数（可选）
void SimpleLowPassFilter::setAlpha(float newAlpha)
{
    if (newAlpha < 0.0f)
        newAlpha = 0.0f;
    if (newAlpha > 1.0f)
        newAlpha = 1.0f;
    alpha = newAlpha;
}

SimpleMedianFilter::SimpleMedianFilter()
    : windowSize(0), count(0), index(0)
{
    // 初始化缓冲区（可选，保证初始值为0）
    for (unsigned int i = 0; i < MAX_WINDOW_SIZE; i++)
    {
        window[i] = 0.0f;
    }
}

void SimpleMedianFilter::setWindowSize(unsigned int newSize)
{
    if (newSize == 0)
    {
        windowSize = 0;
    }
    else if (newSize > MAX_WINDOW_SIZE)
    {
        windowSize = MAX_WINDOW_SIZE;
    }
    else
    {
        windowSize = newSize;
    }
    // 重置缓冲区索引和计数器
    count = 0;
    index = 0;
}

void SimpleMedianFilter::sortArray(float *arr, unsigned int n)
{
    unsigned int i, j;
    float key;
    for (i = 1; i < n; i++)
    {
        key = arr[i];
        j = i;
        while (j > 0 && arr[j - 1] > key)
        {
            arr[j] = arr[j - 1];
            j--;
        }
        arr[j] = key;
    }
}

float SimpleMedianFilter::update(float rawData)
{
    // 如果窗口未设置，则直接返回原始数据
    if (windowSize == 0)
        return rawData;

    // 循环缓冲存储数据
    window[index] = rawData;
    index = (index + 1) % windowSize;
    if (count < windowSize)
    {
        count++;
    }

    // 将有效数据复制到临时数组中，用于排序计算中值
    float temp[MAX_WINDOW_SIZE];
    for (unsigned int i = 0; i < count; i++)
    {
        temp[i] = window[i];
    }
    sortArray(temp, count);

    // 计算中值
    if (count % 2 == 0)
    {
        return (temp[count / 2 - 1] + temp[count / 2]) / 2.0f;
    }
    else
    {
        return temp[count / 2];
    }
}

// ----------------------- SimpleMeanFilter 实现 -----------------------

SimpleMeanFilter::SimpleMeanFilter()
    : windowSize(0), count(0), index(0)
{
    // 初始化缓冲区（可选，保证初始值为0）
    for (unsigned int i = 0; i < MAX_WINDOW_SIZE; i++)
    {
        window[i] = 0.0f;
    }
}

void SimpleMeanFilter::setWindowSize(unsigned int newSize)
{
    if (newSize == 0)
    {
        windowSize = 0;
    }
    else if (newSize > MAX_WINDOW_SIZE)
    {
        windowSize = MAX_WINDOW_SIZE;
    }
    else
    {
        windowSize = newSize;
    }
    // 重置索引和计数器
    count = 0;
    index = 0;
}

float SimpleMeanFilter::update(float rawData)
{
    // 如果窗口未设置，则直接返回原始数据
    if (windowSize == 0)
        return rawData;

    // 循环缓冲存储数据
    window[index] = rawData;
    index = (index + 1) % windowSize;
    if (count < windowSize)
    {
        count++;
    }

    // 计算均值
    float sum = 0.0f;
    for (unsigned int i = 0; i < count; i++)
    {
        sum += window[i];
    }
    return sum / count;
}