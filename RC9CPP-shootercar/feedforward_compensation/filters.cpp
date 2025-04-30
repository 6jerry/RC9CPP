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