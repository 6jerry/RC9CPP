#pragma once
#include <cstdint>
#include <cmath>
//定义最大窗口
#define filter_max_size 30
#define PI 3.141592653589793



/**
 * @file
 * @brief Description of the file's purpose and contents.
 *
 * 一阶带通采用直接输入和输出
 * 均值滤波和中值滤波需要先获得窗口数的数据后再进行滑动输出
 *
 * @author loopgad
 * @date 11/29/2024
 * @version 1.0
 * @copyright Copyright (c) Year Company. All rights reserved.
 */


/*****************一阶通频带滤波器*********************/

class First_Order_BPF {
public:
    First_Order_BPF(double fc, double bw, double fs);
    float filter(const float& input);

private:
    float fc_, bw_, fs_;  // 中心频率，带宽，采样频率
    float a0_, a1_; // 滤波器系数
    void calculateCoefficients(); //计算截止频率
};

/*****************************************************/



/*************************均值滤波器类******************/

class MeanFilter {
public:
    // 构造函数
    MeanFilter(double windowSize);
    // 对输入信号进行滤波处理
    float filter(const float input[filter_max_size]);
    //滤波器输入输出接口
    float input(const float &input);
    float output(const float &output);
private:
    uint8_t windowSize_ = 5;  // 窗口大小
    uint8_t index = 0; //索引值
    bool flag = false; //滑动状态位
    float tmp[filter_max_size]; //缓存数组
};

/*****************************************************/



/***********************中值滤波*************************/

// 中值滤波器类
class MedianFilter {
public:
    // 构造函数，设置窗口大小
    MedianFilter(int windowSize);
    // 对固定大小的输入数组进行滤波处理
    float filter(float input[filter_max_size]);
    // 对输入数组进行排序
    void sort(float input[], int n);
    //滤波器输入输出接口
    float input(const float &input);
    float output(const float &output);

private:
    uint8_t windowSize_ = 5;  // 窗口大小
    float window_[filter_max_size] = {0};  // 用于存储窗口内元素的容器
    uint8_t index = 0; //索引值
    bool flag = false; //滑动状态位
    float tmp[filter_max_size]; //缓存数组
};
/*****************************************************/


