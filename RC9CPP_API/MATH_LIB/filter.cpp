//
// Created by 32806 on 24-11-29.
//

#include "filter.h"



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

/***************************一阶带通滤波器*****************************/
// 构造函数，初始化滤波器参数并计算系数
First_Order_BPF::First_Order_BPF(double fc, double bw, double fs) : fc_(fc), bw_(bw), fs_(fs) {
    calculateCoefficients();
}

void First_Order_BPF::calculateCoefficients() {
    // 计算归一化截止频率
    float Wn = 2 * M_PI * (fc_ - bw_ / 2) / fs_;
    float Wd = 2 * M_PI * (fc_ + bw_ / 2) / fs_;
    float T = 1 / fs_;

    // 计算一阶通频带滤波器的系数 a0_为低阶，a1_为高阶
    a0_ = Wn/(Wn+1);
    a1_ = 1/(1+Wd);
}

float First_Order_BPF::filter(const float& input) {
    // 静态变量用于存储上一次的输入和输出值
    static float lastInput = 0;
    static float lastOutput = 0;

    // 计算当前输出值
    float output1 = a0_ * input + (1 - a0_) * lastInput;
    float output2 = a1_*(input - lastInput) + a1_ * lastOutput;

    // 计算反馈项
    float feedback = - output1 + output2;


    lastOutput = feedback; // 更新上一次的输出值
    lastInput = input;  // 更新上一次的输入值
    return feedback;  // 返回滤波后的输出值
}
/********************************************************************/






/*****************************均值滤波器******************************/
// 构造函数，设置窗口大小
MeanFilter::MeanFilter(double windowSize) : windowSize_(static_cast<uint8_t>(windowSize)) {}

// 滤波函数，对固定大小的输入数组进行滤波处理
float MeanFilter::filter(const float input[filter_max_size]) {
    float sum = 0.0f;  // 用于累加窗口内的所有值
    for (uint8_t i = 0; i < windowSize_; ++i) {
        sum += input[i];  // 累加窗口内的所有值
    }
    return sum / windowSize_;  // 返回窗口内值的平均值
}


float MeanFilter::input(float &input) {
    //用于设置开始滑动状态位与填充新数据
    if(index < 4) {
        tmp[index++] = input;
    }
    else {
        if(flag == false) {
            flag = true;
        }
        index = 0;
    }
}

float MeanFilter::output() {
    if(flag == true) {
        return filter(tmp);
    }
    return 0;
}
/********************************************************************/





/**************************中值滤波器**********************************/
// 构造函数，设置窗口大小
MedianFilter::MedianFilter(int windowSize) : windowSize_(windowSize) {}

float MedianFilter::filter(float input[filter_max_size]) {
    for (int i = 0; i < windowSize_; ++i) {
        window_[i] = input[i];  // 将输入数组的值复制到窗口数组中
    }
    sort(window_,windowSize_);  // 对窗口数组进行排序
    return window_[windowSize_ / 2];  // 返回窗口数组的中值
}

void MedianFilter::sort(float arr[], int n) {
    if (n <= 1) return; // 如果数组长度小于等于1，直接返回

    // 选择基准元素（这里选择最后一个元素）
    float pivot = arr[n - 1];
    int i = 0; // i 是小于基准元素的边界

    // 分区操作：将小于基准的元素放到左边，大于基准的元素放到右边
    for (int j = 0; j < n - 1; j++) {
        if (arr[j] < pivot) {
            // 手动交换 arr[i] 和 arr[j]
            float temp = arr[i];
            arr[i] = arr[j];
            arr[j] = temp;
            i++;
        }
    }

    // 将基准元素放到正确的位置
    float temp = arr[i];
    arr[i] = arr[n - 1];
    arr[n - 1] = temp;

    // 递归排序左半部分和右半部分
    sort(arr, i);            // 排序左半部分
    sort(arr + i + 1, n - i - 1); // 排序右半部分
}


float MedianFilter::input(float &input) {
    //用于设置开始滑动状态位与填充新数据
    if(index < 4) {
        tmp[index++] = input;
    }
    else {
        if(flag == false) {
            flag = true;
        }
        index = 0;
    }
}

float MedianFilter::output() {
    if(flag == true) {
        return filter(tmp);
    }
    return 0;
}

/********************************************************************/





/**************************卡尔曼滤波器********************************/
// 卡尔曼滤波器构造函数
/*
 * Q：表示系统模型的不确定性,通常设置为一个较小的值（如 0.01），表示系统模型的不确定性较低。
 * R: 表示传感器测量的不确定性,通常设置为一个较大的值（如 1.0），表示传感器测量的不确定性较高。
 * P：表示状态估计的不确定性，初始值可以设置为一个较大的值，后续会动态更新。初始时可以设置为 1.0，表示初始估计的不确定性较高。
 */
void KalmanFilter::KalmanFilter_init(float Q, float R, float P) {
    kf->Q = Q;
    kf->R = R;
    kf->P = P;
    kf->x = 0;
    kf->K = 0;
}

// 更新卡尔曼滤波器状态,传入需要滤波的数据
float KalmanFilter::filter(const float &measurement) {
    // 预测步骤
    kf->P = kf->P + kf->Q;

    // 更新步骤
    kf->K = kf->P / (kf->P + kf->R);  // 计算卡尔曼增益
    kf->x = kf->x + kf->K * (measurement - kf->x);  // 更新状态估计值
    kf->P = (1 - kf->K) * kf->P;  // 更新估计误差协方差

    return kf->x;  // 返回滤波后的状态值
}
/********************************************************************/
