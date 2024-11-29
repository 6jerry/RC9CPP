//
// Created by 32806 on 24-11-29.
//

#include "filter.h"

/***************************一阶带通滤波器*****************************/
// 构造函数，初始化滤波器参数并计算系数
First_Order_BPF::First_Order_BPF(double fc, double bw, double fs) : fc_(fc), bw_(bw), fs_(fs) {
    calculateCoefficients();
}

// 计算滤波器系数
void First_Order_BPF::calculateCoefficients() {
    // 计算归一化截止频率
    float Wn = 2 * PI * (fc_ - bw_ / 2) / fs_;
    float Wd = 2 * PI * (fc_ + bw_ / 2) / fs_;
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

    // 冒泡排序函数
    bool swapped;
    for (int i = 0; i < n - 1; i++) {
            swapped = false;
        for (int j = 0; j < n - i - 1; j++) {
                // 如果当前元素大于下一个元素，交换它们
                if (arr[j] > arr[j + 1]) {
                    swap(&arr[j], &arr[j + 1]);
                    swapped = true;
                }
            }
            // 如果在这一轮遍历中没有发生交换，说明数组已经排序好了
            if (!swapped) {
                break;
            }
        }
}

void MedianFilter::swap(float *x ,float *y){
	float *tmp = x;
	x = y;
	y = tmp;
}


float MedianFilter::input(const float &input) {
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

float MedianFilter::output(const float &output) {
    if(flag == true) {
        return filter(tmp);
    }
    return 0;
}