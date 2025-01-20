#include "filter.h"
#include <iostream>


void mysort(float arr[],int n) {
    // 冒泡排序函数
    bool swapped;
    for (int i = 0; i < n - 1; i++) {
        swapped = false;
        for (int j = 0; j < n - i - 1; j++) {
            // 如果当前元素大于下一个元素，交换它们
            if (arr[j] > arr[j + 1]) {
                std::swap(arr[j], arr[j + 1]);
                swapped = true;
            }
        }
        // 如果在这一轮遍历中没有发生交换，说明数组已经排序好了
        if (!swapped) {
            break;
        }
    }
}

int main() {
    // 滤波器参数
    double centerFrequency = 1000; // 中心频率 500 Hz
    double bandwidth = 100;       // 带宽 100 Hz
    double samplingFrequency = 1000; // 采样频率 1000 Hz

    const int windowSize = 5;
    // 创建滤波器实例
    First_Order_BPF filter1(centerFrequency, bandwidth, samplingFrequency);
    MeanFilter filter2(windowSize);  // 窗口大小为5
    MedianFilter filter3(windowSize);  // 窗口大小为5
    //KalmanFilter filter4(3, 5, 1);  // 过程噪声为3，测量噪声为2

    // 创建一个简单的测试信号
    float signal[windowSize] = {1.0, 2.0, 3.5, 4.0, 5.0};
    float eigenSignal[windowSize] = {0};

    // 滤波处理
    float filteredSignal1[windowSize] = {0};
    for(int i = 0; i < windowSize ; i++) {
        filteredSignal1[i] = filter1.filter(signal[i]);
    }

    auto filteredEigenSignal1 = filter1.filter(eigenSignal[0]);

    auto filteredSignal2 = filter2.filter(signal);
    auto filteredEigenSignal2 = filter2.filter(eigenSignal);

    auto filteredSignal3 = filter3.filter(signal);
    auto filteredEigenSignal3 = filter3.filter(eigenSignal);

    //auto filteredEigenSignal4 = filter4.filter(eigenSignal);
    //auto filteredSignal4 = filter4.filter(signal);  // 需要将卡尔曼滤波器适配到std::vector

    // 输出滤波结果
    std::cout << "Filtered Signal (std::vector) - First_Order_BPF: ";
    for (size_t i = 0; i < 5; ++i) {
        std::cout << filteredSignal1[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Filtered Signal (std::vector) - MeanFilter: ";
    for (int i = 0; i < 5; ++i) {  // 假设窗口大小为5
        std::cout << filteredSignal2 << " ";
    }
    std::cout << std::endl;

    std::cout << "Filtered Signal (std::vector) - MedianFilter: ";
    for (int i = 0; i < 5; ++i) {  // 假设窗口大小为5
        std::cout << filteredSignal3 << " ";
    }
    std::cout << std::endl;

    return 0;
    // std::cout << "Filtered Signal (std::vector) - KalmanFilter: ";
    // for (auto& val : filteredSignal4) {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;
    //
    // std::cout << "Filtered Signal (Eigen::VectorXd) - KalmanFilter: " << filteredEigenSignal4.transpose() << std::endl;

}