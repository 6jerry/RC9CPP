#pragma once

#include <vector>
#include <Eigen/Dense>
/*****************一阶通频带滤波器*********************/

class First_Order_BPF {
public:
    First_Order_BPF(double fc, double bw, double fs);
    Eigen::VectorXd filter(const Eigen::VectorXd& input);
    std::vector<double> filter(const std::vector<double>& input);

private:
    double fc_, bw_, fs_;  // 中心频率，带宽，采样频率
    double a0_, a1_; // 滤波器系数
    void calculateCoefficients(); //计算截止频率
};

/*****************************************************/



/*************************均值滤波器类******************/
#include <numeric>
class MeanFilter {
public:
    // 构造函数
    MeanFilter(double windowSize);
    // 对输入信号进行滤波处理，支持Eigen::VectorXd和std::vector<double>
    Eigen::VectorXd filter(const Eigen::VectorXd& input);
    std::vector<double> filter(const std::vector<double>& input);

private:
    double windowSize_;  // 窗口大小
};

/*****************************************************/



/***********************中值滤波*************************/
#include <algorithm>
// 中值滤波器类
class MedianFilter {
public:
    // 构造函数
    MedianFilter(int windowSize);
    // 对输入信号进行滤波处理，支持Eigen::VectorXd和std::vector<double>
    Eigen::VectorXd filter(const Eigen::VectorXd& input);
    std::vector<double> filter(const std::vector<double>& input);

private:
    int windowSize_;  // 窗口大小
    std::vector<double> window_;  // 用于存储窗口内元素的容器
    int max(int x, int y); //用于获取最大值
};
/*****************************************************/



/***********************卡尔曼滤波**********************/
// 卡尔曼滤波器类()
class KalmanFilter {
public:
    // 构造函数
    // processNoise: 过程噪声
    // measurementNoise: 测量噪声
    //stateSize: 状态阶数
    KalmanFilter(double processNoise, double measurementNoise, int stateSize);
    // 对Eigen::VectorXd类型的测量值进行滤波处理
    Eigen::VectorXd filter(const Eigen::VectorXd& measurement);
    // 对std::vector<double>类型的测量值进行滤波处理
    std::vector<double> filter(const std::vector<double>& measurement);

private:
    Eigen::VectorXd stateEstimate_;  // 状态估计
    Eigen::MatrixXd errorCovariance_;  // 误差协方差
    Eigen::MatrixXd processNoise_;  // 过程噪声
    Eigen::MatrixXd measurementNoise_;  // 测量噪声
    Eigen::MatrixXd kalmanGain_;  // 卡尔曼增益

    void predict();  // 预测步骤
    void update(const Eigen::VectorXd& measurement);  // 更新步骤
};
/*****************************************************/
