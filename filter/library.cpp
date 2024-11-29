#include "library.h"

/************************一阶通频带滤波器*************************/
First_Order_BPF::First_Order_BPF(double fc, double bw, double fs)
    : fc_(fc), bw_(bw), fs_(fs) {
    calculateCoefficients();
}

void First_Order_BPF::calculateCoefficients() {
    // 计算归一化截止频率
    double Wn = 2 * M_PI * (fc_ - bw_ / 2) / fs_;
    double Wd = 2 * M_PI * (fc_ + bw_ / 2) / fs_;

    // 计算一阶通频带滤波器的系数
    a0_ = 1 / (1 + std::sin(Wd) / std::sin(Wn) * std::tan(M_PI / 4));
    a1_ = -a0_;
}

Eigen::VectorXd First_Order_BPF::filter(const Eigen::VectorXd& input) {
    Eigen::VectorXd output(input.size()); // 初始化输出向量
    for (size_t i = 1; i < input.size(); ++i) {
        // 一阶滤波器的差分方程
        output(i) = (input(i) - a1_ * input(i - 1)) / a0_;
    }
    return output;
}

std::vector<double> First_Order_BPF::filter(const std::vector<double>& input) {
    std::vector<double> output(input.size()); // 初始化输出向量
    for (size_t i = 1; i < input.size(); ++i) {
        // 一阶滤波器的差分方程
        output[i] = (input[i] - a1_ * input[i - 1]) / a0_;
    }
    return output;
}
/**************************************************************/




/****************************均值滤波器类**************************/
//设置窗口大小
MeanFilter::MeanFilter(double windowSize) : windowSize_(windowSize) {}

Eigen::VectorXd MeanFilter::filter(const Eigen::VectorXd& input) {
    Eigen::VectorXd output(input.size());
    for (int i = 0; i < input.size(); ++i) {
        double sum = 0.0;
        int count = 0;
        // 计算窗口内元素的均值
        for (int j = std::max(0, i - static_cast<int>(windowSize_)); j <= i; ++j) {
            sum += input(j);
            count++;
        }
        output(i) = sum / count;
    }
    return output;
}

std::vector<double> MeanFilter::filter(const std::vector<double>& input) {
    std::vector<double> output(input.size());
    for (size_t i = 0; i < input.size(); ++i) {
        double sum = 0.0;
        size_t count = 0;
        // 计算窗口内元素的均值
        for (size_t j = std::max<size_t>(0, i - static_cast<size_t>(windowSize_)); j <= i; ++j) {
            sum += input[j];
            count++;
        }
        output[i] = sum / count;
    }
    return output;
}
/**************************************************************/




/***************************中值滤波器**************************/
MedianFilter::MedianFilter(int windowSize) : windowSize_(windowSize) {}

Eigen::VectorXd MedianFilter::filter(const Eigen::VectorXd& input) {
    Eigen::VectorXd output(input.size());
    for (int i = 0; i < input.size(); ++i) {
        window_.clear();
        // 填充窗口
        for (int j = std::max(0, i - windowSize_ / 2); j <= i + windowSize_ / 2; ++j) {
            if (j >= 0 && j < input.size()) {
                window_.push_back(input(j));
            }
        }
        // 计算中值
        std::sort(window_.begin(), window_.end());
        output(i) = window_.size() % 2 == 0 ? (window_[window_.size() / 2 - 1] + window_[window_.size() / 2]) / 2.0
                                                             : window_[window_.size() / 2];
    }
    return output;
}

std::vector<double> MedianFilter::filter(const std::vector<double>& input) {
    std::vector<double> output(input.size());
    for (size_t i = 0; i < input.size(); ++i) {
        window_.clear();
        // 填充窗口
        //此处max为类内自定义函数
        for (int j = max(0, i - windowSize_ / 2); j <= i + windowSize_ / 2; ++j) {
            if (j >= 0 && j < static_cast<int>(input.size())) {
                window_.push_back(input[j]);
            }
        }
        // 计算中值
        std::sort(window_.begin(), window_.end());
        output[i] = window_.size() % 2 == 0 ? (window_[window_.size() / 2 - 1] + window_[window_.size() / 2]) / 2.0
                                                             : window_[window_.size() / 2];
    }
    return output;
}

int MedianFilter::max(const int x, const int y) {
    return x > y ? x : y;
}
/**************************************************************/




/*************************卡尔曼滤波器************************/
// 构造函数实现，初始化状态估计、误差协方差、过程噪声和测量噪声,状态阶数
KalmanFilter::KalmanFilter(double processNoise, double measurementNoise, int stateSize)
    : processNoise_(Eigen::MatrixXd::Identity(stateSize, stateSize) * processNoise),
      measurementNoise_(Eigen::MatrixXd::Identity(stateSize, stateSize) * measurementNoise),
      stateEstimate_(Eigen::VectorXd::Zero(stateSize)),
      errorCovariance_(Eigen::MatrixXd::Identity(stateSize, stateSize)) {}

// 对Eigen::VectorXd类型的测量值进行滤波处理
Eigen::VectorXd KalmanFilter::filter(const Eigen::VectorXd& measurement) {
    predict();  // 执行预测步骤
    update(measurement);  // 执行更新步骤
    return stateEstimate_;  // 返回状态估计
}

// 对std::vector<double>类型的测量值进行滤波处理
std::vector<double> KalmanFilter::filter(const std::vector<double>& measurement) {
    Eigen::VectorXd eigenMeasurement(measurement.size());
    for (size_t i = 0; i < measurement.size(); ++i) {
        eigenMeasurement(i) = measurement[i];
    }

    Eigen::VectorXd filteredEigenSignal = filter(eigenMeasurement);
    std::vector<double> filteredSignal(filteredEigenSignal.size());
    for (size_t i = 0; i < filteredEigenSignal.size(); ++i) {
        filteredSignal[i] = filteredEigenSignal(i);
    }
    return filteredSignal;
}

// 预测下一状态和误差协方差
void KalmanFilter::predict() {
    stateEstimate_ += processNoise_;  // 状态预测
    errorCovariance_ += processNoise_;  // 误差协方差预测
}

// 根据新的测量值更新状态估计和误差协方差
void KalmanFilter::update(const Eigen::VectorXd& measurement) {
    // 将 Eigen::VectorXd 转换成 Eigen::MatrixXd
    Eigen::MatrixXd measurementMatrix = measurement.transpose();
    // 确保误差协方差矩阵和测量噪声矩阵是方阵且大小相同
    assert(errorCovariance_.rows() == errorCovariance_.cols() && measurementNoise_.rows() == measurementNoise_.cols());
    assert(errorCovariance_.rows() == measurementNoise_.rows());

    Eigen::MatrixXd totalNoise = errorCovariance_ + measurementNoise_;

    // 确保总噪声矩阵是可逆的
    assert(totalNoise.determinant() != 0);

    kalmanGain_ = errorCovariance_ * totalNoise.inverse();  // 计算卡尔曼增益
    stateEstimate_ += kalmanGain_ * (measurement - stateEstimate_);  // 更新状态估计
    errorCovariance_ -= kalmanGain_ * measurementNoise_ * kalmanGain_.transpose();  // 更新误差协方差
}

/**************************************************************/