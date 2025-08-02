#include "BasketCalibrator.h"
#include <cmath>

BasketCalibrator::BasketCalibrator() : measurementCount(0), isCalculated(false) {
    // 构造时调用reset确保初始状态干净
    reset();
}

void BasketCalibrator::reset() {
    measurementCount = 0;
    calibratedCenter = Vector2D(0.0f, 0.0f);
    isCalculated = false;
}

void BasketCalibrator::addMeasurement(float x, float y, float yaw) {
    if (measurementCount < MAX_MEASUREMENTS) {
        measurements[measurementCount] = {x, y, yaw};
        measurementCount++;
        // 添加新数据后，旧的计算结果失效
        isCalculated = false;
    }
}

void BasketCalibrator::addMeasurement(const PointYaw& measurement) {
    if (measurementCount < MAX_MEASUREMENTS) {
        measurements[measurementCount] = measurement;
        measurementCount++;
        isCalculated = false;
    }
}

int BasketCalibrator::getMeasurementCount() const {
    return measurementCount;
}

bool BasketCalibrator::isReadyToCalculate() const {
    return getMeasurementCount() >= MIN_POINTS_REQUIRED;
}

Vector2D BasketCalibrator::getCalibratedCenter() const {
    return calibratedCenter;
}

Vector2D BasketCalibrator::calculateCenter() {
    // 检查是否有足够的点进行计算
    if (!isReadyToCalculate()) {
        return Vector2D(0.0f, 0.0f); // 点不够，返回原点
    }

    // 如果已经计算过且未添加新数据，直接返回缓存的结果
    if (isCalculated) {
        return calibratedCenter;
    }

    // --- 最小二乘法计算 ---
    // 我们要解一个超定线性方程组 M * p = q, 其中 p = [X, Y]^T
    // M^T * M * p = M^T * q
    
    float sum_A_sq = 0.0f;
    float sum_B_sq = 0.0f;
    float sum_AB = 0.0f;
    float sum_AC = 0.0f;
    float sum_BC = 0.0f;

    // 遍历所有测量点，构建方程并累加
    for (int i = 0; i < measurementCount; i++) {
        const PointYaw& measurement = measurements[i];
        // 角度转为弧度
        float yaw_rad = measurement.yaw * 0.01745329f;
        
        // 法向量点法式建立直线方程: A*X + B*Y = C
        // A*x + B*y - C = 0
        // A = cos(yaw), B = -sin(yaw)
        // C = cos(yaw)*robot_x - sin(yaw)*robot_y
        float cos_yaw = cos(yaw_rad);
        float sin_yaw = sin(yaw_rad);

        float A = cos_yaw;
        float B = -sin_yaw;
        float C = A * measurement.x + B * measurement.y;

        // 累加最小二乘法矩阵的元素
        sum_A_sq += A * A;
        sum_B_sq += B * B;
        sum_AB   += A * B;
        sum_AC   += A * C;
        sum_BC   += B * C;
    }

    // 求解 2x2 线性方程组
    // | sum_A_sq  sum_AB | | X | = | sum_AC |
    // | sum_AB    sum_B_sq | | Y |   | sum_BC |
    float det = sum_A_sq * sum_B_sq - sum_AB * sum_AB;

    // 检查行列式，避免除以零（所有线都平行的情况）
    if (abs(det) < 1e-6f) {
        // 矩阵奇异，无法求解。返回一个错误码
        return Vector2D(-50.0f, -50.0f);
    }
    
    // 使用克莱姆法则或求逆矩阵求解
    float inv_det = 1.0f / det;
    float basket_x = inv_det * (sum_B_sq * sum_AC - sum_AB * sum_BC);
    float basket_y = inv_det * (sum_A_sq * sum_BC - sum_AB * sum_AC);

    // 保存并返回结果
    calibratedCenter = Vector2D(basket_x, basket_y);
    isCalculated = true;

    return calibratedCenter;
}