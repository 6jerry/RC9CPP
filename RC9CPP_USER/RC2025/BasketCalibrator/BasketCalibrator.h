#ifndef BASKET_CALIBRATOR_H
#define BASKET_CALIBRATOR_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "Vector2D.h"    

// 用于存储单次测量的数据结构
struct PointYaw {
    float x;   // 雷达给出的机器人x坐标
    float y;   // 雷达给出的机器人y坐标
    float yaw; // IMU给出的机器人航向角 (单位：度)
};
#ifdef __cplusplus
}

class BasketCalibrator {
public:
    /**
     * @brief 构造函数
     */
    BasketCalibrator();

    /**
     * @brief 重置标定器，清空所有已记录的测量点。
     */
    void reset();

    /**
     * @brief 添加一次测量数据。
     * @param x 机器人当前的x坐标 (来自雷达)。
     * @param y 机器人当前的y坐标 (来自雷达)。
     * @param yaw 机器人当前的全局航向角 (来自IMU, 单位：度)。
     */
    void addMeasurement(float x, float y, float yaw);
    
    /**
     * @brief 添加一次测量数据 (重载版本)。
     * @param measurement 包含(x, y, yaw)的结构体。
     */
    void addMeasurement(const PointYaw& measurement);

    /**
     * @brief 使用当前所有测量点，通过最小二乘法计算目标中心。
     * @return 计算出的目标中心坐标。如果测量点不足，返回(0, 0)。
     */
    Vector2D calculateCenter();

    /**
     * @brief 检查是否已采集到足够的数据点以进行计算。
     * @return 如果点数满足最小要求，则返回true。
     */
    bool isReadyToCalculate() const;

    /**
     * @brief 获取当前已记录的测量点数量。
     * @return 测量点数量。
     */
    int getMeasurementCount() const;

    /**
     * @brief 获取上一次计算出的中心点坐标，不重新计算。
     * @return 上一次标定的中心点坐标。
     */
    Vector2D getCalibratedCenter() const;


private:
    // 最小需要的测量点数，至少为2，推荐为3或更多以提高精度
    static const int MIN_POINTS_REQUIRED = 2; 
    // 最大存储的测量点数量
    static const int MAX_MEASUREMENTS = 10;
    // 存储所有测量点的静态数组
    PointYaw measurements[MAX_MEASUREMENTS];
    // 当前已存储的测量点数量
    int measurementCount;
    
    // 存储上一次计算结果
    Vector2D calibratedCenter;

    // 标记结果是否已计算
    bool isCalculated;
};
#endif
#endif // BASKET_CALIBRATOR_H