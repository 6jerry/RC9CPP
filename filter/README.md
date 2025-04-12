# 滤波器库使用文档

## 概述
本项目实现了一个多功能滤波器库，包含以下滤波器类型：
- 一阶带通滤波器（First-Order Bandpass Filter）
- 均值滤波器（Mean Filter）
- 中值滤波器（Median Filter）
- 卡尔曼滤波器（Kalman Filter）

支持数据类型：
✅ `std::vector<double>`  
✅ `Eigen::VectorXd`

## 文件结构
```plaintext
project_root/
├── filter.h            # 滤波器基类定义
├── filter.cpp          # 基本滤波器实现
└── main.cpp            # 使用示例