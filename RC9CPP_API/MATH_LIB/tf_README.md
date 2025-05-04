# 坐标变换库

该坐标变换库提供了简单的二维坐标系之间的转换功能，可用于机器人学、计算机图形学或其他需要坐标变换的领域。

## 文件结构

- `transformation_of_coordinates.h`：包含了坐标变换类的声明。
- `transformation_of_coordinates.cpp`：实现了坐标变换类的方法。

## 功能概述

### 类 `tf`

#### 成员变量

- `float r`：存储从原点到目标坐标的半径距离（单位：米）。
- `float theta`：存储相对于原点的角度偏移（单位：度）。
- `bool inverse`：指示角度是否逆时针增大

#### 方法

- `tf_init(bool inverse_, float r_, float theta_)`：初始化坐标变换参数。
  - `inverse_`：指定角度是否逆时针增大
  - `r_`：从原点到目标坐标的半径距离。
  - `theta_`：相对于原点的角度偏移。
  
- `coordinate_map(Vector2D *original, Vector2D *target, float now_theta)`：执行坐标变换。
  - `original`：指向原始坐标的指针。
  - `target`：指向变换后坐标的指针。
  - `now_theta`：当前角度（单位：度）。

## 使用方法

### 初始化

在使用坐标变换功能前，需先初始化`tf`对象：

```cpp
#include "transformation_of_coordinates.h"
tf transformer;
transformer.tf_init(false, 1.0f, 45.0f); // 初始化角度为顺时针增大，半径1米，初始角度45度
```
