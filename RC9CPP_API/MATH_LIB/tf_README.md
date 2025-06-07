# 坐标变换库使用指南

## 1. 简介
本库提供二维坐标系统的正向和逆向变换功能，适用于需要在机械中心与安装位置之间进行坐标转换的场景。核心功能包括：
- 将原始坐标转换到旋转中心坐标系
- 将旋转中心坐标系转换回原始安装位置

## 2. 快速开始

### 2.1 依赖
- C++11及以上编译器
- CMSIS DSP库（用于`arm_cos_f32`和`arm_sin_f32`函数）

### 2.2 包含头文件
```cpp
#include "transformation_of_coordinates.h"
```
3. 类方法说明
3.1 tf类构造函数
无需显式初始化，类成员变量默认值为：
offset_x = 0.0f
offset_y = 0.0f
r = 0.0f（极坐标半径）
rad = 0.0f（弧度值）
3.2 正向变换方法
```cpp
void coordinate_map(Vector2D *original, Vector2D *target, float r, float now_rad)
```
功能：将原始坐标变换到旋转中心坐标系
参数：
original：源坐标指针
target：目标坐标指针
r：当前极坐标半径
now_rad：当前弧度值
过程：
计算偏移量offset_x和offset_y
目标坐标 = 源坐标 - 偏移量
3.3 逆变换方法
```cpp
void coordinate_map_inverse(Vector2D *original, Vector2D *target, float r, float now_rad)
```
功能：将旋转中心坐标系变换回原始安装位置
参数：
original：源坐标指针
target：目标坐标指针
r：当前极坐标半径
now_rad：当前弧度值
过程：
计算偏移量offset_x和offset_y
目标坐标 = 源坐标 + 偏移量
4. 使用示例
4.1 正向变换示例
```cpp
#include "transformation_of_coordinates.h"

int main() {
    Vector2D original = {10.0f, 20.0f}; // 原始坐标
    Vector2D target;                   // 目标坐标
    
    tf transformer;
    float radius = 5.0f;
    float angle = PI/4; // 45度
    
    transformer.coordinate_map(&original, &target, radius, angle);
    
    // 输出变换后的坐标
    printf("Transformed coordinates: x=%.2f, y=%.2f\n", target.x, target.y);
    
    return 0;
}
```
4.2 逆变换示例
```cpp
#include "transformation_of_coordinates.h"

int main() {
    Vector2D original = {3.5355f, 3.5355f}; // 假设的变换后坐标
    Vector2D target;                        // 恢复后的坐标
    
    tf transformer;
    float radius = 5.0f;
    float angle = PI/4; // 45度
    
    transformer.coordinate_map_inverse(&original, &target, radius, angle);
    
    // 输出恢复后的坐标
    printf("Restored coordinates: x=%.2f, y=%.2f\n", target.x, target.y);
    
    return 0;
}
```