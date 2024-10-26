#ifndef VECTOR2D_H
#define VECTOR2D_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <arm_math.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Vector2D
{
public:
    float32_t x;
    float32_t y;

    // 构造函数
    Vector2D();
    Vector2D(float32_t x_, float32_t y_);

    // 赋值运算符
    Vector2D &operator=(const Vector2D &other);

    // 向量加法
    Vector2D operator+(const Vector2D &other) const;

    // 向量减法
    Vector2D operator-(const Vector2D &other) const;
    Vector2D operator-() const;

    // 向量点乘
    float32_t operator*(const Vector2D &other) const;

    // 向量乘以标量
    Vector2D operator*(float32_t scalar) const;

    // 标量乘以向量 (标量在左侧)
    friend Vector2D operator*(float32_t scalar, const Vector2D &vec);

    // 向量的模（长度）
    float32_t magnitude() const;

    // 单位化向量
    Vector2D normalize() const;

    // 向量投影：投影this向量到other向量上
    Vector2D project_onto(const Vector2D &other) const;
};

#endif // VECTOR2D_H
#endif
