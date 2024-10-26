#include "Vector2D.h"
//try12
// 默认构造函数
Vector2D::Vector2D() : x(0.0f), y(0.0f) {}

// 带参数构造函数
Vector2D::Vector2D(float32_t x_, float32_t y_) : x(x_), y(y_) {}

// 重载赋值运算符
Vector2D &Vector2D::operator=(const Vector2D &other)
{
    if (this != &other)
    {
        this->x = other.x;
        this->y = other.y;
    }
    return *this;
}

// 向量加法
Vector2D Vector2D::operator+(const Vector2D &other) const
{
    return Vector2D(this->x + other.x, this->y + other.y);
}

// 向量减法
Vector2D Vector2D::operator-(const Vector2D &other) const
{
    return Vector2D(this->x - other.x, this->y - other.y);
}

// 向量点乘，使用DSP库
float32_t Vector2D::operator*(const Vector2D &other) const
{
    float32_t dotProduct;
    float32_t vecA[2] = {this->x, this->y};
    float32_t vecB[2] = {other.x, other.y};
    arm_dot_prod_f32(vecA, vecB, 2, &dotProduct);
    return dotProduct;
}

// 向量乘以标量
Vector2D Vector2D::operator*(float32_t scalar) const
{
    return Vector2D(this->x * scalar, this->y * scalar);
}

// 标量乘以向量，标量在左侧
Vector2D operator*(float32_t scalar, const Vector2D &vec)
{
    return Vector2D(vec.x * scalar, vec.y * scalar);
}

// 向量的模
float32_t Vector2D::magnitude() const
{
    float32_t result;
    arm_sqrt_f32((this->x * this->x) + (this->y * this->y), &result);
    return result;
}

// 单位化向量
Vector2D Vector2D::normalize() const
{
    Vector2D result;
    float32_t mag = this->magnitude();
    if (mag > 0)
    {
        result.x = this->x / mag;
        result.y = this->y / mag;
    }
    return result;
}

// 向量投影
Vector2D Vector2D::project_onto(const Vector2D &other) const
{
    float32_t dotProduct = (*this) * other;
    float32_t magOtherSquared = other.x * other.x + other.y * other.y;
    float32_t scalar = dotProduct / magOtherSquared;
    return other * scalar;
}
// 负号运算符重载
Vector2D Vector2D::operator-() const
{
    return Vector2D(-this->x, -this->y);
}