#ifndef MATRIX_LITE_H
#define  MATRIX_LITE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <arm_math.h>
#include <cstring>
#include <cstdint>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Matrix {
public:
    // 构造函数
    Matrix(uint16_t rows, uint16_t cols);
    Matrix(uint16_t rows, uint16_t cols, const float32_t* data);
    Matrix(uint16_t rows, uint16_t cols, const float32_t** data);

    ~Matrix();

    // 拷贝构造函数
    Matrix(const Matrix& other);

    // 赋值运算符
    Matrix& operator=(const Matrix& other);

    // 矩阵加法
    Matrix operator+(const Matrix& other) const;

    // 矩阵减法
    Matrix operator-(const Matrix& other) const;

    // 矩阵乘法
    Matrix operator*(const Matrix& other) const;

    // 矩阵乘以标量
    Matrix operator*(float32_t scalar) const;

    // 标量乘以矩阵 (标量在左侧)
    friend Matrix operator*(float32_t scalar, const Matrix& mat);

    // 矩阵转置
    Matrix transpose() const;

    // 矩阵求逆
    Matrix inverse() const;

    // 获取矩阵的行数
    uint16_t getRows() const;

    // 获取矩阵的列数
    uint16_t getCols() const;

    // 获取矩阵元素
    float32_t getElement(uint16_t row, uint16_t col) const;

    // 设置矩阵元素
    void setElement(uint16_t row, uint16_t col, float32_t value);


private:
    uint16_t rows;
    uint16_t cols;
    float32_t* data;
    arm_matrix_instance_f32 mat; // arm_math 库的矩阵结构体

    // 初始化矩阵
    void initialize(uint16_t rows, uint16_t cols, const float32_t* data = nullptr);
};

#endif // MATRIX_H
#endif