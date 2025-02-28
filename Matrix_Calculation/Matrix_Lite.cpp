
/*
 * @author loopgad
 * @contact 3280646246@qq.com
 * @license MIT License
 *
 * Copyright (c) 2024 loopgad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Matrix_Lite.h"

// 构造函数（默认初始化）
Matrix::Matrix(uint16_t rows, uint16_t cols) : rows(rows), cols(cols) {
    initialize(rows, cols);
}

// 构造函数（用一维数组初始化）
Matrix::Matrix(uint16_t rows, uint16_t cols, const float32_t* data) : rows(rows), cols(cols) {
    initialize(rows, cols, data);
}

// 构造函数（用二级指针初始化）
Matrix::Matrix(uint16_t rows, uint16_t cols, const float32_t** data) : rows(rows), cols(cols) {
    initialize(rows, cols);
    for (uint16_t i = 0; i < rows; ++i) {
        for (uint16_t j = 0; j < cols; ++j) {
            this->data[i * cols + j] = data[i][j];
        }
    }
    arm_mat_init_f32(&mat, rows, cols, this->data); // 初始化 arm_matrix_instance_f32
}

Matrix::~Matrix() {
    delete[] data;
}

// 拷贝构造函数
Matrix::Matrix(const Matrix& other) : rows(other.rows), cols(other.cols) {
    initialize(rows, cols);
    memcpy(data, other.data, rows * cols * sizeof(float32_t));
    arm_mat_init_f32(&mat, rows, cols, data); // 初始化 arm_matrix_instance_f32
}

// 赋值运算符
Matrix& Matrix::operator=(const Matrix& other) {
    if (this != &other) {
        delete[] data;
        rows = other.rows;
        cols = other.cols;
        initialize(rows, cols);
        memcpy(data, other.data, rows * cols * sizeof(float32_t));
        arm_mat_init_f32(&mat, rows, cols, data); // 初始化 arm_matrix_instance_f32
    }
    return *this;
}

// 矩阵加法
Matrix Matrix::operator+(const Matrix& other) const {
    Matrix result(rows, cols);
    arm_mat_add_f32(&this->mat, &other.mat, &result.mat);
    return result;
}

// 矩阵减法
Matrix Matrix::operator-(const Matrix& other) const {
    Matrix result(rows, cols);
    arm_mat_sub_f32(&this->mat, &other.mat, &result.mat);
    return result;
}

// 矩阵乘法
Matrix Matrix::operator*(const Matrix& other) const {
    Matrix result(rows, other.cols);
    arm_mat_mult_f32(&this->mat, &other.mat, &result.mat);
    return result;
}

// 矩阵乘以标量
Matrix Matrix::operator*(float32_t scalar) const {
    Matrix result(rows, cols);
    arm_mat_scale_f32(&this->mat, scalar, &result.mat);
    return result;
}

// 标量乘以矩阵 (标量在左侧)
Matrix operator*(float32_t scalar, const Matrix& mat) {
    return mat * scalar;
}

// 矩阵转置
Matrix Matrix::transpose() const {
    Matrix result(cols, rows);
    arm_mat_trans_f32(&this->mat, &result.mat);
    return result;
}

// 矩阵求逆
Matrix Matrix::inverse() const {
    Matrix result(rows, cols);
    arm_mat_inverse_f32(&this->mat, &result.mat);
    return result;
}

// 获取矩阵的行数
uint16_t Matrix::getRows() const {
    return rows;
}

// 获取矩阵的列数
uint16_t Matrix::getCols() const {
    return cols;
}

// 获取矩阵元素
float32_t Matrix::getElement(uint16_t row, uint16_t col) const {
    return data[row * cols + col];
}

// 设置矩阵元素
void Matrix::setElement(uint16_t row, uint16_t col, float32_t value) {
    data[row * cols + col] = value;
}


// 初始化矩阵
void Matrix::initialize(uint16_t rows, uint16_t cols, const float32_t* data) {
    this->rows = rows;
    this->cols = cols;
    this->data = new float32_t[rows * cols];
    if (data) {
        memcpy(this->data, data, rows * cols * sizeof(float32_t));
    } else {
        memset(this->data, 0, rows * cols * sizeof(float32_t));
    }
    arm_mat_init_f32(&this->mat, rows, cols, this->data); // 初始化 arm_matrix_instance_f32
}