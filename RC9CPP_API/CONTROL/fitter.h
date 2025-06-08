#ifndef POLYNOMIAL_FITTER_H
#define POLYNOMIAL_FITTER_H



// 最大多项式次数 (三次)
#define MAX_DEGREE 3
#ifdef __cplusplus
extern "C"
{
#endif
#include <math.h>	
	
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class PolynomialFitter {
public:
    float min, max; // 数据范围
    bool zeroPoint; // 是否包含零点

    PolynomialFitter(float under ,float upper ,bool zero_point);
    
    // 重置所有系数为零
    void resetCoefficients();
    void resetCoefficientsLinear();
    void resetCoefficientsQuadratic();
    void resetCoefficientsCubic();
    
    // 拟合函数接口
    void fitAll(float r[], float d[], int n);
    void fitLinear(float r[], float d[], int n);
    void fitQuadratic(float r[], float d[], int n);
    void fitCubic(float r[], float d[], int n);
    
    // 函数调用接口
    float evalLinear(float x) const;
    float evalQuadratic(float x) const;
    float evalCubic(float x) const;
    
    // 获取系数接口
    const float* getLinearCoeffs() const;
    const float* getQuadraticCoeffs() const;
    const float* getCubicCoeffs() const;

    // 存储拟合系数
    float linearCoeffs[2];      // 线性: a0 + a1*x
    float quadraticCoeffs[3];   // 二次: a0 + a1*x + a2*x^2
    float cubicCoeffs[4];       // 三次: a0 + a1*x + a2*x^2 + a3*x^3

    // 高斯消元法解线性方程组
    int solveLinearSystem(int n, float A[][MAX_DEGREE + 1], float b[]) const;
    
    // 多项式拟合核心函数
    void polynomialFit(float r[], float d[], int n, int degree, float coeffs[]);
    
    // 快速幂计算
    float powFast(float base, int exponent) const;
};

#endif // POLYNOMIAL_FITTER_H

#endif 
