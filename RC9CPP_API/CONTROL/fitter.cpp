#include "fitter.h"

PolynomialFitter::PolynomialFitter(float under ,float upper ,bool zero_point) {
    min = under;
    max = upper;
    zeroPoint = zero_point;

    // 初始化系数为零
    resetCoefficients();
}

// 辅助函数，用于查找零点位置
int findZeroPoint(float r[], float d[], int n) {
    for (int i = 0; i < n; ++i) {
        if (fabs(r[i]) <= 1e-6f && fabs(d[i]) <= 1e-6f) {
            return i;
        }
    }
    return n; // 未找到零点，返回数组长度
}

void PolynomialFitter::resetCoefficients() {
    for (int i = 0; i <= 1; i++) linearCoeffs[i] = 0.0f;
    for (int i = 0; i <= 2; i++) quadraticCoeffs[i] = 0.0f;
    for (int i = 0; i <= 3; i++) cubicCoeffs[i] = 0.0f;
}

void PolynomialFitter::resetCoefficientsLinear() {
    for (int i = 0; i <= 1; i++) linearCoeffs[i] = 0.0f;
}

void PolynomialFitter::resetCoefficientsQuadratic() {
    for (int i = 0; i <= 2; i++) quadraticCoeffs[i] = 0.0f;
}

void PolynomialFitter::resetCoefficientsCubic() {
    for (int i = 0; i <= 3; i++) cubicCoeffs[i] = 0.0f; 
}

void PolynomialFitter::fitLinear(float r[], float d[], int n) {
		static int last_N = 0;
    int newN = n;
		static float last_r,last_d;
    if (zeroPoint) {
        newN = findZeroPoint(r, d, n);
    }
		if(last_N != newN){
			last_N = newN;
			last_r = r[newN-1];
			last_d = d[newN-1];
			resetCoefficientsLinear();
			polynomialFit(r, d, newN, 1, linearCoeffs);
		}
		else if(last_d != d[newN-1] || last_r != r[newN-1])
		{
			last_r = r[newN-1];
			last_d = d[newN-1];
			resetCoefficientsLinear();
			polynomialFit(r, d, newN, 1, linearCoeffs);
		}
}

void PolynomialFitter::fitQuadratic(float r[], float d[], int n) {
		static int last_N = 0;
    int newN = n;
		static float last_r,last_d;
    if (zeroPoint) {
        newN = findZeroPoint(r, d, n);
    }
		if(last_N != newN){
			last_N = newN;
			last_r = r[newN-1];
			last_d = d[newN-1];
			resetCoefficientsQuadratic();
			polynomialFit(r, d, newN, 2, quadraticCoeffs);
		}
		else if(last_d != d[newN-1] || last_r != r[newN-1])
		{
			last_r = r[newN-1];
			last_d = d[newN-1];
			resetCoefficientsQuadratic();
			polynomialFit(r, d, newN, 2, quadraticCoeffs);
		}
}

void PolynomialFitter::fitCubic(float r[], float d[], int n) {
		static int last_N = 0;
		static float last_r,last_d;
    int newN = n;
    if (zeroPoint) {
        newN = findZeroPoint(r, d, n);
    }
		
		if(last_N != newN){
			last_N = newN;
			last_r = r[newN-1];
			last_d = d[newN-1];
			resetCoefficientsCubic();
			polynomialFit(r, d, newN, 3, cubicCoeffs);
		}
		else if(last_d != d[newN-1] || last_r != r[newN-1])
		{
			last_r = r[newN-1];
			last_d = d[newN-1];
			resetCoefficientsCubic();
			polynomialFit(r, d, newN, 3, cubicCoeffs);
		}
}

void PolynomialFitter::fitAll(float r[], float d[], int n) {
    fitLinear(r, d, n);
    fitQuadratic(r, d, n);
    fitCubic(r, d, n);
}

float PolynomialFitter::evalLinear(float x) const {
    float result = linearCoeffs[0] + linearCoeffs[1] * x;
    if (result < min) {
        // 如果包含零点且x小于最小值，则返回最小值对应的线性值
        return min;
    } else if (result > max) {
        // 如果包含零点且x大于最大值，则返回最大值对应的线性值
        return max;
    }
    return result;
}

float PolynomialFitter::evalQuadratic(float x) const {
    float result = quadraticCoeffs[0] 
           + quadraticCoeffs[1] * x 
           + quadraticCoeffs[2] * x * x;
    if (result < min) {
        // 如果包含零点且x小于最小值，则返回最小值对应的二次值
        return min;
    } else if (result > max) {
        // 如果包含零点且x大于最大值，则返回最大值对应的二次值
        return max;
    }
    return result;
}

float PolynomialFitter::evalCubic(float x) const {
    // 使用霍纳法则减少乘法次数
    float result = cubicCoeffs[0] 
           + x * (cubicCoeffs[1] 
           + x * (cubicCoeffs[2] 
           + x * cubicCoeffs[3]));
    if (result < min) {
        // 如果包含零点且x小于最小值，则返回最小值对应的三次值
        return min;
    } else if (result > max) {
        // 如果包含零点且x大于最大值，则返回最大值对应的三次值
        return max;
    }
    return result;
}

const float* PolynomialFitter::getLinearCoeffs() const { 
    return linearCoeffs; 
}

const float* PolynomialFitter::getQuadraticCoeffs() const { 
    return quadraticCoeffs; 
}

const float* PolynomialFitter::getCubicCoeffs() const { 
    return cubicCoeffs; 
}

int PolynomialFitter::solveLinearSystem(int n, float A[][MAX_DEGREE + 1], float b[]) const {
    // 增广矩阵 [A | b]
    float Ab[MAX_DEGREE + 1][MAX_DEGREE + 2];
    
    // 初始化增广矩阵
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Ab[i][j] = A[i][j];
        }
        Ab[i][n] = b[i];
    }

    // 前向消元
    for (int pivot = 0; pivot < n; pivot++) {
        // 部分主元法提高稳定性
        float maxVal = 0;
        int maxRow = pivot;
        for (int row = pivot; row < n; row++) {
            if (fabsf(Ab[row][pivot]) > maxVal) {
                maxVal = fabsf(Ab[row][pivot]);
                maxRow = row;
            }
        }
        if (maxVal < 1e-10f) return -1; // 奇异矩阵

        // 行交换
        if (maxRow != pivot) {
            for (int col = pivot; col <= n; col++) {
                float temp = Ab[pivot][col];
                Ab[pivot][col] = Ab[maxRow][col];
                Ab[maxRow][col] = temp;
            }
        }

        // 消元
        for (int row = pivot + 1; row < n; row++) {
            float factor = Ab[row][pivot] / Ab[pivot][pivot];
            for (int col = pivot; col <= n; col++) {
                Ab[row][col] -= factor * Ab[pivot][col];
            }
        }
    }

    // 回代求解
    for (int i = n - 1; i >= 0; i--) {
        b[i] = Ab[i][n];
        for (int j = i + 1; j < n; j++) {
            b[i] -= Ab[i][j] * b[j];
        }
        b[i] /= Ab[i][i];
    }
    return 0;
}

void PolynomialFitter::polynomialFit(float r[], float d[], int n, int degree, float coeffs[]) {
    // 检查数据量是否足够
    if (n <= degree) return;
    
    // 预计算幂次和以减少重复计算
    float xPowers[2 * MAX_DEGREE + 1] = {0};
    for (int exp = 0; exp <= 2 * degree; exp++) {
        for (int i = 0; i < n; i++) {
            xPowers[exp] += powFast(r[i], exp);
        }
    }
    
    // 初始化正规方程矩阵
    float A[MAX_DEGREE + 1][MAX_DEGREE + 1] = {{0}};
    float B[MAX_DEGREE + 1] = {0};
    
    // 构建正规方程: A^T*A * coeffs = A^T*d
    for (int k = 0; k <= degree; k++) {       // 方程行
        for (int m = 0; m <= degree; m++) {   // 方程列
            A[k][m] = xPowers[k + m];
        }
        
        // 计算右侧向量
        for (int i = 0; i < n; i++) {
            B[k] += d[i] * powFast(r[i], k);
        }
    }
    
    // 解线性方程组
    solveLinearSystem(degree + 1, A, B);
    
    // 存储系数
    for (int i = 0; i <= degree; i++) {
        coeffs[i] = B[i];
    }
}

float PolynomialFitter::powFast(float base, int exponent) const {
    // 处理特殊情况
    if (exponent == 0) return 1.0f;
    if (base == 0.0f) return 0.0f;
    
    // 使用迭代计算幂
    float result = 1.0f;
    while (exponent > 0) {
        if (exponent & 1) {
            result *= base;
        }
        base *= base;
        exponent >>= 1;
    }
    return result;
}