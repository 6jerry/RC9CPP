

# Matrix_Lite: Lightweight Matrix Library for Embedded Systems

## Overview
`Matrix_Lite` is a lightweight matrix library designed for embedded systems, particularly those using ARM Cortex-M processors. It leverages the ARM CMSIS-DSP library (`arm_math.h`) to provide efficient matrix operations. This library supports basic matrix arithmetic, including addition, subtraction, multiplication, and more advanced operations like inversion and transposition.

## File Structure
- **`Matrix_Lite.h`**: Header file containing the class definition and method declarations.
- **`Matrix_Lite.cpp`**: Implementation file containing the method definitions.

## Features

### 1. Matrix Class
The `Matrix` class provides a convenient interface for matrix operations. It supports:
- Dynamic matrix creation with specified dimensions.
- Initialization from one-dimensional arrays or two-dimensional arrays.
- Basic arithmetic operations: addition, subtraction, multiplication.
- Advanced operations: transpose, inverse.
- Element-wise access and modification.

### 2. ARM CMSIS-DSP Integration
The library integrates with the ARM CMSIS-DSP library (`arm_math.h`), ensuring efficient matrix operations on ARM Cortex-M processors.

## Usage

### 1. Matrix Creation
You can create a matrix with specified dimensions and optionally initialize it with data.

```cpp
Matrix mat1(2, 3);  // Create a 2x3 matrix with default values (0)
Matrix mat2(2, 3, data_array);  // Initialize with a one-dimensional array
Matrix mat3(2, 3, data_2d);  // Initialize with a two-dimensional array
```

### 2. Basic Arithmetic Operations
Perform matrix addition, subtraction, and multiplication.

```cpp
Matrix result_add = mat1 + mat2;  // Matrix addition
Matrix result_sub = mat1 - mat2;  // Matrix subtraction
Matrix result_mul = mat1 * mat2;  // Matrix multiplication
```

### 3. Advanced Operations
Transpose and invert matrices.

```cpp
Matrix transposed = mat1.transpose();  // Transpose matrix
Matrix inverted = mat1.inverse();  // Invert matrix
```

### 4. Element Access and Modification
Access and modify individual elements.

```cpp
float32_t element = mat1.getElement(1, 2);  // Get element at row 1, column 2
mat1.setElement(1, 2, 5.0f);  // Set element at row 1, column 2 to 5.0
```

### 5. Matrix Dimensions
Get the dimensions of the matrix.

```cpp
uint16_t rows = mat1.getRows();  // Get number of rows
uint16_t cols = mat1.getCols();  // Get number of columns
```

## Example Code
The following example demonstrates how to use the `Matrix` class to create matrices, perform operations, and print results.

```cpp
#include "Matrix_Lite.h"
#include <iostream>

int main() {
    // Create matrices
    float32_t data1[] = {1, 2, 3, 4, 5, 6};
    Matrix mat1(2, 3, data1);

    float32_t data2[] = {7, 8, 9, 10, 11, 12};
    Matrix mat2(2, 3, data2);

    // Perform operations
    Matrix result_add = mat1 + mat2;
    Matrix result_mul = mat1 * mat2.transpose();

    // Print results
    std::cout << "Matrix 1:\n";
    for (uint16_t i = 0; i < mat1.getRows(); ++i) {
        for (uint16_t j = 0; j < mat1.getCols(); ++j) {
            std::cout << mat1.getElement(i, j) << " ";
        }
        std::cout << "\n";
    }

    std::cout << "\nMatrix 2:\n";
    for (uint16_t i = 0; i < mat2.getRows(); ++i) {
        for (uint16_t j = 0; j < mat2.getCols(); ++j) {
            std::cout << mat2.getElement(i, j) << " ";
        }
        std::cout << "\n";
    }

    std::cout << "\nResult (Addition):\n";
    for (uint16_t i = 0; i < result_add.getRows(); ++i) {
        for (uint16_t j = 0; j < result_add.getCols(); ++j) {
            std::cout << result_add.getElement(i, j) << " ";
        }
        std::cout << "\n";
    }

    std::cout << "\nResult (Multiplication):\n";
    for (uint16_t i = 0; i < result_mul.getRows(); ++i) {
        for (uint16_t j = 0; j < result_mul.getCols(); ++j) {
            std::cout << result_mul.getElement(i, j) << " ";
        }
        std::cout << "\n";
    }

    return 0;
}
```

## Dependencies
- **ARM CMSIS-DSP Library**: Ensure that the ARM CMSIS-DSP library is included in your project.
- **C++11 or later**: The library uses C++11 features, so ensure your compiler supports it.

## Installation
1. Include `Matrix_Lite.h` and `Matrix_Lite.cpp` in your project.
2. Link against the ARM CMSIS-DSP library.
3. Compile and run your application.

## License
This library is released under the MIT License. See the LICENSE file for details.

# Conculsion
`Matrix_Lite` provides a simple and efficient way to handle matrix operations in embedded systems, leveraging the power of the ARM CMSIS-DSP library.