cmake_minimum_required(VERSION 3.28)
project(filter)

set(CMAKE_CXX_STANDARD 17)

set(Eigen3_DIR "${CMAKE_SOURCE_DIR}") # 替换为 Eigen 的实际路径
#find_package(Eigen3 REQUIRED)


# 包含 Eigen 的头文件目录
include_directories(${Eigen3_DIR})

# 添加库
add_library(filter STATIC filter.cpp
        filter.h)

# 添加可执行文件
add_executable(my_test main.cpp)

# 链接库到可执行文件
target_link_libraries(my_test PRIVATE filter)