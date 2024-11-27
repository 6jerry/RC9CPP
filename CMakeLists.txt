cmake_minimum_required(VERSION 3.20.0)
project(adjust_test VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 定义源文件，无用，只用来为了cmakelist不报错
add_executable(adjust_test main.cpp)

# 自动查找工作区目录下的所有源文件和头文件
file(GLOB_RECURSE SOURCE_FILES "${CMAKE_SOURCE_DIR}/*.cpp" "${CMAKE_SOURCE_DIR}/*.c")
file(GLOB_RECURSE HEADER_FILES "${CMAKE_SOURCE_DIR}/*.h" "${CMAKE_SOURCE_DIR}/*.hpp")


# 提取头文件所在的目录
foreach(HEADER_FILE ${HEADER_DIRS})
    get_filename_component(HEADER_DIR ${HEADER_FILE} DIRECTORY)
    list(APPEND HEADER_PATHS ${HEADER_DIR})
endforeach()


# 去重并添加到 include_directories
list(REMOVE_DUPLICATES HEADER_PATHS)
include_directories(${HEADER_PATHS})

# 启用测试
include(CTest)
enable_testing()

# 配置 CPACK
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
