cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32C6_AP_Server)

# 显式添加component目录
set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/component")
