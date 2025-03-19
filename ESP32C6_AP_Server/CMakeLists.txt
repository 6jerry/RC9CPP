cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32C6_AP_Server)

# Set partition table
set(PARTITION_TABLE_FILE "${CMAKE_SOURCE_DIR}/partitions.csv")

# Explicitly add component directory
set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/component")
