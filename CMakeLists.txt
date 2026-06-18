cmake_minimum_required(VERSION 3.16)

# ESP-ADF must be set via ADF_PATH environment variable
# Clone ESP-ADF and run: export ADF_PATH=/path/to/esp-adf
include($ENV{ADF_PATH}/CMakeLists.txt)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp-adf-streamer VERSION 0.1.0)
