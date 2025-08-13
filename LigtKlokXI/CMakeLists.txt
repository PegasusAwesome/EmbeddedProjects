cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include_directories($ENV{IDF_PATH}/components/esp_system/include)
add_compile_options("-Wno-format")
project(SquareLine_Project)
