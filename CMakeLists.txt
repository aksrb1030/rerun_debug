cmake_minimum_required(VERSION 3.21)
project(sensor_visualizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(rerun_sdk CONFIG REQUIRED)

add_executable(sensor_visualizer
    src/main.cpp
)

target_link_libraries(sensor_visualizer PRIVATE rerun::rerun)

target_compile_features(sensor_visualizer PRIVATE cxx_std_20)
