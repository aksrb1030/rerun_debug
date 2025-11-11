cmake_minimum_required(VERSION 3.21)
project(p13_rerun_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(rerun_sdk CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rmw REQUIRED)

add_executable(p13_rerun_bridge
    src/p13_rerun_bridge.cpp
)

target_link_libraries(p13_rerun_bridge
    PRIVATE
        rerun::rerun
        rclcpp::rclcpp
        geometry_msgs::geometry_msgs
        nav_msgs::nav_msgs
        sensor_msgs::sensor_msgs
        std_msgs::std_msgs
        rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
        rcpputils::rcpputils
        rmw::rmw
)

target_compile_features(p13_rerun_bridge PRIVATE cxx_std_20)
