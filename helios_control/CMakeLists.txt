cmake_minimum_required(VERSION 3.22.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(helios_control)

# find dependencies
find_package(ament_cmake REQUIRED)

ament_package()
