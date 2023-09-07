cmake_minimum_required(VERSION 3.8)
project(rm_bring_up)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bring_up REQUIRED)
find_package(rm_interfaces REQUIRED)

ament_package()
