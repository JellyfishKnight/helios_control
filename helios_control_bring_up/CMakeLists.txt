cmake_minimum_required(VERSION 3.22.0)
project(helios_control_bring_up)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  descriptions
)