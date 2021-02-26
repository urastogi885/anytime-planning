cmake_minimum_required(VERSION 2.8.3)
project(ata_star)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Build target library
include_directories(
  include
)

## Add executables
add_executable(explorer src/explorer.cpp src/actions.cpp)