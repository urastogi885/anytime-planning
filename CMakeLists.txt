cmake_minimum_required(VERSION 3.2.1)
project(ata_star)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Build target library
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

## Add executables
add_executable(explorer src/explorer.cc src/pathFinder.cc src/actions.cc src/errorLogger.cc)
find_package( OpenCV REQUIRED )
target_link_libraries( explorer ${OpenCV_LIBS} )