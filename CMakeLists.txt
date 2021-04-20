cmake_minimum_required(VERSION 3.2.1)
project(anytime-planning)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin and any catkin packages
find_package(
  catkin REQUIRED COMPONENTS 
  rospy
  urdf
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Declare a catkin package
catkin_package(
	INCLUDE_DIRS
	CATKIN_DEPENDS rospy geometry_msgs
	DEPENDS
)


# Build target library
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Add executables
add_executable(explorer src/explorer.cc src/pathFinder.cc src/nodes.cc src/actions.cc src/consoleLogger.cc)
find_package( OpenCV REQUIRED )
target_link_libraries( explorer ${OpenCV_LIBS} )