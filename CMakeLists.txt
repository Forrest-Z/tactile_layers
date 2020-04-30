cmake_minimum_required(VERSION 2.8.3)
project(tactile_layers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(tactile_layer src/tactile_layer.cpp)