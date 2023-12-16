cmake_minimum_required(VERSION 2.8.3)
project(gazebo_continuous_track)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roslib
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ContinuousTrackSimple
  CATKIN_DEPENDS gazebo_dev roslib
#  DEPENDS system_lib
)

## Include directory of ODE (gazebo/ode/ode.h, etc)
set(gazebo_ode_dev_INCLUDE_DIRS "")
foreach(gazebo_dev_INCLUDE_DIR ${gazebo_dev_INCLUDE_DIRS})
  list(APPEND gazebo_ode_dev_INCLUDE_DIRS ${gazebo_dev_INCLUDE_DIR}/gazebo)  
endforeach()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${gazebo_ode_dev_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  ContinuousTrack
  src/gazebo_continuous_track.cpp
)
add_library(
  ContinuousTrackSimple
  src/gazebo_continuous_track_simple.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ContinuousTrack
  ${catkin_LIBRARIES}
)
target_link_libraries(
  ContinuousTrackSimple
  ${catkin_LIBRARIES}
)
