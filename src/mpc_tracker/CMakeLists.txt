cmake_minimum_required(VERSION 3.0.2)
project(mpc_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
# ADD_COMPILE_OPTIONS(-std=c++11 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
add_definitions(-std=c++14 -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Eigen3 REQUIRED)
find_package(octomap 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES mpc_tracker
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS octomap_msgs octomap_ros roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
)
link_libraries(${OCTOMAP_LIBRARIES} )

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# add_definitions(${EIGEN3_DEFINITIONS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mpc_tracker.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

# add_executable(${PROJECT_NAME} src/cppad_demo.cpp)

add_executable(mpc_tracker src/mpc_app.cpp src/mpc.cpp)

# add_executable(mpc_tracker src/mpc_control_test_nonholonomic.cpp src/mpc_nonholonomic.cpp src/uniform_bspline.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   ipopt
# )
message(${OCTOMAP_LIBRARIES})

target_link_libraries(mpc_tracker
  ${catkin_LIBRARIES}
  ipopt
  ${OCTOMAP_LIBRARIES}
)
