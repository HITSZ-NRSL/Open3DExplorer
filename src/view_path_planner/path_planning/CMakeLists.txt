cmake_minimum_required(VERSION 2.8.3)

# set (PCL_INCLUDE_DIRS "/usr/local/include/pcl-1.11/pcl")
# set (OCTOMAP_LIBRARIES "/usr/local/include/octomap")

project(path_planning)
# set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++14)

find_package(octomap 1.7 REQUIRED)

# find_package(pcl 1.11 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  # octomap
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planning
  CATKIN_DEPENDS octomap_msgs octomap_ros pcl_conversions pcl_ros roscpp std_msgs
  DEPENDS octomap PCL
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_libraries(${OCTOMAP_LIBRARIES} )


# add_executable(${PROJECT_NAME}_node src/path_planning.cpp)
AUX_SOURCE_DIRECTORY(src DIR_SRCS)
add_library(KDTree src/KDTree.cpp)

# add_library(octomap_rrt src/octomap_rrt.cpp)


SET(TEST_MATH
${DIR_SRCS}
)

ADD_EXECUTABLE(${PROJECT_NAME}_node ${TEST_MATH})

# ADD_EXECUTABLE(${PROJECT_NAME}_node src/polygon_pub.cpp)

message(${OCTOMAP_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   yaml-cpp
)