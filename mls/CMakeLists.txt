cmake_minimum_required(VERSION 2.8.3)
project(mls)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs
	geometry_msgs pcl_ros tf tf_conversions nav_msgs eigen_conversions visualization_msgs ground_segmentation grid_map_ros grid_map_cv grid_map_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS include
LIBRARIES mls
)


###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(mls src/mls.cpp)
target_link_libraries(mls ${catkin_LIBRARIES})

install(TARGETS mls
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)