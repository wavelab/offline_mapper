cmake_minimum_required(VERSION 2.8.3)
project(laser_slam)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules")


#select G2O or GTSAM as the backend optmizer (G2O is default).  G2O requires the libg2o ROS package installation, GTSAM requires installation of GTSAM from source
set(OPTIMIZER "G2O")

if(OPTIMIZER STREQUAL "GTSAM")
	message("Building using GTSAM")
	## This is the standard installation dir for GTSAM
    ## You may need to change this for your system
    set(GTSAM_DIR "/usr/local/lib/cmake/GTSAM")
else()
	message("Building using G2O")

endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        roscpp
        std_msgs
        sensor_msgs
        nav_msgs
	      geometry_msgs 
        grid_map_ros 
        grid_map_cv 
        grid_map_msgs 
        pcl_ros 
        tf 
        tf_conversions 
        message_generation 
        visualization_msgs 
        mls
        eigen_conversions 
        ground_segmentation 
        actionlib 
        std_srvs
        anm_msgs
        dbw_mkz_msgs)
find_package(SUITESPARSE REQUIRED)

if(OPTIMIZER STREQUAL "GTSAM")
	find_package(GTSAM)
	set(OPTIMIZER_INCLUDE_DIRS GTSAM_INCLUDE_DIRS)
	set(OPTIMIZER_SRC_FILES src/optimizer_impl_gtsam.cpp)
	set(OPTIMIZER_LIBS gtsam)
else()
	find_package(G2O REQUIRED)
	message(STATUS "G2O_CORE_LIBRARY=${G2O_CORE_LIBRARY}")
	set(OPTIMIZER_INCLUDE_DIRS G2O_INCLUDE_DIRS)
	set(OPTIMIZER_SRC_FILES src/optimizer_impl_g2o.cpp)
	set(OPTIMIZER_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_TYPES_SLAM3D} ${G2O_SOLVER_CSPARSE_EXTENSION})
endif()


add_service_files(
  DIRECTORY srv
  FILES InsertPointCloud.srv
)

generate_messages(
        DEPENDENCIES 
        std_msgs 
        geometry_msgs 
        sensor_msgs 
        nav_msgs 
        grid_map_msgs 
        anm_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -O3")
include_directories(
  include
  ${mls_INCLUDE_DIRS} 
  ${ccicp2d_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
${SUITESPARSE_INCLUDE_DIRS}
${GTSAM_INCLUDE_DIRS}
)


link_directories(
       ${catkin_LIBRARY_DIRS}
       ${SUITESPARSE_LIBRARY_DIRS}
)

add_executable(laser_slam_service_from_bag src/laser_slam_service_from_bag.cpp)
add_executable(laser_slam_service_tester  src/laser_slam_service_tester.cpp src/csv_tools.cpp)
add_executable(laser_slam src/laser_slam.cpp src/global_mapping_viz.cpp src/scan_registration.cpp src/csv_tools.cpp src/main.cpp ${OPTIMIZER_SRC_FILES})

add_dependencies( laser_slam_service_from_bag
                  ${${PROJECT_NAME}_EXPORTED_TARGETS}
                  ${catkin_EXPORTED_TARGETS} )

add_dependencies( laser_slam_service_tester
                  ${${PROJECT_NAME}_EXPORTED_TARGETS}
                  ${catkin_EXPORTED_TARGETS} )

add_dependencies( laser_slam
                  ${${PROJECT_NAME}_EXPORTED_TARGETS}
                  ${catkin_EXPORTED_TARGETS} )

target_link_libraries (laser_slam ${catkin_LIBRARIES} ${OPTIMIZER_LIBS})
target_link_libraries (laser_slam_service_tester ${catkin_LIBRARIES})
target_link_libraries (laser_slam_service_from_bag ${catkin_LIBRARIES})

install(TARGETS laser_slam_service_tester laser_slam_service_from_bag laser_slam
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(    DIRECTORY launch config test_data
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
