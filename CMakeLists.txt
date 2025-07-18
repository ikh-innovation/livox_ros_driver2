# Copyright(c) 2019 livoxtech limited.

cmake_minimum_required(VERSION 3.0)


#---------------------------------------------------------------------------------------
# Start livox_ros_driver2 project
#---------------------------------------------------------------------------------------
include(cmake/version.cmake)
project(livox_ros_driver2 VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
message(STATUS "livox_ros_driver2 version: ${LIVOX_ROS_DRIVER2_VERSION}")

#---------------------------------------------------------------------------------------
# find package and the dependecy
#---------------------------------------------------------------------------------------
find_package(Boost 1.54 REQUIRED COMPONENTS
  system
  thread
  chrono
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  dynamic_reconfigure
  rosbag
  pcl_ros
  nodelet
  pluginlib
)

generate_dynamic_reconfigure_options(
  cfg/Filter.cfg
  cfg/CustomFilter.cfg
)

## Find pcl lib
find_package(PCL REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(FILES
  CustomPoint.msg
  CustomMsg.msg
  FovConfig.msg
  HostIpConfig.msg
  IpConfig.msg
  InstallAttitude.msg
  StateInfoMsg.msg
#   Message2.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
  message(${APR_INCLUDE_DIRS})
  message(${APR_LIBRARIES})
endif (APR_FOUND)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects als    o need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also n    eed
catkin_package(
  INCLUDE_DIRS src/include
  LIBRARIES ${PROJECT_NAME}_nodelet filter_nodelet custom_filter_nodelet
  CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure message_runtime nodelet pcl_ros
)

#---------------------------------------------------------------------------------------
# Set default build to release
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## make sure the livox_lidar_sdk_shared library is installed
unset(LIVOX_LIDAR_SDK_LIBRARY CACHE)
set(CMAKE_FIND_LIBRARY_SUFFIXES ".so")  
find_library(LIVOX_LIDAR_SDK_LIBRARY  liblivox_lidar_sdk_shared.so  /usr/local/lib)
set(CMAKE_FIND_LIBRARY_SUFFIXES ".so" ".a")


## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_nodelet
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp
  src/lddc.cpp
  src/livox_ros_driver2.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/lidar_state_info_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

add_library(filter_nodelet
  src/filter.cpp
)

add_library(custom_filter_nodelet
  src/filter_custom.cpp
)

add_library(livox_to_pointcloud2
  src/livox_to_pointcloud2.cpp
)

add_library(pointcloud2_to_livox
  src/pointcloud2_to_livox.cpp
)

add_library(pcl_analyzer
  src/pcl_analyzer.cpp
)

#---------------------------------------------------------------------------------------
# precompile macro and compile option
#---------------------------------------------------------------------------------------
target_compile_options(${PROJECT_NAME}_nodelet
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
)

target_compile_options(filter_nodelet
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
)

target_compile_options(custom_filter_nodelet
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
)

target_compile_options(livox_to_pointcloud2
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
)

target_compile_options(pointcloud2_to_livox
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
)

target_compile_options(pcl_analyzer
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
)

#---------------------------------------------------------------------------------------
# add projects that depend on
#---------------------------------------------------------------------------------------
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(filter_nodelet ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(custom_filter_nodelet ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(livox_to_pointcloud2 ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(pointcloud2_to_livox ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(pcl_analyzer ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

#---------------------------------------------------------------------------------------
# include file
#---------------------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME}_nodelet
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
  3rdparty
  src
)

target_include_directories(filter_nodelet
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
)

target_include_directories(custom_filter_nodelet
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
)

target_include_directories(livox_to_pointcloud2
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
)

target_include_directories(pointcloud2_to_livox
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
)

target_include_directories(pcl_analyzer
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
)
#---------------------------------------------------------------------------------------
# link libraries
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}_nodelet
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${Boost_LIBRARY}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
)

target_link_libraries(filter_nodelet
  ${Boost_LIBRARY}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
)

target_link_libraries(custom_filter_nodelet
  ${Boost_LIBRARY}
  ${catkin_LIBRARIES}
  ${APR_LIBRARIES}
)

target_link_libraries(livox_to_pointcloud2
  ${Boost_LIBRARY}
  ${catkin_LIBRARIES}
  ${APR_LIBRARIES}
)

target_link_libraries(pointcloud2_to_livox
  ${Boost_LIBRARY}
  ${catkin_LIBRARIES}
  ${APR_LIBRARIES}
)

target_link_libraries(pcl_analyzer
  ${Boost_LIBRARY}
  ${catkin_LIBRARIES}
  ${APR_LIBRARIES}
)


#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS filter_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS custom_filter_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES ${PROJECT_NAME}_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES library_filter_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES library_custom_filter_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES library_livox_to_pointcloud2.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES library_pointcloud2_to_livox.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES library_pcl_analyzer.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#---------------------------------------------------------------------------------------
# end of CMakeList.txt
#---------------------------------------------------------------------------------------
