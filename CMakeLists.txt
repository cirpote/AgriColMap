cmake_minimum_required(VERSION 2.8.3)
project(uav_ugv_collaboration_module)

#To uncomment in the future
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++11") # comment -march=native for ubuntu 18.04

set(CMAKE_BUILD_TYPE Release)

message(STATUS "PROCESSOR_TYPE: [${CMAKE_HOST_SYSTEM_PROCESSOR}]")
if (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES armv7l)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  message("ENABLING ARM NEON OPTIMIZATIONS")
endif ()

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

find_package(OpenCV REQUIRED)
  message(STATUS "${PROJECT_NAME}: OpenCV version ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})
  message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

find_package(PCL REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(yaml-cpp REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/CPM)
add_subdirectory(${PROJECT_SOURCE_DIR}/cpd)
add_subdirectory(${PROJECT_SOURCE_DIR}/src_GoICP)
add_subdirectory(${PROJECT_SOURCE_DIR}/include)
