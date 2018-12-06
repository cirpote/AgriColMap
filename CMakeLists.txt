cmake_minimum_required(VERSION 2.8.3)
project(uav_ugv_collaboration_module)

#To uncomment in the future
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++11 -fpic") # comment -march=native for ubuntu 18.04

set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
  message("${PROJECT_NAME}|OpenCV version ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

message("${PROJECT_NAME}|PROCESSOR_TYPE: [${CMAKE_HOST_SYSTEM_PROCESSOR}]")
if (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES armv7l)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  message("ENABLING ARM NEON OPTIMIZATIONS")
endif ()

find_package(PCL REQUIRED) 
  message("${PROJECT_NAME}|PCL version ${PCL_VERSION}")
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


set(PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/packageDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/src/packageDir.h)