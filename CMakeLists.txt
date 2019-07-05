cmake_minimum_required(VERSION 2.8.3)
project(uav_ugv_collaboration_module)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++11 -fpic -pthread -march=native") 
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)
option(BUILD_WITH_PCL "Build with ad-hoc PCL library" off)

find_package(OpenCV REQUIRED)
  message("${PROJECT_NAME}|OpenCV version ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
find_package(yaml-cpp REQUIRED)

# BUILD With the ad-hod PCL installation
if(BUILD_WITH_PCL)
  add_definitions(-DBUILD_WITH_PCL )
  include_directories(
    ${PROJECT_SOURCE_DIR}/pcl
    ${PROJECT_SOURCE_DIR}/pcl/io/include
    ${PROJECT_SOURCE_DIR}/pcl/common/include
    ${PROJECT_SOURCE_DIR}/pcl/build/include
    ${PROJECT_SOURCE_DIR}/pcl/visualization/include
    ${PROJECT_SOURCE_DIR}/pcl/geometry/include
    ${PROJECT_SOURCE_DIR}/pcl/filters/include
    ${PROJECT_SOURCE_DIR}/pcl/sample_consensus/include
    ${PROJECT_SOURCE_DIR}/pcl/search/include
    ${PROJECT_SOURCE_DIR}/pcl/features/include
    ${PROJECT_SOURCE_DIR}/pcl/kdtree/include
    ${PROJECT_SOURCE_DIR}/pcl/octree/include
  )
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
  find_package(VTK)
  include (${VTK_USE_FILE})
  include_directories(${VTK_INCLUDE_DIRS})
  find_package(Boost 1.4.0 COMPONENTS system filesystem REQUIRED)
  include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
else()
  find_package(PCL REQUIRED) 
  include_directories(${PCL_INCLUDE_DIRS})
  message("${PROJECT_NAME}|PCL version ${PCL_VERSION}")
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/CPM)
add_subdirectory(${PROJECT_SOURCE_DIR}/cpd)
add_subdirectory(${PROJECT_SOURCE_DIR}/src_GoICP)
add_subdirectory(${PROJECT_SOURCE_DIR}/include)

set(PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/packageDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/src/packageDir.h)