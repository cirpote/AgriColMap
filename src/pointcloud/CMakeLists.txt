add_library(pointcloud_lib pointcloud.cpp)
target_link_libraries(pointcloud_lib utils_lib GoICP_lib)
add_dependencies(pointcloud_lib ${catkin_EXPORTED_TARGETS}) 
