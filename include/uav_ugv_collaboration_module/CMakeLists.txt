add_library(utils_lib utils.cpp)
target_link_libraries(utils_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )
add_dependencies(pointcloud_lib ${catkin_EXPORTED_TARGETS}) 

