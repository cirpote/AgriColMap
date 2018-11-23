add_library(visualizer_lib visualizer.cpp)
target_link_libraries(visualizer_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
add_dependencies(visualizer_lib ${catkin_EXPORTED_TARGETS}) 
