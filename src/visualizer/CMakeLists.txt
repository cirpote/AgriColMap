add_library(visualizer_lib visualizer.cpp)
target_link_libraries(visualizer_lib ${OpenCV_LIBS} ${PCL_LIBRARIES})