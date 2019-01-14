add_library(visualizer_lib visualizer.cpp)
target_link_libraries(visualizer_lib ${Open3D_LIBRARIES})
target_include_directories(visualizer_lib PUBLIC ${Open3D_INCLUDE_DIRS})
