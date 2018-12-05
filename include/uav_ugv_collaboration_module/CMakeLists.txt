add_library(defs_lib defs.cpp)
target_link_libraries(defs_lib ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library(utils_lib utils.cpp)
target_link_libraries(utils_lib defs_lib )