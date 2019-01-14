add_library(defs_lib defs.cpp)
target_link_libraries(defs_lib ${OpenCV_LIBS} ${Open3D_LIBRARIES})
target_include_directories(defs_lib PUBLIC ${Open3D_INCLUDE_DIRS})

add_library(utils_lib utils.cpp)
target_link_libraries(utils_lib defs_lib)


