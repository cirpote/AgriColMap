add_library(cpm_lib CPM.cpp)
target_link_libraries(cpm_lib ${OpenCV_LIBS} defs_lib)
