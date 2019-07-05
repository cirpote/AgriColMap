add_library(defs_lib defs.cpp)
if(BUILD_WITH_PCL)
    target_link_libraries(defs_lib 
                         ${VTK_LIBRARIES}
                         ${Boost_LIBRARIES}
                         ${OpenCV_LIBS}  
                         ${PROJECT_SOURCE_DIR}/pcl/build/lib/libpcl_common.so
                         ${PROJECT_SOURCE_DIR}/pcl/build/lib/libpcl_features.so
                         ${PROJECT_SOURCE_DIR}/pcl/build/lib/libpcl_io_ply.so
                         ${PROJECT_SOURCE_DIR}/pcl/build/lib/libpcl_segmentation.so
                         ${PROJECT_SOURCE_DIR}/pcl/build/lib/libpcl_visualization.so)
else()
    target_link_libraries(defs_lib  
                         ${OpenCV_LIBS} 
                         ${PCL_LIBRARIES}) 
endif()

add_library(utils_lib utils.cpp)
target_link_libraries(utils_lib defs_lib)