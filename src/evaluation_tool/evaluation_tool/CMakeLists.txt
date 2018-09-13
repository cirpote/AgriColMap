add_library(evaluation_tool_lib evaluation_tool.cpp  )
target_link_libraries(evaluation_tool_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
add_dependencies(evaluation_tool_lib ${catkin_EXPORTED_TARGETS})
