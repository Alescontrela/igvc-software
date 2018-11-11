add_executable(pather main.cpp)
add_dependencies(pather igvc_msgs_gencpp)
target_link_libraries(pather ${catkin_LIBRARIES})
