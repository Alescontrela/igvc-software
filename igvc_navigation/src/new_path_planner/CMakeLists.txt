add_executable(generate_path_server main.cpp searchlocation.cpp searchmove.cpp igvcsearchproblemdiscrete.cpp)
add_dependencies(generate_path_server igvc_msgs_gencpp)
target_link_libraries(generate_path_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})
