cmake_minimum_required(VERSION 2.8.3)
project(igvc_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  genmsg
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  actionlib
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  velocity_pair.msg
  lights.msg
  action_path.msg
  system_stats.msg
  map.msg
  igvc_path.msg
)

# Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  PathExecute.action
)

# Generate services in the 'srv' folder
add_service_files (
  DIRECTORY srv
  FILES
  GeneratePath.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package()
