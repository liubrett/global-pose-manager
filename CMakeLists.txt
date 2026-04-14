cmake_minimum_required(VERSION 3.0.2)
project(global_pose_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(global_pose_manager_node src/global_pose_manager_node.cpp)

target_link_libraries(global_pose_manager_node
  ${catkin_LIBRARIES}
)
