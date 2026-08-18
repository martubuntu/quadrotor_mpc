cmake_minimum_required(VERSION 3.0.2)

project(traj_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  quadrotor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  quadrotor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(circle_trajectory_node
  src/circle_trajectory_node.cpp
)

add_dependencies(circle_trajectory_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(circle_trajectory_node
  ${catkin_LIBRARIES}
)
add_executable(uav_data_logger
  src/uav_data_logger.cpp
)

add_dependencies(uav_data_logger
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(uav_data_logger
  ${catkin_LIBRARIES}
)
