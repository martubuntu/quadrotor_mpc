cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

add_message_files(
  FILES
  mpc_ref_point.msg
  mpc_ref_traj.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)
