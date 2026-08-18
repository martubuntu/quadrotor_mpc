cmake_minimum_required(VERSION 3.0.2)
project(uav_mpc)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  roslib
)

find_package(Eigen3 REQUIRED)

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
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime roslib
)

add_library(mpc_solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_mpc_codegen/acado_qpoases_interface.cpp
  model/quadrotor_mpc_codegen/acado_integrator.c
  model/quadrotor_mpc_codegen/acado_solver.c
  model/quadrotor_mpc_codegen/acado_auxiliary_functions.c)
  
target_include_directories(mpc_solver PUBLIC
  model/quadrotor_mpc_codegen/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC)
  
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/model/quadrotor_mpc_codegen
  ${PROJECT_SOURCE_DIR}/externals/qpoases
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_executable(mpc_node
  src/mpc_node.cpp
  src/mpc_wrapper.cpp
  src/ros_mission.cpp)
  
target_link_libraries(mpc_node
  mpc_solver
  ${catkin_LIBRARIES})
add_dependencies(mpc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(eso_observer_node
  src/eso_observer_node.cpp)

target_link_libraries(eso_observer_node
  ${catkin_LIBRARIES})
add_dependencies(eso_observer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(circle_traj_node
  src/circle_traj_node.cpp)

target_link_libraries(circle_traj_node
  ${catkin_LIBRARIES})
add_dependencies(circle_traj_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(data_logger_node
  src/data_logger_node.cpp)

target_link_libraries(data_logger_node
  ${catkin_LIBRARIES})
add_dependencies(data_logger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pid_baseline_node
  src/pid_baseline_node.cpp)

target_link_libraries(pid_baseline_node
  ${catkin_LIBRARIES})
add_dependencies(pid_baseline_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
