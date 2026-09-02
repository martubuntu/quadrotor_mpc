cmake_minimum_required(VERSION 3.8)
project(uav_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MpcRefPoint.msg"
  "msg/MpcRefTraj.msg"
  DEPENDENCIES geometry_msgs
)

add_library(mpc_solver STATIC
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
  model/quadrotor_mpc_codegen/acado_auxiliary_functions.c
)

target_include_directories(mpc_solver PUBLIC
  model/quadrotor_mpc_codegen
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
)
set_target_properties(mpc_solver PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mpc_ros_core src/mpc_wrapper.cpp src/ros_mission.cpp)
target_include_directories(mpc_ros_core PUBLIC include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(mpc_ros_core mpc_solver Eigen3::Eigen)
ament_target_dependencies(mpc_ros_core
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs mavros_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mpc_ros_core "${cpp_typesupport_target}")

add_executable(mpc_node src/mpc_node.cpp)
target_link_libraries(mpc_node mpc_ros_core)
ament_target_dependencies(mpc_node rclcpp)

add_executable(circle_traj_node src/circle_traj_node.cpp)
target_include_directories(circle_traj_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(circle_traj_node Eigen3::Eigen "${cpp_typesupport_target}")
ament_target_dependencies(circle_traj_node rclcpp geometry_msgs nav_msgs mavros_msgs)

add_executable(data_logger_node src/data_logger_node.cpp)
target_include_directories(data_logger_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(data_logger_node Eigen3::Eigen)
ament_target_dependencies(data_logger_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs mavros_msgs)

install(TARGETS mpc_solver mpc_ros_core mpc_node circle_traj_node data_logger_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
