cmake_minimum_required(VERSION 2.8.3)
project(motoman_viz)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_move_group
  moveit_msgs
  motoman_viz_msgs
  tf
  tf_conversions
  eigen_conversions
  )

catkin_package(
  CATKIN_DEPENDS
  )

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
add_definitions(-std=c++11)

# Build
include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

add_executable(forward_kinematics_solver src/forward_kinematics_solver.cpp)

target_link_libraries(forward_kinematics_solver
  ${catkin_LIBRARIES}
  )


add_executable(forward_kinematics_solver_joint_state src/forward_kinematics_solver_joint_state.cpp)

target_link_libraries(forward_kinematics_solver_joint_state
  ${catkin_LIBRARIES}
  )
