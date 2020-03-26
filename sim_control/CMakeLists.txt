cmake_minimum_required(VERSION 2.8.3)
project(sim_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  tf
  nav_msgs
  sensor_msgs
)

add_message_files(
  FILES
  desiredMsg.msg
  motorDynamics.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sim_control
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(tracking_controller_pid src/tracking_controller_pid.cpp)
target_link_libraries(tracking_controller_pid ${catkin_LIBRARIES})
add_dependencies(tracking_controller_pid sim_control_generate_messages_cpp)

add_executable(motor_dynamics src/motor_dynamics.cpp)
target_link_libraries(motor_dynamics ${catkin_LIBRARIES})
add_dependencies(motor_dynamics sim_control_generate_messages_cpp)
