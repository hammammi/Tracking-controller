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
  cmdMsg.msg
  desiredMsg.msg
  groundMsg.msg
  slamposeMsg.msg
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

add_executable(tracking_controller_rev src/tracking_controller_rev.cpp)
target_link_libraries(tracking_controller_rev ${catkin_LIBRARIES})
add_dependencies(tracking_controller_rev sim_control_generate_messages_cpp)

add_executable(motor_dynamics src/motor_dynamics.cpp)
target_link_libraries(motor_dynamics ${catkin_LIBRARIES})
add_dependencies(motor_dynamics sim_control_generate_messages_cpp)

add_executable(simulation_controller src/simulation_controller.cpp)
target_link_libraries(simulation_controller ${catkin_LIBRARIES})
add_dependencies(simulation_controller sim_control_generate_messages_cpp)

add_executable(simulation_controller2 src/simulation_controller2.cpp)
target_link_libraries(simulation_controller2 ${catkin_LIBRARIES})
add_dependencies(simulation_controller2 sim_control_generate_messages_cpp)

add_executable(gnd_truth src/gnd_truth.cpp)
target_link_libraries(gnd_truth ${catkin_LIBRARIES})
add_dependencies(gnd_truth sim_control_generate_messages_cpp)

add_executable(slam_pose src/slam_pose.cpp)
target_link_libraries(slam_pose ${catkin_LIBRARIES})
add_dependencies(slam_pose sim_control_generate_messages_cpp)

add_executable(route1_pub src/route1_pub.cpp)
target_link_libraries(route1_pub ${catkin_LIBRARIES})

add_executable(route2_pub src/route2_pub.cpp)
target_link_libraries(route2_pub ${catkin_LIBRARIES})

add_executable(rectangle_simulation src/rectangle_simulation.cpp)
target_link_libraries(rectangle_simulation ${catkin_LIBRARIES})