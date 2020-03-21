# Tracking-controller

In the include folder

Header file directory - /sim_control/include/sim_control/tracking_controller_rev_slam.hpp

Source file directory - /sim_control/src/tracking_controller_rev.cpp

wheel separation info

--> l_a , and l_b

1. Execution of tracking controller in actual mobile robot

Xavier $ sudo ifconfig eth1 192.168.3.70

Xavier $ sudo route add 192.168.1.202 eth1

Xavier $ roslaunch velodyne_pointcloud VLP16_points.launch

Check out whether the scan data is published well.

Xavier $ roslaunch dual_arm_bringup bringup.launch

Laptop $ roslaunch dual_arm_localization gmapping.launch

RaspberryPi $ rosrun epos_tutorial controller_pdo

Xavier $ rosrun vehicle_control wheel_odometry

Xavier $ rosrun sim_control tracking_controller_rev

Laptop $ rosrun path_planner velicity_plan

2. Execution of tracking controller in simulation

$ roslaunch mobile_platform_description gazebo_v1.launch

$ roslaunch dual_arm_localization gmapping.launch

$ rosrun sim_control motor_dynamics

$ rosrun sim_control tracking_controller_rev

$ rosrun path_planner velocity_plan
