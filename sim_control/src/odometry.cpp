#include <sim_control/odometry.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheel_odom");

  wheel_odom wheel_odometry;

  ros::Rate loop_rate(200);

  while(ros::ok()){


    wheel_odometry.wheel_odom_pub();
    ros::spin();
    loop_rate.sleep();

  }
  return 0;
}