#include <vehicle_control/wheel_odometry.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_odom");
    wheel_odom wheel_odometry;
    wheel_odometry.wheel_odom_sub();
  
    ros::Rate loop_rate(200);


  while(ros::ok()){

    wheel_odometry.wheel_odom_pub();
    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
