#include <math.h>
#include "ros/ros.h"
#include "vehicle_control/commendMsg.h"
#include "mobile_control/motorMsg.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#define _USE_MATH_DEFINES

double u_x;
double u_y;
double u_phi;


void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u_x = msg->linear.x;
    u_y = msg->linear.y;
    u_phi = msg->angular.z;
}


/* Main function */

int main(int argc, char **argv)
{
  ros::init(argc,argv,"vehicle_control");
  ros::NodeHandle nh;
  ros::Publisher publ_input = nh.advertise<mobile_control::motorMsg>("/input_msg",1000);


  ros::Subscriber sub2 = nh.subscribe("/cmd_vel",1000,velCallback);



  // Publish rate : 100Hz //
  ros::Rate loop_rate(100);


/** Initialization for controller **/
  // reference {prev, current} //
  double wheel_speed_lf = 0;
  double wheel_speed_rf = 0;
  double wheel_speed_lb = 0;
  double wheel_speed_rb = 0;

  // Motor speed in RPM - initialization //

  int w1 = 0;
  int w2 = 0;
  int w3 = 0;
  int w4 = 0;

  float prev_t = 0;
  float dt = 0.01;
  double curr_t=0.01;


/** Controller gains Setting **/



// Gear ratio //

  int gear_ratio = 76;

// radps_to_rpm : rad/sec -->Xrpm //
// rpm_to_radps : rpm --> rad/sec //

  double radps_to_rpm = 60.0/2.0/M_PI;
  double rpm_to_radps = 2.0 * M_PI / 60;
  double wheel_diameter = 0.152;
  double wheel_radius = wheel_diameter / 2.0;
  double wheel_separation_a = 0.2600;
  double wheel_separation_b = 0.2680;
  double l = wheel_separation_a + wheel_separation_b;




  while(ros::ok())
  {

    mobile_control::motorMsg input_msg;


    w1 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x - u_y - l * u_phi);
    w2 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x + u_y + l * u_phi);
    w3 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x - u_y + l * u_phi);
    w4 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x + u_y - l * u_phi);

    // Model - motor's velocity //
    wheel_speed_lf = (double) w1 * rpm_to_radps;
    wheel_speed_rf = (double) w2 * rpm_to_radps;
    wheel_speed_lb = (double) w3 * rpm_to_radps;
    wheel_speed_rb = (double) w4 * rpm_to_radps;

    input_msg.omega1 = w1 * gear_ratio;
    input_msg.omega2 = -w2 * gear_ratio;
    input_msg.omega3 = -w3 * gear_ratio;
    input_msg.omega4 = w4 * gear_ratio;

    publ_input.publish(input_msg);
    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
