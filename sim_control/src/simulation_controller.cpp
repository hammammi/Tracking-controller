#include <math.h>
#include "ros/ros.h"
#include "sim_control/cmdMsg.h"
#include "sim_control/motorDynamics.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

#define _USE_MATH_DEFINES

// From commend_msg //
double pose_des[3];

// From odometry msg //
double pose_gnd[2];
double quat_gnd[4];


/* Callback function to subscribe */

void cmdCallback(const sim_control::cmdMsg::ConstPtr& cmd_msg)
{
    pose_des[0]   = cmd_msg->xd;
    pose_des[1]   = cmd_msg->yd;
    pose_des[2]   = cmd_msg->phid;
}

void gndCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose_gnd[0] = msg->pose.pose.position.x;
    pose_gnd[1] = msg->pose.pose.position.y;

    quat_gnd[0] = msg->pose.pose.orientation.x;
    quat_gnd[1] = msg->pose.pose.orientation.y;
    quat_gnd[2] = msg->pose.pose.orientation.z;
    quat_gnd[3] = msg->pose.pose.orientation.w;
}

/* Main function */

int main(int argc, char **argv)
{
  ros::init(argc,argv,"vehicle_control");
  ros::NodeHandle nh;

  //100 que size//
  ros::Publisher publ_input = nh.advertise<sim_control::motorDynamics>("/input_msg",100);

  // Quesize : 100 //

  ros::Subscriber sub1 = nh.subscribe("/cmd_msg",100,cmdCallback);
  ros::Subscriber sub2 = nh.subscribe("/odom",100,gndCallback);

  // Publish rate : 200 Hz //
  ros::Rate loop_rate(200);

/** Initialization for controller **/
  // reference {prev, current} //
  float x_d      = 0;
  float y_d      = 0;
  float phi_d    = 0;
  
  // Feedback data //
  float x_gnd    = 0;
  float y_gnd    = 0;
  float phi_gnd  = 0;

  float x_quat   = 0;
  float y_quat   = 0;
  float z_quat   = 0;
  float w_quat   = 0;

  float angle = 0;

  // Input(Velocity) //
  float del_s = 0;
  float vel_linear;

  float u_x   = 0;
  float u_y   = 0;
  float u_p   = 0;

  // Motor speed in rad/sec - initialization {prev,curr}//
  float wheel_speed_lf = 0;
  float wheel_speed_rf = 0;
  float wheel_speed_lb = 0;
  float wheel_speed_rb = 0;

  // Motor speed in RPM - initialization //

  int w1 = 0;
  int w2 = 0;
  int w3 = 0;
  int w4 = 0;

  // Time //
  float prev_t = 0;
  float dt = 0.01;
  float curr_t=0.01;


/** Controller gains Setting **/

  // P control //
  float kp_s = 1.0;
  float kp_phi = 1.0;

  float v_lim       = 0.6;
  float dphidt_lim  = 0.6;

/* Wheel specification - unit: meter */
  float wheel_diameter = 0.152;
  float wheel_radius = wheel_diameter / 2.0;
  float wheel_separation_a = 0.2600;
  float wheel_separation_b = 0.2680;
  float l = wheel_separation_a + wheel_separation_b;


// motor specification //
  int gear_ratio = 74;

// radps_to_rpm : rad/sec --> rpm //
// rpm_to_radps : rpm --> rad/sec //

  float radps_to_rpm = 60.0/2.0/M_PI;
  float rpm_to_radps = 2.0 * M_PI / 60;

  while(ros::ok())
  {

    sim_control::motorDynamics input_msg;

   //Time dt : loop time
  curr_t = ros::Time::now().toSec();
  dt = curr_t - prev_t;
  prev_t = curr_t;

  phi_gnd =  atan2(2.0 * (quat_gnd[3] * quat_gnd[2] + quat_gnd[0] * quat_gnd[1]),1.0-2.0 * (quat_gnd[1] * quat_gnd[1] + quat_gnd[2] * quat_gnd[2]));

    // Current feedback value
    x_gnd = pose_gnd[0];
    y_gnd = pose_gnd[1];

    x_d   = pose_des[0];
    y_d   = pose_des[1];
    phi_d = pose_des[2];

    // Current values from Odometry  //
    x_quat   = quat_gnd[0];
    y_quat   = quat_gnd[1];
    z_quat   = quat_gnd[2];
    w_quat   = quat_gnd[3];

    angle = atan2(y_d-y_gnd,x_d-x_gnd);

   /* P controller */

  del_s = sqrt( (x_d - x_gnd) * (x_d - x_gnd) + (y_d - y_gnd) * (y_d - y_gnd));     // distance b/t the desired position and mobile robot's one

  // Control Input
  vel_linear = kp_s * del_s;
  u_p =  kp_phi * (phi_d-phi_gnd);


    if(vel_linear>v_lim)
    {
        vel_linear = v_lim;
    }

    if(dphidt_lim < u_p)
    {
      u_p = dphidt_lim;
    }
    else if(-dphidt_lim > u_p)
    {
      u_p = -dphidt_lim;
    }

    u_x = vel_linear * cos(angle-phi_gnd);
    u_y = vel_linear * sin(angle-phi_gnd);

    // Inverse Kinematics for motor input (uint : RPM) //
    w1 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x - u_y - l * u_p);
    w2 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x + u_y + l * u_p);
    w3 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x + u_y - l * u_p);
    w4 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x - u_y + l * u_p);

    // Model - motor's velocity //
    wheel_speed_lf = (float) w1 * rpm_to_radps;
    wheel_speed_rf = (float) w2 * rpm_to_radps;
    wheel_speed_lb = (float) w3 * rpm_to_radps;
    wheel_speed_rb = (float) w4 * rpm_to_radps;

    if(u_x * u_x + u_y * u_y + u_p * u_p < 0.05*0.05){
        w1 = 0;
        w2 = 0;
        w3 = 0;
        w4 = 0;

        u_x = 0;
        u_y = 0;
        u_p = 0;
    }

    input_msg.omega1 = -w1*gear_ratio;
    input_msg.omega2 = w2*gear_ratio;
    input_msg.omega3 = -w3*gear_ratio;
    input_msg.omega4 = w4*gear_ratio;

    publ_input.publish(input_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}