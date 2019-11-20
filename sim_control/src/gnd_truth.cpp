#include <math.h>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sim_control/groundMsg.h>

// From odometry msg
float pose_gnd[2];
float v_gnd[2];
float quat_gnd[4];

// From Kinect
//double pose_rgbd[2];
//double v_rgbd[2];
//double quat_rgbd[4];


void gndCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose_gnd[0] = msg->pose.pose.position.x;
    pose_gnd[1] = msg->pose.pose.position.y;

    v_gnd[0]    = msg->twist.twist.linear.x;
    v_gnd[1]    = msg->twist.twist.linear.y;

    quat_gnd[0] = msg->pose.pose.orientation.x;
    quat_gnd[1] = msg->pose.pose.orientation.y;
    quat_gnd[2] = msg->pose.pose.orientation.z;
    quat_gnd[3] = msg->pose.pose.orientation.w;
}

//void rgbdCallback(const nav_msgs::Odometry::ConstPtr& msg)
//{
//    pose_rgbd[0] = msg->pose.pose.position.x;
//    pose_rgbd[1] = msg->pose.pose.position.y;
//
//    v_rgbd[0]    = msg->twist.twist.linear.x;
//    v_rgbd[1]    = msg->twist.twist.linear.y;
//
//    quat_rgbd[0] = msg->pose.pose.orientation.x;
//    quat_rgbd[1] = msg->pose.pose.orientation.y;
//    quat_rgbd[2] = msg->pose.pose.orientation.z;
//    quat_rgbd[3] = msg->pose.pose.orientation.w;
//}

/* Main function */

int main(int argc, char **argv)
{
  ros::init(argc,argv,"vehicle_control");
  ros::NodeHandle nh;

  //100 que size//
  ros::Publisher publ_gnd = nh.advertise<sim_control::groundMsg>("/gnd",1);
//  ros::Publisher publ_rgbd = nh.advertise<simulation_control::groundMsg>("/rgbd_odom",1);


  ros::Subscriber sub_gnd = nh.subscribe("/gnd_truth",1,gndCallback,ros::TransportHints().unreliable().reliable().maxDatagramSize(1000).tcpNoDelay());
//  ros::Subscriber sub_rgbd = nh.subscribe("/kinect/odom",1,rgbdCallback,ros::TransportHints().unreliable().reliable().maxDatagramSize(1000).tcpNoDelay());

  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    // Message Declaration
    sim_control::groundMsg gnd;
//    simulation_control::groundMsg rgbd;


    gnd.x_gnd = pose_gnd[0];
    gnd.y_gnd = pose_gnd[1];
    gnd.vx_gnd = v_gnd[0];
    gnd.vy_gnd = v_gnd[1];
    gnd.qx_gnd = quat_gnd[0];
    gnd.qy_gnd = quat_gnd[1];
    gnd.qz_gnd = quat_gnd[2];
    gnd.qw_gnd = quat_gnd[3];


//    rgbd.x_gnd = pose_rgbd[0];
//    rgbd.y_gnd = pose_rgbd[1];
//    rgbd.vx_gnd = v_rgbd[0];
//    rgbd.vy_gnd = v_rgbd[1];
//    rgbd.qx_gnd = quat_rgbd[0];
//    rgbd.qy_gnd = quat_rgbd[1];
//    rgbd.qz_gnd = quat_rgbd[2];
//    rgbd.qw_gnd = quat_rgbd[3];


    publ_gnd.publish(gnd);
//    publ_rgbd.publish(rgbd);

    ros::spinOnce();
    loop_rate.sleep();

  }
    return 0;
}
