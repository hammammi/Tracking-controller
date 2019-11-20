#include <math.h>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sim_control/slamposeMsg.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>

float x_est;
float y_est;
float phi_est;

int main(int argc, char** argv){

  ros::init(argc, argv,"SLAM_pose");
  ros::NodeHandle nh;

  ros::Publisher slam_pose_pub = nh.advertise<sim_control::slamposeMsg>("/pose_slam_v2",1);

  tf::TransformListener listener;

  ros::Rate rate(50);

  while (nh.ok()){
    tf::StampedTransform transform;
    try{

      listener.lookupTransform("/map","/base_footprint",ros::Time(0),transform);

    }
    catch (tf::TransformException ex){

      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    sim_control::slamposeMsg pose;

    pose.x_est = transform.getOrigin().x();
    pose.y_est = transform.getOrigin().y();
//    pose.phi_est = atan2(transform.getOrigin().y(),transform.getOrigin().x());

    slam_pose_pub.publish(pose);

    rate.sleep();

  }
    return 0;
}
