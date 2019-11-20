#include "ros/ros.h"
#include <iostream>
#include <teb_local_planner/FeedbackMsg.h>

using namespace teb_local_planner;
using namespace ros;

void msgCallback(const FeedbackMsg::ConstPtr& msg)
{
    ROS_INFO(" Robot index ");
    ROS_INFO("Robot index : %d",msg->selected_trajectory_idx);
    int n = msg->selected_trajectory_idx;
    ROS_INFO("Trajectory : %lf",msg->trajectories[n].trajectory[n].velocity.linear.x);
    ROS_INFO("Trajectory : %lf",msg->trajectories[n].trajectory[n].velocity.linear.y);

}


int main(int argc,char** argv)
{
    init(argc,argv,"Test_subs");
    ROS_INFO(" Start");

    NodeHandle nh;

    Subscriber sub_msg = nh.subscribe("/move_base/TebLocalPlannerROS/teb_feedback",10,msgCallback);

    spin();

    return 0;
}