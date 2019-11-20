#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <math.h>


int main(int argc, char **argv)
{
  ros::init(argc,argv,"cmd_publisher2");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub=nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",100); //100 que size//

  int hz1 = 10;
//  int hz2 = 20; // 0.30m/s

  ros::Rate loop_rate(hz1); // Setting 50 Hz //


  int N=200; //


  double xd=0;
  double yd=0;
  double phid;

  double x1 = -0.7;
  double y1 = 0;

  double x2 = x1;
  double y2 = -0.7;

  double x3 = 0.7;
  double y3 = y2;

  double x4 = x3;
  double y4 = 0.7;

  double x5 = x2;
  double y5 = y4;


  int n=1;

  int rotation_index = 1;

  int i=0;
  int j=0;
  int k=0;

  int index=1;


  while(ros::ok())
  {


    geometry_msgs::PoseStamped cmd_msg;

    // Trajectory : Rectangular


//     From point 1 to point 2
    if(index ==1){
    xd = (double) x1;
    yd = (double) y1;


    i++;
        if(i>N){
            i=0;
            index = 2;
            ros::Duration(1).sleep();
            rotation_index++;
        }

    }

//     From point 2 to point 3
    if(index ==2){
    xd = (double) x2;
    yd = (double) y2;
    i++;


        if(i>N){
            i=0;
            index = 3;
            rotation_index++;
            ros::Duration(1).sleep();
            }

    }

//     From point 3 to point 4
    if(index ==3){
    xd = (double)x3;
    yd = (double)y3;
    i++;


        if(i>N){
            i=0;
            index = 4;
            rotation_index++;
            ros::Duration(1).sleep();
        }
    }


//     From point 4 to point 1
    if(index ==4){
    xd = (double)x4;
    yd = (double)y4;
    i++;


        if(i>N){
            i=0;
            index = 5;
            rotation_index++;
            ros::Duration(1).sleep();
        }
    }

    if(index ==5){
    xd = x5;
    yd = y5;
    i++;


        if(i>N){
            i=0;
            index = 6;
            rotation_index++;
            ros::Duration(1).sleep();
        }
    }

    if(index == 6){
    xd = x2;
    yd = y2;
    i++;


        if(i>N){
            i=0;
            index = 3;
            rotation_index++;
            ros::Duration(1).sleep();
            n++;
        }
    }

    if(n>2){
    xd=0;
    yd=0;
    index = 7;
    }

    cmd_msg.header.frame_id = "map";
    cmd_msg.pose.position.x = xd;
    cmd_msg.pose.position.y = yd;
    cmd_msg.pose.orientation.w = 1;
    cmd_msg.pose.orientation.x = 0;
    cmd_msg.pose.orientation.y = 0;
    cmd_msg.pose.orientation.z = 0;    

    cmd_pub.publish(cmd_msg);

    if(n==3 && xd == 0 && yd == 0){
        break;
    }

    ROS_INFO("%d" ,n);
    ROS_INFO("Pub Msg : %lf %lf",xd,yd);

    loop_rate.sleep();

  }

  return 0;

}

