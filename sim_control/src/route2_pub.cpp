#include <math.h>
#include "ros/ros.h"
#include "sim_control/cmdMsg.h"

#define _USE_MATH_DEFINES

/* Main function */

int main(int argc, char **argv)
{
  ros::init(argc,argv,"Trajectory_command");
  ros::NodeHandle nh;

  //100 que size//
  ros::Publisher cmd_pub = nh.advertise<sim_control::cmdMsg>("/cmd_msg",100);

  int freq = 30;

  // Publish rate : 50 Hz //
  ros::Rate loop_rate(freq);

  int N = 200;

  double xd = 0;
  double yd = 0;
  double phid = 0;

  double x1 = -1;
  double y1 = -1;

  double x2 = 1;
  double y2 = -1;

  double x3 = 1;
  double y3 = 0.5;

  double x4 = 0;
  double y4 = 1;

  double x5 = -1;
  double y5 = 1;

  double x6 = -1;
  double y6 = 0.5;

  int n=1;

  int i=0;

  int index=1;



  while(ros::ok())
  {

    sim_control::cmdMsg cmd_msg;


   //Time dt : loop time
//    curr_t = ros::Time::now().toSec();
//    dt = curr_t - prev_t;
//    prev_t = curr_t;

//     From point 1 to point 2
    if(index ==1){
    xd = (double) x1;
    yd = (double) y1;

    i++;
        if(i>N){
            i=0;
            index = 2;
            ros::Duration(1).sleep();
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
            ros::Duration(1).sleep();
        }
    }

    if(index == 6){
    xd = x6;
    yd = y6;
    i++;


        if(i>N){
            i=0;
            index = 7;
            ros::Duration(1).sleep();
            n++;
        }
    }

    cmd_msg.xd = xd;
    cmd_msg.yd = yd;
    cmd_msg.phid = phid;

    cmd_pub.publish(cmd_msg);

    if(index>0 && index<7){
    ROS_INFO("Pub Msg : %lf %lf %lf",cmd_msg.xd,cmd_msg.yd,cmd_msg.phid);

    }

    if(index == 7)
    break;
    loop_rate.sleep();


  }
  return 0;
}
