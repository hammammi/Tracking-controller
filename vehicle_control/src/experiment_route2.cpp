#include "ros/ros.h"
#include "vehicle_control/commendMsg.h"
#include <math.h>


int main(int argc, char **argv)
{
  ros::init(argc,argv,"cmd_publisher1");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub=nh.advertise<vehicle_control::commendMsg>("/ns1/cmd_msg",100); //100 que size//

  int hz1 = 30;
//  int hz2 = 20; // 0.30m/s

  ros::Rate loop_rate(hz1); // Setting 50 Hz //


  int N=200; //


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


    vehicle_control::commendMsg cmd_msg;

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

    ROS_INFO("Pub Msg : %lf %lf %lf",cmd_msg.xd,cmd_msg.yd,cmd_msg.phid);
    loop_rate.sleep();

    if(index == 7)
    break;

  }

  return 0;

}

