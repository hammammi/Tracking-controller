#include "ros/ros.h"
#include "vehicle_control/commendMsg.h"
#include <math.h>


int main(int argc, char **argv)
{
  ros::init(argc,argv,"Rectangle_exp");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub=nh.advertise<vehicle_control::commendMsg>("/ns1/cmd_msg",100); //100 que size//

  int hz1 = 25;

  ros::Rate loop_rate(hz1); // Setting 50 Hz //


  int N=150; //


  double xd=0;
  double yd=0;
  double phid;

  double x1 = -0.6;
  double y1 = 0;

  double x2 = 0;
  double y2 = 0.6;

  double x3 = 0.6;
  double y3 = 0;

  double x4 = 0;
  double y4 = -0.6;

  int n=1;

  int rotation_index = 1;

  int i=0;
  int j=0;
  int k=0;

  int index=1;


  while(ros::ok())
  {


    vehicle_control::commendMsg cmd_msg;

    // Trajectory : Rectangular


    if(index ==1){
    xd = (double) x1;
    yd = (double) y1;

    i++;
        if(i>N){
            i=0;
            index = 2;
            rotation_index++;
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
            rotation_index++;

                        ros::Duration(1).sleep();
            }

    }

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
    xd = x1;
    yd = y1;
    i++;

        if(i>N){
            i=0;
            index = 2;
            rotation_index++;

                        ros::Duration(1).sleep();
            n++;
        }
    }

    if(n>2){
    xd=0;
    yd=0;
    index = 3;
    }


    cmd_msg.xd = xd;
    cmd_msg.yd = yd;
    cmd_msg.phid = phid;


    cmd_pub.publish(cmd_msg);


    ROS_INFO("%d" ,n);
    ROS_INFO("Pub Msg : %lf %lf %lf",cmd_msg.xd,cmd_msg.yd,cmd_msg.phid);

    loop_rate.sleep();

  }

  return 0;

}

