#include <iostream>
#include "ros/ros.h"
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <sim_control/motorDynamics.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#define _USE_MATH_DEFINES


using namespace std;
using namespace nav_msgs;
using namespace sim_control;


class wheel_odom{


public:


    // Estimated Position and orientation
    double x_est   = 0, y_est   = 0, phi_est = 0;

    // displacement in encoder call back function
    double dx_body = 0, dy_body = 0;

    double v_est[2] = {0,0};
    double q[4] = {1,0,0,0};


    void wheel_odom_pub(){
        
        // Setting pub_odom
        pub_odom = nh_.advertise<Odometry>("/wheel_odom",1);
        
        enc_sub = nh_.subscribe("/input_msg",1,&wheel_odom::enc_call_back,this);
        quat_sub = nh_.subscribe("/gnd_truth",1,&wheel_odom::quat_call_back,this);
        
    }

    // Callback function 2 : quaternion
    void quat_call_back(const Odometry::ConstPtr& quat)
    {   
        q[0] =  quat->pose.pose.orientation.w;
        q[1] =  quat->pose.pose.orientation.x;
        q[2] =  quat->pose.pose.orientation.y;
        q[3] =  quat->pose.pose.orientation.z;
           
    }

    // Callback function : encoder
    void enc_call_back(const motorDynamics::ConstPtr& enc)
    {

        current_time = ros::Time::now().toSec();
        if(last_time==0){
            dt = 0;
        }
        else{
            dt = current_time - last_time;
        }

        ROS_INFO("Time : %lf",dt);
        
        // Getting Encoder data
        
        double w[4];
        w[0] = enc->omega1;
        w[1] = -enc->omega2;
        w[2] = -enc->omega3;
        w[3] = enc->omega4;


        w[0] = (double) w[0];
        w[1] = (double) w[1];
        w[2] = (double) w[2];
        w[3] = (double) w[3];
            /* unit conversion */
        w[0] =  w[0] / gear_ratio * rpm_to_radps;
        w[1] =  w[1] / gear_ratio * rpm_to_radps;
        w[2] =  w[2] / gear_ratio * rpm_to_radps;
        w[3] =  w[3] / gear_ratio * rpm_to_radps;

        v_est[0] = r/4.0 * (w[0] + w[1] + w[2] + w[3]);
        v_est[1] = r/4.0 * (-w[0] + w[1] - w[2] + w[3]);


        dx_body = v_est[0]*dt;
        dy_body = v_est[1]*dt;

        

        last_time = current_time;        


        Odometry est_pos;
        


        est_pos.pose.pose.position.x = dx_body;
        est_pos.pose.pose.position.y = dy_body;
        est_pos.twist.twist.linear.x = v_est[0];
        est_pos.twist.twist.linear.x = v_est[1];
        est_pos.pose.pose.orientation.w = q[0];
        est_pos.pose.pose.orientation.x = q[1];
        est_pos.pose.pose.orientation.y = q[2];
        est_pos.pose.pose.orientation.z = q[3];
                

        pub_odom.publish(est_pos);


    }

private:

    // Declare
    ros::NodeHandle nh_;
    ros::Publisher pub_odom;
    ros::Subscriber quat_sub;
    ros::Subscriber enc_sub;

    // Time in encoder call back function
    double current_time = 0, last_time = 0;
    double dt;
    
    // Time checking - total
    double time_0 = 0, time_1=0, dt_total;

    // Mobile robot wheel radius
    const double r = 0.152/2.0;
    const double gear_ratio = 76.0;
    const double rpm_to_radps = 2.0 * M_PI/60.0;

};