#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#define _USE_MATH_DEFINES
#include <math.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_cmd_circle_fix");
    ros::NodeHandle n;
    ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Rate loop_rate(100);

    int count = 0;
    double r = 0.6;
    double phi = 0;
    double phi_dot = 0.25/r;
    double dt = 0.01;
    int end = 16*M_PI/dt/phi_dot+1;
    while (ros::ok())
    {
        geometry_msgs::Twist msg;

        msg.linear.x = -r*sin(phi)*phi_dot;
        msg.linear.y = r*cos(phi)*phi_dot;
        msg.angular.z = 0;

        phi += phi_dot*dt;
        ++count;
        if (count == end){
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.angular.z = 0;
            command_pub.publish(msg);
            ROS_INFO("Vx: %f, Vy: %f, phi_dot: %f", msg.linear.x, msg.linear.y, msg.angular.z);
            break;
        }

        ROS_INFO("Vx: %f, Vy: %f, phi_dot: %f", msg.linear.x, msg.linear.y, msg.angular.z);

        command_pub.publish(msg);

        loop_rate.sleep();
    }


    return 0;
}
