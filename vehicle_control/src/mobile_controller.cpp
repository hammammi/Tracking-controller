#include <math.h>
#include "ros/ros.h"
#include "vehicle_control/commendMsg.h"
#include "mobile_control/motorMsg.h"
#include "vehicle_control/positionMsg.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#define PI 3.14159265358979323846

// From commend_msg //
double pose_des[3];

// From odometry msg //
double pose_est[2];
double vel_est[3];
double quat_est[4];

double phi_est;


void cmdCallback(const vehicle_control::commendMsg::ConstPtr& cmd_msg)
{
    pose_des[0]   = cmd_msg->xd;
    pose_des[1]   = cmd_msg->yd;
    pose_des[2]   = cmd_msg->phid;
}

void odomCallback(const vehicle_control::positionMsg::ConstPtr& msg)
{
    pose_est[0] = msg->x;
    pose_est[1] = msg->y;

    quat_est[0] = msg->qx;
    quat_est[1] = msg->qy;
    quat_est[2] = msg->qz;
    quat_est[3] = msg->qw;
}


/* Main function */

int main(int argc, char **argv)
{
  ros::init(argc,argv,"vehicle_control");
  ros::NodeHandle nh;

  ros::Publisher publ_input = nh.advertise<mobile_control::motorMsg>("/input_msg",1000);
  ros::Publisher ctrl_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",100);

  ros::Subscriber sub1 = nh.subscribe("/ns1/cmd_msg",1000,cmdCallback);
  ros::Subscriber sub2 = nh.subscribe("/odom",1000,odomCallback);

  // Publish rate : 100Hz //
  ros::Rate loop_rate(100);


/** Initialization for controller **/
  // reference {prev, current} //
  double x_d[2]      = {0, 0};
  double y_d[2]      = {0, 0};
  double phi_d[2]    = {0, 0};
  
  // Feedback data //
  double x_est[2]    = {0, 0};
  double y_est[2]    = {0, 0};
  double phi_est[2]  = {0, 0};
  double multiturn_phi_est[2] = {0,0};
  double n_phi=0;

  double vx_est[2]   = {0, 0};
  double vy_est[2]   = {0, 0};
  double dphi_est[2] = {0, 0};

  double x_quat   = 0;
  double y_quat   = 0;
  double z_quat   = 0;
  double w_quat   = 0;

  double angle[2] = {0,0};
  double multiturn_angle[2] = {0,0};
  double n_angle=0;

  // Output velocity //
  double w_curr[4] = {0,0,0,0};
  double w_prev[4] = {0,0,0,0};


  // Input(Velocity) //
  double del_s[2]={0,0};
  double vel_linear;

  double u_x = 0;
  double u_y = 0;
  double u_p = 0;

  // Motor speed in rad/sec - initialization {prev,curr}//
  double wheel_speed_lf = 0;
  double wheel_speed_rf = 0;
  double wheel_speed_lb = 0;
  double wheel_speed_rb = 0;

  // Motor speed in RPM - initialization //

  int w1 = 0;
  int w2 = 0;
  int w3 = 0;
  int w4 = 0;

/** Controller gains Setting **/

  // P control //
  double kp_s = 1.0;
  double kp_phi = 10.0;

// Linear velocity : 0.2m/s , dphidt constraint :  10 deg/sec ( 0.174 rad/sec ), and scale factor  //
  double v_lim       = 0.6;
  double dphidt_lim  = 0.174;

/* Wheel specification - unit: meter */
  double wheel_diameter = 0.152;
  double wheel_radius = wheel_diameter / 2.0;
  double wheel_separation_a = 0.2600;
  double wheel_separation_b = 0.2680;
  double l = wheel_separation_a + wheel_separation_b;


// Motor specification and unit conversion factors //

  int gear_ratio = 72;
  double radps_to_rpm = 60.0/2.0/PI;        // rads_to_rpm : rad/sec --> rpm
  double rpm_to_radps = 2.0 * PI / 60;      // rpm_to_radps : rev/min --> rad/sec

  while(ros::ok())
  {

    mobile_control::motorMsg input_msg;
    geometry_msgs::Twist cmd_vel;

 
    phi_est[1] =  atan2(2.0 * (quat_est[3] * quat_est[2] + quat_est[0] * quat_est[1])
    ,1.0-2.0 * (quat_est[1] * quat_est[1] + quat_est[2] * quat_est[2]));


    /* Extended estimated yaw */
    if (phi_est[0]>(PI/2.0) && phi_est[1]<-(PI/2.0)){
    multiturn_phi_est[1] = phi_est[1] + PI + (2.0*n_phi+1)*PI;
    n_phi = n_phi + 1;
    }

    if (phi_est[0]<(-PI/2.0) && phi_est[1]>(PI/2.0)){
    multiturn_phi_est[1] = phi_est[1] - PI + (2.0*n_phi+1)*PI;
    n_phi = n_phi - 1;
    }

    if (phi_est[0]*phi_est[1]>=0){
    multiturn_phi_est[1] = phi_est[1] + 2.0*n_phi*PI;
    }

    if (phi_est[0]*phi_est[1]<=0){
    multiturn_phi_est[1] = phi_est[1] + 2.0*n_phi*PI;
    }


    x_est[1] = pose_est[0];
    y_est[1] = pose_est[1];

    // Current values from reference  phi --> rad//
    x_d[1]      = pose_des[0];
    y_d[1]      = pose_des[1];
    phi_d[1]    = pose_des[2];


    angle[1] = atan2(y_d[1]-y_est[1],x_d[1]-x_est[1]);

    /* Extended input angle */
    if (angle[0]>(PI/2.0) && angle[1]<-(PI/2.0)){
    multiturn_angle[1] = angle[1] + PI + (2.0*n_angle+1)*PI;
    n_angle = n_angle + 1;
    }

    if (angle[0]<(-PI/2.0) && angle[1]>(PI/2.0)){
    multiturn_angle[1] = angle[1] - PI + (2.0*n_angle+1)*PI;
    n_angle = n_angle - 1;
    }

    if (angle[0]*angle[1]>=0){
    multiturn_angle[1] = angle[1] + 2.0*n_angle*PI;
    }


    if (angle[0]*angle[1]<=0){
    multiturn_angle[1] = angle[1] + 2.0*n_angle*PI;
    }

   /* PID control */

  del_s[1] = sqrt( (x_d[1] - x_est[1]) * (x_d[1] - x_est[1]) + (y_d[1] - y_est[1]) * (y_d[1] - y_est[1]));

  vel_linear = kp_s * del_s[1]; //+ kd_s * ( del_s[1] - del_s[0] ) / dt + ki_s * Is;

  u_p =  kp_phi * (phi_d[1]-multiturn_phi_est[1]);


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

    u_x = vel_linear * cos(angle[1]-phi_est[1]);
    u_y = vel_linear * sin(angle[1]-phi_est[1]);


    // Inverse Kinematics for motor input (uint : RPM) //
    w1 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x - u_y - l * u_p);
    w2 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x + u_y + l * u_p);
    w3 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x + u_y - l * u_p);
    w4 = (int) radps_to_rpm * 1.0 / wheel_radius * ( u_x - u_y + l * u_p);

    // Model - motor's velocity //
    wheel_speed_lf = (double) w1 * rpm_to_radps;
    wheel_speed_rf = (double) w2 * rpm_to_radps;
    wheel_speed_lb = (double) w3 * rpm_to_radps;
    wheel_speed_rb = (double) w4 * rpm_to_radps;

    if(u_x * u_x + u_y * u_y + u_p * u_p < 0.01*0.01){
        w1 = 0;
        w2 = 0;
        w3 = 0;
        w4 = 0;

        u_x = 0;
        u_y = 0;
        u_p = 0;
    }


    // Forward Kinematics - Simulation part//
    u_x = wheel_radius / 4.0 * (wheel_speed_lf + wheel_speed_rf + wheel_speed_lb
        + wheel_speed_rb);

    u_y = wheel_radius / 4.0 * (-wheel_speed_lf + wheel_speed_rf + wheel_speed_lb
        - wheel_speed_rb);

    u_p = wheel_radius / ( 4.0 * l) * (-wheel_speed_lf + wheel_speed_rf -wheel_speed_lb + wheel_speed_rb);

    cmd_vel.linear.x  = u_x;
    cmd_vel.linear.y  = u_y;
    cmd_vel.angular.z = u_p;

    input_msg.omega1 = w1 * gear_ratio;
    input_msg.omega2 = -w2 * gear_ratio;
    input_msg.omega3 = w3 * gear_ratio;
    input_msg.omega4 = -w4 * gear_ratio;


    ctrl_pub.publish(cmd_vel);
    publ_input.publish(input_msg);

    ros::spinOnce();
    loop_rate.sleep();

        // Last values from reference  phi --> rad//
    x_d[0]      = x_d[1];
    y_d[0]      = y_d[1];
    del_s[0]    = del_s[1];
    phi_d[0]    = phi_d[1];

    // Last values from Odometry  //

    x_est[0]    = x_est[1];
    y_est[0]    = y_est[1];
    phi_est[0]  = phi_est[1];


  }
  return 0;
}
