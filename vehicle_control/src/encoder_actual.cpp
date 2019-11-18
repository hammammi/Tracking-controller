#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "epos_tutorial/realVel.h"
#include "vehicle_control/positionMsg.h"

#define Pi 3.14159265358979323846

/* Ecoder data variables (unit : rpm) */
double w[4];

/* IMU data variables*/
double a[3];
double gyro_yaw;
double q[4];


double dx;
double dy;
double dt;

/* Time variables */
int i_time = 0;            // Time initialization index
double curr_time=0;
double last_time=0;

/* Angle variables */
double phi=0;
double phi0=0;
int index_angle = 1;    // Angle initialization index

/* Wheel Specification */
double wheel_diameter = 0.1520;
double wheel_radius = wheel_diameter / 2.0;
double wheel_separation_a = 0.2600;
double wheel_separation_b = 0.2680;
double l = wheel_separation_a + wheel_separation_b;

/* Motor specification */
double gear_ratio = 74.5;
/* Conversion ratio */
double rpm_to_radps = 2.0 * Pi / 60;

void encoderCallback(const epos_tutorial::realVel::ConstPtr& motors){

    /* Time initialization */
    if (i_time==0){
        last_time = ros::Time::now().toSec();
    }
    i_time += 1;

    /* Get motors' velocities */
    w[0] = motors -> realVel[0];
    w[1] = motors -> realVel[1];
    w[2] = motors -> realVel[2];
    w[3] = motors -> realVel[3];

    w[0] = w[0];
    w[1] = -w[1];
    w[2] = -w[2];
    w[3] = w[3];

    w[0] = (double) w[0];
    w[1] = (double) w[1];
    w[2] = (double) w[2];
    w[3] = (double) w[3];

    /* unit conversion */
    w[0] =  w[0] / gear_ratio * rpm_to_radps;
    w[1] =  w[1] / gear_ratio * rpm_to_radps;
    w[2] =  w[2] / gear_ratio * rpm_to_radps;
    w[3] =  w[3] / gear_ratio * rpm_to_radps;

    curr_time = ros::Time::now().toSec();
    dt = curr_time - last_time;

    dx = wheel_radius/4 * (w[0]+w[1]+w[2]+w[3])*dt;
    dy = wheel_radius/4 * (-w[0]+w[1]-w[2]+w[3])*dt;


    last_time = curr_time;
//    ROS_INFO("dy: %lf, dt: %lf", dy, dt);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu){

    a[0]      = imu -> linear_acceleration.x;
    a[1]      = imu -> linear_acceleration.y;

    q[0]     = imu -> orientation.w;
    q[1]     = imu -> orientation.x;
    q[2]     = imu -> orientation.y;
    q[3]     = imu -> orientation.z;
    gyro_yaw = imu -> angular_velocity.z;

    /* Initialization of yaw */
    if(index_angle==1 && phi!=0){
        phi0 = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));        // phi0 : initial yaw
        index_angle=2;
    }
    phi = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3])) - phi0;
}

int main(int argc,char** argv){

  ros::init(argc,argv,"state_publisher");
  ros::NodeHandle n;

  /* Subscriber*/
  ros::Subscriber enc_sub = n.subscribe("/measure",1,encoderCallback);
  //ros::Subscriber imu_sub = n.subscribe("/imu",1,imuCallback);


  /* Publisher */
  ros::Publisher odom_pub = n.advertise<vehicle_control::positionMsg>("/odom",1);

  ros::Rate loop_rate(500);

  /* Declare variables to store position data */
  double x=0;
  double y=0;
  double dphi=0;

  while(ros::ok()){


//   ROS_INFO("phi : %lf  phi0 : %lf  index : %d",phi,phi0,index_angle);
    ROS_INFO("w1 : %lf w2 : %lf w3 : %lf w4 : %lf ",w[0],w[1],w[2],w[3]);
    ROS_INFO("dy : %lf phi : %lf", dy, phi);
    x = x + dx;
    y = y + dy;

    vehicle_control::positionMsg pose;

    double cy = 1;
    double sy = 0;
    double cr = 1;
    double sr = 0;
    double cp = cos(phi * 0.5);
    double sp = sin(phi * 0.5);

    double qw = cy * cr * cp + sy * sr * sp;
    double qx = cy * sr * cp - sy * cr * sp;
    double qy = cy * cr * sp + sy * sr * cp;
    double qz = sy * cr * cp - cy * sr * sp;

    pose.x = x;
    pose.y = y;
    pose.qx = qx;
    pose.qy = qy;
    pose.qz = qz;
    pose.qw = qw;


    odom_pub.publish(pose);

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
