#include "ros/ros.h"
#include <math.h>

#include <nav_msgs/Odometry.h>
#include "epos_tutorial/realVel.h"
#include "vehicle_control/timeLoop.h"
#define _USE_MATH_DEFINES

using namespace nav_msgs;
using namespace epos_tutorial;
using namespace vehicle_control;

class wheel_odom{

public:

	// Global variables : Estimated position and orientation
	double x_est = 0, y_est = 0, phi_est = 0;
	double q[4] = {1,0,0,0};

	double w[4] = {0,0,0,0};

	// Time info.
	double current_time = 0, last_time = 0;
	double dt = 0;
	
	// Displacement in encoder call back function
	double dx_body = 0, dy_body = 0;

	// Estimated velocity
	double vx_est = 0;
	double vy_est = 0;
 
	// publish member function
	void wheel_odom_sub(){

		// Setting pub odom
		pub_odom = nh_.advertise<Odometry>("/wheel_odom",1);
		//pub_motors = nh_.advertise<realVel>("/motors",1);
		//pub_time = nh_.advertise<timeLoop>("/time",1);
        quat_sub = nh_.subscribe("/camera/odom/sample",1,&wheel_odom::quat_call_back,this);
		enc_sub = nh_.subscribe("/measure",1,&wheel_odom::enc_call_back,this);
	
	}


	// call back member function 1 : quaternion
    void quat_call_back(const Odometry::ConstPtr& quat)
	{
		q[0] = quat->pose.pose.orientation.w;
		q[1] = quat->pose.pose.orientation.x;
		q[2] = quat->pose.pose.orientation.y;
		q[3] = quat->pose.pose.orientation.z;
	
	}


	// call back member function 2 : encoder
	void enc_call_back(const realVel::ConstPtr& enc)
	{

	
	//timeLoop time;
	//time.t = current_time;
	//pub_time.publish(time);



	w[0] = enc->realVel[0];
	w[1] = enc->realVel[1];
	w[2] = enc->realVel[2];
	w[3] = enc->realVel[3];
/*
	realVel motors;
	motors.realVel[0] = w[0];
	motors.realVel[1] = w[1];
	motors.realVel[2] = w[2];
	motors.realVel[3] = w[3];
	pub_motors.publish(motors);
*/
	w[0] =  w[0];
	w[1] = -w[1];
	w[2] = -w[2];
	w[3] =  w[3];

	//ROS_INFO("W1 W2 W3 W4 dt: %f %f %f %f %f",w[0],w[1],w[2],w[3],dt);

	w[0] = w[0] / gear_ratio * rpm_to_radps;
	w[1] = w[1] / gear_ratio * rpm_to_radps;
	w[2] = w[2] / gear_ratio * rpm_to_radps;
	w[3] = w[3] / gear_ratio * rpm_to_radps;
	
	//ROS_INFO("x : %lf  y : %lf phi_est : %lf",x_est,y_est,phi_est);


	}

	void wheel_odom_pub(){

	current_time = ros::Time::now().toSec();
	if(last_time == 0)
	{
		dt = 0;
	}
	else
	{
		dt = current_time - last_time;
	}

	vx_est = r/4.0*(w[0]+w[1]+w[2]+w[3]);
    vy_est = r/4.0*(-w[0]+w[1]-w[2]+w[3]);

	dx_body = vx_est*dt;
	dy_body = vy_est*dt;

    phi_est = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));

	x_est = x_est + dx_body*cos(phi_est) - dy_body*sin(phi_est);
	y_est = y_est + dy_body*sin(phi_est) + dy_body*cos(phi_est);

	last_time = current_time;

	Odometry est_pos;
		est_pos.pose.pose.position.x = x_est;
		est_pos.pose.pose.position.y = y_est;

		est_pos.twist.twist.linear.x = vx_est;
		est_pos.twist.twist.linear.y = vy_est;
			
		est_pos.pose.pose.orientation.w = q[0];
		est_pos.pose.pose.orientation.x = q[1];
		est_pos.pose.pose.orientation.y = q[2];
		est_pos.pose.pose.orientation.z = q[3];
	pub_odom.publish(est_pos);

	}


private:

	// Declare ros configuration
	ros::NodeHandle nh_;
	ros::Publisher pub_odom;
	ros::Publisher pub_motors;
	ros::Publisher pub_time;
	ros::Subscriber quat_sub;
	ros::Subscriber enc_sub;

	// Mobile robot dimension
	const double r = 0.152/2.0;
	const double gear_ratio = 74.5;
	const double rpm_to_radps = 2.0 * M_PI/60.0;

};