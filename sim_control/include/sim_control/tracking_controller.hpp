/** Code contents
1. Recieve desired velocity, position, and orientation of the robot


2. Commend velocity of the robot

Eqn 1
ddrdt2_cmd = ddrdt2_des + K_vel * (drdt_des - drdt_robot) + K_pos * (r_des - r_robot)
where r = [x y phi]'

By integrating ddrdt2_cmd,
get commend velocity of the robot : drdt_cmd

3. motor velocity and Clampping
Applying the inverse kinematics...
obtain the motor velocities.
Find out the maximum motor velocity and then clampping

**/

#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>

#include <teb_local_planner/FeedbackMsg.h>
#include <nav_msgs/Odometry.h>

#include <sim_control/motorDynamics.h>

#define _USE_MATH_DEFINES

using namespace teb_local_planner;
using namespace nav_msgs;

struct motor_vel{
    double w[4];
};


class tracking_controller{


    public:
    // Robot index
    uint16_t robot_index = 0;

    // Desired trajectory : acc, vel, position, orientation
    double ax_des = 0, ay_des = 0, aphi_des = 0;
    double vx_des = 0, vy_des = 0, vphi_des = 0;
    double x_des = 0, y_des = 0;
    double qw_des = 1, qx_des = 0, qy_des = 0, qz_des = 0;
    double phi_des = 0;

    // Robot state : vel, position, orientation
    double vx_robot = 0, vy_robot = 0, vphi_robot = 0;
    double x_robot = 0, y_robot = 0;
    double qw = 1, qx = 0, qy = 0, qz = 0;
    double phi_robot = 0;

    // Time info
    double current_time = 0;
    double last_time = 0;
    double dt = 0;

    // Controller Gain
    double K_v[3] = {1,1,1};
    double K_p[3] = {0,0,0};
    // Error : linear vel, angular vel, position, orientation
    double vx_err = 0, vy_err = 0, vphi_err = 0;
    double x_err = 0, y_err = 0, phi_err = 0;

    // Commend velocity in the global frame
    double ax_cmd = 0, ay_cmd = 0, aphi_cmd = 0;
    double vx_cmd = 0, vy_cmd = 0, vphi_cmd = 0; 

    // Commend velocity in the robot frame
    double vx_cmd_robot = 0, vy_cmd_robot = 0, vphi_cmd_robot = 0;

    // Clamping info : Limited motor vel = 6000 RPM
    const int motor_vel_lim = 6000; 

    // Commend motor velocity
    motor_vel cmd_motor_vel;

    // Specification of robot
    const double wheel_radious = 0.1520/2.0;
    const double r = wheel_radious;
    const double gear_ratio = 74.5;
    const double radps_to_rpm = 60.0/2.0/M_PI;
    const double rpm_to_radps = 2.0*M_PI/60.0;
    const double l_a = 0.2680;
    const double l_b = 0.2000;
    const double l = l_a + l_b;

    // Publisher Declaration
    void cmd_vel_pub_setting(){
        publisher_cmd_vel = nh_.advertise<sim_control::motorDynamics>("/input_msg",1);
    }


    // Subscriber Declaration
    void sub_traj(){
        // Subscribe
        subscriber_state = nh_.subscribe("/odom",1,&tracking_controller::callback_state,this);
        subscriber_trajectory = nh_.subscribe("/move_base/TebLocalPlannerROS/teb_feedback",1,&tracking_controller::callback_traj,this);

    }

    // Publish commend motor velocity
    void cmd_vel_pub(){
        
        // Loop time
        current_time = ros::Time::now().toSec();
        if(last_time == 0)
        {
            dt = 0;
        }

        dt = current_time - last_time;

        last_time = current_time;

        // Inverse Kinematics
        cmd_motor_vel = inverse_kinematics();

        if(vx_cmd_robot == 0 && vy_cmd_robot == 0 && vphi_cmd_robot == 0){
            cmd_motor_vel.w[0] = 0;
            cmd_motor_vel.w[1] = 0;
            cmd_motor_vel.w[2] = 0;
            cmd_motor_vel.w[3] = 0;
            
        }

        sim_control::motorDynamics motor_vel;
            // Unit conversion
            cmd_motor_vel.w[0] = gear_ratio*radps_to_rpm*cmd_motor_vel.w[0];
            cmd_motor_vel.w[1] = -gear_ratio*radps_to_rpm*cmd_motor_vel.w[1];
            cmd_motor_vel.w[2] = -gear_ratio*radps_to_rpm*cmd_motor_vel.w[2];
            cmd_motor_vel.w[3] = gear_ratio*radps_to_rpm*cmd_motor_vel.w[3];
                         
                         
            cmd_motor_vel = clamping(cmd_motor_vel);

            motor_vel.omega1 = (int) cmd_motor_vel.w[0];
            motor_vel.omega2 = (int) cmd_motor_vel.w[1];
            motor_vel.omega3 = (int) cmd_motor_vel.w[2];
            motor_vel.omega4 = (int) cmd_motor_vel.w[3];
        publisher_cmd_vel.publish(motor_vel);

        // Vel error
        vx_err   = vx_des - vx_robot;
        vy_err   = vy_des - vy_robot;
        vphi_err = vphi_des - vphi_robot;

        //ROS_INFO("vx_des : %lf vy_des : %lf vphi_des %lf",vx_des,vy_des,vphi_des);

        // Position and orientation error
        x_err    = (x_des - x_robot)*cos(phi_robot) - (y_des - y_robot)*sin(phi_robot);
        y_err    = (x_des - x_robot)*sin(phi_robot) + (y_des - y_robot)*cos(phi_robot);
        phi_err  = phi_des - phi_robot;

        ax_cmd   = ax_des + K_v[1] * vx_err + K_p[1] * x_err;
        ay_cmd   = ay_des + K_v[2] * vy_err + K_p[2] * y_err;
        aphi_cmd = aphi_des + K_v[3] * vphi_err + K_p[3] * phi_err;
        
        /*
        vx_cmd   = vx_cmd + ax_cmd*dt;
        vy_cmd   = vy_cmd + ay_cmd*dt;
        vphi_cmd = vphi_cmd + aphi_cmd*dt;
        */

       vx_cmd = K_v[1] * vx_err + K_p[1] * x_err;
       vy_cmd = K_v[2] * vy_err + K_p[1] * y_err;
       vphi_cmd = K_v[2] * vphi_err + K_p[1] * phi_err;
    
       vx_cmd_robot = vx_cmd;
       vy_cmd_robot = vy_cmd;
       vphi_cmd_robot = vphi_cmd;
    
    }

    motor_vel inverse_kinematics(){
        motor_vel m;
        m.w[0] = (int) 1.0/r * vx_cmd_robot - 1.0/r * vy_cmd_robot - l/r * vphi_cmd_robot;
        m.w[1] = (int) 1.0/r * vx_cmd_robot + 1.0/r * vy_cmd_robot + l/r * vphi_cmd_robot;
        m.w[2] = (int) 1.0/r * vx_cmd_robot - 1.0/r * vy_cmd_robot + l/r * vphi_cmd_robot;
        m.w[3] = (int) 1.0/r * vx_cmd_robot + 1.0/r * vy_cmd_robot - l/r * vphi_cmd_robot;

        return m;
    }

    motor_vel clamping(motor_vel clamped_motor_vel){

        double arr[4] = {clamped_motor_vel.w[0],clamped_motor_vel.w[1],clamped_motor_vel.w[2],clamped_motor_vel.w[3]};
        double motor_vel_max = 0;
        std::sort(arr,arr+4,std::greater<double>());
        motor_vel_max = arr[0];
        
        if(fabs(motor_vel_max)>motor_vel_lim){
            clamped_motor_vel.w[0] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[0];
            clamped_motor_vel.w[1] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[1];
            clamped_motor_vel.w[2] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[2];
            clamped_motor_vel.w[3] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[3];
        }
        
        return clamped_motor_vel;
    }



    // Callback function 1 : Trajectory from planner
    void callback_traj(const FeedbackMsg::ConstPtr& traj_msg){
        
        // Robot index
        robot_index = traj_msg -> selected_trajectory_idx;
        int traj_index_ = traj_msg->trajectories[robot_index].trajectory.size();
        traj_index_ = traj_index_-1;
        int traj_index = 1;
        
        // Desired acceleration
        ax_des =traj_msg->trajectories[robot_index].trajectory[traj_index].acceleration.linear.x;
        ay_des =traj_msg->trajectories[robot_index].trajectory[traj_index].acceleration.linear.y;
        aphi_des =traj_msg->trajectories[robot_index].trajectory[traj_index].acceleration.angular.z;

        ROS_INFO("ax_des : %lf ay_des : %lf",ax_des,ay_des);        

        // Desired velocity
        vx_des = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.linear.x;
        vy_des = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.linear.y;
        vphi_des = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.angular.z;

        ROS_INFO("x_des : %lf y_des : %lf phi_des : %lf",x_des,y_des,phi_des);

        // Desired position
        x_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.position.x;
        y_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.position.y;
        
        // Desired orientation
        qw_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.w;
        qx_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.x;
        qy_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.y;
        qz_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.z;

        // Conversion from Quaternion to euler angle
        double siny_cosp = 2.0 * (qw_des*qz_des + qx_des*qy_des);
        double cosy_cosp = 1 - 2.0 * (qy_des*qy_des + qz_des*qz_des);
        phi_des = atan2(siny_cosp,cosy_cosp);
    }

    // Callback function 2 : State
    void callback_state(const Odometry::ConstPtr& state_msg){
        
        // Robot Velocity 
        vx_robot = state_msg -> twist.twist.linear.x;
        vy_robot = state_msg -> twist.twist.linear.y;
        vphi_robot = state_msg -> twist.twist.angular.z;

        // Robot position
        x_robot = state_msg -> pose.pose.position.x;
        y_robot = state_msg -> pose.pose.position.y;
        
        // Robot orientation
        qw = state_msg -> pose.pose.orientation.w;
        qx = state_msg -> pose.pose.orientation.x;        
        qy = state_msg -> pose.pose.orientation.y;
        qz = state_msg -> pose.pose.orientation.z;

        // Conversion from Quaternion to euler angle
        double siny_cosp = 2.0 * (qw*qz + qx*qy);
        double cosy_cosp = 1 - 2.0 * (qy*qy + qz*qz);
        phi_robot = atan2(siny_cosp,cosy_cosp);

    }



    private:

    ros::NodeHandle nh_;
    ros::Subscriber subscriber_trajectory;
    ros::Subscriber subscriber_state;
    ros::Publisher publisher_cmd_vel;
            
    

};