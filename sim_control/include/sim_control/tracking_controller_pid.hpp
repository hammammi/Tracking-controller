/** Code contents

PID controller

**/
#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>

#include <teb_local_planner/FeedbackMsg.h>
#include <teb_local_planner/TrajectoryPointMsg.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mobile_control/motorMsg.h>
#include <sim_control/desiredMsg.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


#define _USE_MATH_DEFINES

using namespace teb_local_planner;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace sim_control;

struct motor_vel{
    double w[4];
};


class tracking_controller{


    public:

    bool init_pos = true;
    // Robot index
    uint16_t robot_index = 0;

    // size of trajectory
    int size_of_traj;


    // Desired trajectory : acceleration, vel, position, orientation
    double ax_des = 0, ay_des = 0, aphi_des = 0;
    double vx_des = 0, vy_des = 0, vphi_des = 0;
    double x_des = 0, y_des = 0;
    double qw_des = 1, qx_des = 0, qy_des = 0, qz_des = 0;
    // phi_des[0] : previous desired yaw angle
    // phi_des[1] : current desired yaw angle
    double phi_des[2] = {0,0};

    int index_multi_turn_phi_des = 0;
    double multi_turn_phi_des = 0;

    double x_goal = 0, y_goal = 0;
    double phi_goal = 0;
    int index_multi_turn_phi_goal = 0;
    double multi_turn_phi_goal = 0;

    // Robot State : velocity, position, and orientation
    double vx_robot = 0, vy_robot = 0, vphi_robot = 0;
    double x_robot = 0, y_robot = 0;
    double qw = 1, qx = 0, qy = 0, qz = 0;
    // phi_robot[0] : previous robot yaw angle
    // phi_robot[1] : current robot yaw angle
    double phi_robot[2] = {0,0};
    int index_multi_turn_phi_robot = 0;
    double multi_turn_phi_robot = 0;

    // Controller Gain
    const double K_p[3] = {10.0,10.0,10.0};
    const double K_i[3] = {0.0,0.0,0.0};
    const double K_d[3] = {1.0,1.0,1.0};

    // P error, I error, D error
    double x_err = 0, y_err = 0, phi_err = 0;
    double vx_accumul = 0, vy_accumul = 0, vphi_accumul = 0;
    double vx_err = 0, vy_err = 0, vphi_err = 0;


    // Commend velocity in the global frame
    double ax_cmd = 0, ay_cmd = 0, aphi_cmd = 0;

    // Commend Velocity
    double vx_cmd = 0, vy_cmd = 0, vphi_cmd = 0;

    // Clamping info : Limited motor vel = 6000 RPM
    const int motor_vel_lim = 3000; 

    // Commend motor velocity
    motor_vel cmd_motor_vel;

    // Specification of robot
    const double wheel_radious = 0.1520/2.0;
    const double r = wheel_radious;
    const double gear_ratio = 74.5;
    const double radps_to_rpm = 60.0/2.0/M_PI;
    const double rpm_to_radps = 2.0*M_PI/60.0;
    const double l_a = 0.2170;
    const double l_b = 0.1687;
    const double l = l_a + l_b;

    // Publisher Declaration
    void cmd_vel_pub_setting(){
        publisher_desired_traj = nh_.advertise<nav_msgs::Odometry>("/des_traj",1);
        publisher_cmd_vel = nh_.advertise<mobile_control::motorMsg>("/input_msg",1);
        publisher_slam_pose = nh_.advertise<nav_msgs::Odometry>("/slam_pose",1);
        publisher_error = nh_.advertise<nav_msgs::Odometry>("/error_msg",1);
    }


    // Subscriber Declaration
    void subscriber_declaration(){
        // Subscribe
        subscriber_state = nh_.subscribe("/wheel_odom",1,&tracking_controller::callback_state,this);
        subscriber_trajectory = nh_.subscribe("/move_base/TebLocalPlannerROS/teb_feedback",1,&tracking_controller::callback_traj,this);
        subscriber_goal = nh_.subscribe("/move_base_simple/goal",1,&tracking_controller::callback_goal,this);
    }

    // Publish commend motor velocity
    void cmd_vel_pub(){


        curr_time = ros::Time::now().toSec();
        dt = curr_time - last_time;


        tf::StampedTransform transform;

        try{
            listener_.lookupTransform("map","base_footprint",ros::Time(0),transform);
            
            x_robot = transform.getOrigin().x();
            y_robot = transform.getOrigin().y();

            //ROS_INFO("x_robot y_robot (SLAM) : %lf, %lf",x_robot,y_robot);
            ROS_INFO("vx robot vy_robot (SLAM) : %lf, %lf",vx_robot, vy_robot);


            tf::Quaternion quat = transform.getRotation();
            phi_robot[1] = tf::getYaw(quat);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        // 1. Tracking Controller

        // Position and orientation error
        x_err    = (x_des - x_robot)*cos(phi_robot[1]) - (y_des - y_robot)*sin(phi_robot[1]);
        y_err    = (x_des - x_robot)*sin(phi_robot[1]) + (y_des - y_robot)*cos(phi_robot[1]);
        //phi_err  = multi_turn_phi_des - multi_turn_phi_robot;
        phi_err = phi_goal - phi_robot[1];

                // Acceleration - Velocity error

        vx_err = vx_des - vx_robot;
        vy_err = vy_des - vy_robot;
        vphi_err = vphi_des - vphi_robot;

        ax_cmd   = ax_des + K_i[0] * vx_err;
        ay_cmd   = ay_des + K_i[1] * vy_err;
        aphi_cmd = aphi_des + K_i[2] * vphi_err;


        vx_accumul += ax_cmd * dt;
        vy_accumul += ay_cmd * dt;
        vphi_accumul += aphi_cmd * dt;

        // PID Control
        vx_cmd = K_p[0] * x_err + vx_accumul + K_d[0] * vx_err;
        vy_cmd = K_p[1] * y_err + vy_accumul + K_d[1] * vy_err;
        vphi_cmd = K_p[2] * phi_err + vphi_accumul + K_d[2] * vphi_err;
       
        // Inverse Kinematics
        cmd_motor_vel = inverse_kinematics();

        mobile_control::motorMsg motor_vel;
        nav_msgs::Odometry error_msg;
            // Unit conversion
        cmd_motor_vel.w[0] = gear_ratio*radps_to_rpm*cmd_motor_vel.w[0];
        cmd_motor_vel.w[1] = -gear_ratio*radps_to_rpm*cmd_motor_vel.w[1];
        cmd_motor_vel.w[2] = -gear_ratio*radps_to_rpm*cmd_motor_vel.w[2];
        cmd_motor_vel.w[3] = gear_ratio*radps_to_rpm*cmd_motor_vel.w[3];
                         
        cmd_motor_vel = clamping(cmd_motor_vel);

        if(sqrt((x_goal-x_robot)*(x_goal-x_robot)+(y_goal-y_robot)*(y_goal-y_robot))<0.010 && fabs(phi_robot[1]-phi_goal)<0.02 || init_pos == true){
            cmd_motor_vel.w[0] = 0;
            cmd_motor_vel.w[1] = 0;
            cmd_motor_vel.w[2] = 0;
            cmd_motor_vel.w[3] = 0;

            ax_cmd = 0;
            ay_cmd = 0;
            aphi_cmd = 0;

            vx_accumul = 0;
            vy_accumul = 0;
            vphi_accumul = 0;

                //init_pos = true;

            ROS_INFO("Stop");
            ROS_INFO("distance error : %lf phi_err : %lf",sqrt((x_goal-x_robot)*(x_goal-x_robot)+(y_goal-y_robot)*(y_goal-y_robot)),fabs(phi_robot[1]-phi_goal)*180/M_PI);
        }

        motor_vel.omega1 = (int) cmd_motor_vel.w[0];
        motor_vel.omega2 = (int) cmd_motor_vel.w[1];
        motor_vel.omega3 = (int) cmd_motor_vel.w[2];
        motor_vel.omega4 = (int) cmd_motor_vel.w[3];
            
        error_msg.pose.pose.position.x = x_err;
        error_msg.pose.pose.position.y = y_err;
        
        // Twist --> Moving average velocity to check
        error_msg.header.stamp = ros::Time::now();
        error_msg.twist.twist.linear.x = vx_cmd-vx_robot;
        error_msg.twist.twist.linear.y = vy_cmd-vy_robot;
        error_msg.twist.twist.angular.z = vphi_cmd-vphi_robot;

        //Publish control input msg
        publisher_cmd_vel.publish(motor_vel);
        publisher_error.publish(error_msg);

        last_time = ros::Time::now().toSec();
    }

    motor_vel inverse_kinematics(){
        motor_vel m;

        m.w[0] = (int) 1.0/r * vx_cmd - 1.0/r * vy_cmd - l/r * vphi_cmd;
        m.w[1] = (int) 1.0/r * vx_cmd + 1.0/r * vy_cmd + l/r * vphi_cmd;
        m.w[2] = (int) 1.0/r * vx_cmd - 1.0/r * vy_cmd + l/r * vphi_cmd;
        m.w[3] = (int) 1.0/r * vx_cmd + 1.0/r * vy_cmd - l/r * vphi_cmd;

        return m;
    }

    motor_vel clamping(motor_vel clamped_motor_vel){

        double arr[4] = {clamped_motor_vel.w[0],clamped_motor_vel.w[1],clamped_motor_vel.w[2],clamped_motor_vel.w[3]};

        double vx_;
        double vy_;
        double vphi_;

        double motor_vel_max = 0;
        std::sort(arr,arr+4,std::greater<double>());
        motor_vel_max = arr[0];
        
        if(fabs(motor_vel_max)>motor_vel_lim){
            clamped_motor_vel.w[0] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[0];
            clamped_motor_vel.w[1] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[1];
            clamped_motor_vel.w[2] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[2];
            clamped_motor_vel.w[3] = (double) motor_vel_lim/fabs(motor_vel_max) * clamped_motor_vel.w[3];
        }
        
        vx_ = r/4.0 * (clamped_motor_vel.w[0] + clamped_motor_vel.w[1] + clamped_motor_vel.w[2] + clamped_motor_vel.w[0])/gear_ratio*rpm_to_radps;
        vy_ = r/4.0 * (-clamped_motor_vel.w[0] + clamped_motor_vel.w[1] - clamped_motor_vel.w[2] + clamped_motor_vel.w[0])/gear_ratio*rpm_to_radps;
        vphi_ = r/4.0/l * (-clamped_motor_vel.w[0] + clamped_motor_vel.w[1] + clamped_motor_vel.w[2] - clamped_motor_vel.w[0])/gear_ratio*rpm_to_radps;
        
        //ROS_INFO("vx_cmd : %lf vy_cmd : %lf vphi_cmd : %lf",vx_,vy_,vphi_);
        //ROS_INFO("Robot vel - vx : %lf, vy : %lf vphi : %lf",vx_,vy_,vphi_);
        //ROS_INFO("vphi_ : %lf vphi_des : %lf phi_err : %lf",vphi_,vphi_des,phi_err);
        return clamped_motor_vel;
        
    }

    void publish_slam_pose(){
        nav_msgs::Odometry slam_pose;

        slam_pose.header.stamp = ros::Time::now();
        // Get SLAM position
        slam_pose.pose.pose.position.x = x_robot;
        slam_pose.pose.pose.position.y = y_robot;
    
        // Get SLAM orientation
        geometry_msgs::Quaternion slam_quat = tf::createQuaternionMsgFromYaw(phi_robot[1]);
        slam_pose.pose.pose.orientation = slam_quat;

        publisher_slam_pose.publish(slam_pose);
    }

    void desired_traj(){
        nav_msgs::Odometry des_traj;
        des_traj.header.stamp = ros::Time::now();
        des_traj.pose.pose.position.x = x_des;
        des_traj.pose.pose.position.y = y_des;
        des_traj.twist.twist.linear.x = vx_des;
        des_traj.twist.twist.linear.y = vy_des;
        des_traj.twist.twist.angular.z = vphi_des;
        // Publish Desired Trajectory 
        publisher_desired_traj.publish(des_traj);
    }

    // Callback function 1 : Trajectory from planner
    void callback_traj(const TrajectoryPointMsg::ConstPtr& traj_msg){


        // Trajectory generation --> TrajectoryPointMsg
        // Teb planner --> FeedbackMsg
        // Robot index

        /**
        robot_index = traj_msg -> selected_trajectory_idx;
        
        int traj_index = 1;
        bool check_goal_message = false;

        
        if(!traj_msg->trajectories.empty())
        {
            if(check_goal_message == false)
            {
                check_goal_message = true;
            }
            init_pos = false;
        }

        // Desired velocity
        vx_des = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.linear.x;
        vy_des = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.linear.y;
        vphi_des = traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.angular.z;

        // Desired position
        x_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.position.x;
        y_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.position.y;
        
        // Desired orientation
        qw_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.w;
        qx_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.x;
        qy_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.y;
        qz_des = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.z;
        
        **/        

        //Trajectory Generation 

        ax_des = traj_msg -> acceleration.linear.x;
        ay_des = traj_msg -> acceleration.linear.y;
        aphi_des = traj_msg -> acceleration.angular.z;
        
        vx_des = traj_msg -> velocity.linear.x;
        vy_des = traj_msg -> velocity.linear.y;

        x_des = traj_msg -> pose.position.x;
        y_des = traj_msg -> pose.position.y;

        qx_des = traj_msg -> pose.orientation.x;
        qy_des = traj_msg -> pose.orientation.y;
        qz_des = traj_msg -> pose.orientation.z;
        qw_des = traj_msg -> pose.orientation.w;

        phi_des[0] = phi_des[1];
        // Conversion from Quaternion to euler angle
        double siny_cosp = 2.0 * (qw_des*qz_des + qx_des*qy_des);
        double cosy_cosp = 1 - 2.0 * (qy_des*qy_des + qz_des*qz_des);
        phi_des[1] = atan2(siny_cosp,cosy_cosp);
        multi_turn_phi_des = multi_turn_angle(phi_des,&index_multi_turn_phi_des);
    }

    // Callback function 2 : State
    void callback_state(const Odometry::ConstPtr& state_msg){

        phi_robot[0] = phi_robot[1];
        // Conversion from Quaternion to euler angle
        double siny_cosp = 2.0 * (qw*qz + qx*qy);
        double cosy_cosp = 1 - 2.0 * (qy*qy + qz*qz);
        phi_robot[1] = atan2(siny_cosp,cosy_cosp);
        multi_turn_phi_robot = multi_turn_angle(phi_robot,&index_multi_turn_phi_robot);
    }

    // Callback function 3 : Goal
    void callback_goal(const PoseStamped::ConstPtr& goal_msg){

        x_goal = goal_msg->pose.position.x;
        y_goal = goal_msg->pose.position.y;

        ROS_INFO("Goal Recieved");

        init_pos = false;

        double qw = goal_msg ->pose.orientation.w;        
        double qx = goal_msg ->pose.orientation.x;
        double qy = goal_msg ->pose.orientation.y;
        double qz = goal_msg ->pose.orientation.z;
        // Conversion from Quaternion to euler angle
        double siny_cosp = 2.0 * (qw*qz + qx*qy);
        double cosy_cosp = 1 - 2.0 * (qy*qy + qz*qz);
        phi_goal = atan2(siny_cosp,cosy_cosp);
    }


    double multi_turn_angle(double yaw[2], int* multi_turn_num){
        double multi_turn_angle;
        if(yaw[0]>M_PI/2.0 && yaw[1]<-M_PI/2.0){
            //multi_turn_angle = yaw[1] + M_PI + (2.0 * (*multi_turn_num) + 1)*M_PI;
            (*multi_turn_num) = (*multi_turn_num) + 1;
        }

        if(yaw[0]<-M_PI/2.0 && yaw[1]>M_PI/2.0){
            //multi_turn_angle = yaw[1] - M_PI + (2.0 * (*multi_turn_num) + 1)*M_PI;
            (*multi_turn_num) = (*multi_turn_num) - 1;
        }
        if(yaw[0]*yaw[1]>=0){
            multi_turn_angle = yaw[1] + 2.0 * (*multi_turn_num) * M_PI;
        }
        if(yaw[0]*yaw[1]<=0){
            multi_turn_angle = yaw[1] + 2.0 * (*multi_turn_num) * M_PI;
        }

        return multi_turn_angle;
    }


    private:

    ros::NodeHandle nh_;
    tf::TransformListener listener_;

    ros::Subscriber subscriber_trajectory;
    ros::Subscriber subscriber_state;
    ros::Subscriber subscriber_goal;

    ros::Publisher publisher_cmd_vel;
    ros::Publisher publisher_desired_traj;
    ros::Publisher publisher_slam_pose;
    ros::Publisher publisher_error;

    // Time info
    double curr_time;
    double last_time = ros::Time::now().toSec();
    double dt = 0;
            

};
